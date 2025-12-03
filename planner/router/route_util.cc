#include "onboard/planner/router/route_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <float.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <numeric>
#include <ostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/types/span.h"
#include "boost/geometry/algorithms/simplify.hpp"
#include "boost/geometry/core/cs.hpp"
#include "boost/geometry/geometries/register/linestring.hpp"
#include "boost/geometry/geometries/register/point.hpp"
#include "glog/logging.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_const_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/geometry/gfc.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/router/util/route_extend_path.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

BOOST_GEOMETRY_REGISTER_POINT_2D(
    qcraft::Vec2d, double,
    ::boost::geometry::cs::cartesian, operator[](0), operator[](1));
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<qcraft::Vec2d>);

namespace qcraft {
namespace planner {

namespace {

constexpr double kMaxRetrogradeMeters = -1.0;
constexpr double kMinForwardSpeed = 0.5;
constexpr double kEpsilon = 1e-9;

// Reference: https://qcraft.feishu.cn/file/AbUlbnO4boFL0vx3pg1caYIbnfb,
// Table 1.
const PiecewiseConstFunction<double, double> kRadiusToSpeedLimitKph(
    {0.0, 20.0, 30.0, 60.0, 125.0}, {20.0, 40.0, 50.0, 60.0});

inline Vec3d ComputePerpBisectorCoeff(const Vec2d& p1, const Vec2d& p2) {
  // ax + by + c = 0, Vec3d(a, b, c).
  const double a = p1.y() - p2.y();
  const double b = p2.x() - p1.x();
  const Vec2d mid_point = (p1 + p2) * 0.5;
  return Vec3d(b, -a, a * mid_point.y() - b * mid_point.x());
}

}  // namespace

namespace internal {

std::vector<mapping::ElementId> ToElementIds(
    absl::Span<const std::int64_t> ids) {
  std::vector<mapping::ElementId> results;
  results.reserve(ids.size());
  for (auto id : ids) {
    results.push_back(mapping::ElementId(id));
  }
  return results;
}

// Convert different types of routing destination to global point
// representation.
absl::StatusOr<mapping::GeoPointProto> ConvertToGlobal(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const RoutingDestinationProto& dest) {
  mapping::GeoPointProto global_point;
  if (dest.has_global_point()) {
    global_point = dest.global_point();
  } else if (dest.has_lane_point()) {
    const auto lane_pt = mapping::LanePoint(dest.lane_point());
    SMM_LANE_PROTO_OR_RETURN(lane_proto, semantic_map_manager,
                             lane_pt.lane_id(),
                             absl::NotFoundError(absl::StrCat(
                                 "Cannot find lane:", lane_pt.lane_id())));
    const auto pt3d =
        ComputePointOnLane(lane_proto->polyline().points(), lane_pt.fraction());
    global_point.set_longitude(pt3d.x());
    global_point.set_latitude(pt3d.y());
    global_point.set_altitude(pt3d.z());
  } else {
    const mapping::NamedSpotProto* named_spot = nullptr;
    for (const auto& spot :
         semantic_map_manager.semantic_map_proto().named_spots()) {
      if (spot.name() == dest.named_spot()) {
        named_spot = &spot;
        break;
      }
    }
    if (named_spot == nullptr) {
      return absl::InvalidArgumentError(
          absl::StrCat("Could not find the named spot ", dest.named_spot(),
                       ", please check your semantic map version."));
    }
    global_point = named_spot->point();
  }
  return global_point;
}
}  // namespace internal

std::vector<Vec2d> GlobalToSmoothPoints(
    const CoordinateConverter& cc,
    const google::protobuf::RepeatedPtrField<mapping::GeoPointProto>&
        points_global) {
  std::vector<Vec2d> points_smooth;
  points_smooth.reserve(points_global.size());
  for (const auto& pt : points_global) {
    points_smooth.push_back(cc.GlobalToSmooth({pt.longitude(), pt.latitude()}));
  }
  return points_smooth;
}

absl::StatusOr<mapping::LanePoint> ParseDestinationProtoToLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index,
    const RoutingDestinationProto& destination_proto, double min_distance,
    RouteMatchType match_type) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(map_index);
  if (destination_proto.has_off_road()) {
    return mapping::LanePoint{mapping::kInvalidElementId, 0.0};
  }
  if (destination_proto.has_global_point() ||
      destination_proto.has_named_spot()) {
    Vec3d global;
    if (!destination_proto.has_global_point() &&
        destination_proto.has_named_spot()) {
      const auto& map = semantic_map_manager.semantic_map_proto();
      const mapping::NamedSpotProto* named_spot = nullptr;
      for (const auto& spot : map.named_spots()) {
        if (spot.name() == destination_proto.named_spot()) {
          named_spot = &spot;
        }
      }
      if (named_spot == nullptr) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Could not find the named spot ", destination_proto.named_spot()));
      }
      global = {named_spot->point().longitude(), named_spot->point().latitude(),
                named_spot->point().altitude()};
    } else {
      global = {destination_proto.global_point().longitude(),
                destination_proto.global_point().latitude(),
                destination_proto.global_point().altitude()};
    }
    std::vector<mapping::PointToLane> point_to_lanes;
    mapping::LevelId level;
    auto level_lanes_pair = PointToNearLanesWithInferLevel(
        semantic_map_manager, *map_index, {global.x(), global.y()}, global.z(),
        min_distance);
    level = level_lanes_pair.first;
    point_to_lanes = std::move(level_lanes_pair.second);
    VLOG(3) << "Infer level:" << level
            << ", lanes count:" << point_to_lanes.size();
    if (FLAGS_route_match_filter_enable) {
      for (const mapping::PointToLane& ptlane : point_to_lanes) {
        if (ptlane.lane_proto == nullptr) {
          QLOG(ERROR) << "Invalid PointToNearLanesAtLevel return nullptr.";
          continue;
        } else if (match_type == RouteMatchType::STATIONARY &&
                   mapping::IsPassengerVehicleAvoidLaneType(
                       ptlane.lane_proto->type())) {
          QLOG(ERROR)
              << "Invalid destination_proto, too close to non-motorway. "
                 "destination:"
              << destination_proto.ShortDebugString()
              << ", match lane:" << ptlane.lane_proto->id();
          continue;
        } else {
          return mapping::LanePoint{mapping::ElementId(ptlane.lane_proto->id()),
                                    ptlane.fraction};
        }
      }
    } else if (point_to_lanes.size() > 0) {
      return mapping::LanePoint{
          mapping::ElementId(point_to_lanes.front().lane_proto->id()),
          point_to_lanes.front().fraction};
    }
    return absl::NotFoundError(absl::StrCat(
        "Could not found lane id from destination proto: ",
        destination_proto.ShortDebugString(),
        absl::StrFormat("lonlat:{ %.8f %.8f} ", global.x(), global.y()),
        ", Level:", level));
  } else if (destination_proto.has_lane_point()) {
    const auto lp = mapping::LanePoint(destination_proto.lane_point());
    if (!lp.Valid()) {
      return absl::InvalidArgumentError(
          absl::StrCat("The provided lane point is invalid: ",
                       destination_proto.ShortDebugString()));
    } else {
      return lp;
    }
  } else {
    return absl::InvalidArgumentError(absl::StrCat(
        "Invalid routing request,", destination_proto.ShortDebugString()));
  }
}

absl::StatusOr<std::vector<mapping::LanePoint>>
ParseDestinationProtoToLanePoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index,
    const RoutingRequestProto& routing_request) {
  FUNC_QTRACE();
  std::vector<mapping::LanePoint> destinations;
  destinations.reserve(routing_request.destinations_size());
  for (const RoutingDestinationProto& destination_proto :
       routing_request.destinations()) {
    // TODO(xiang): Use RouteParamProto as parameter instead.
    const auto lane_point_or = ParseDestinationProtoToLanePoint(
        semantic_map_manager, map_index, destination_proto);
    if (!lane_point_or.ok()) return lane_point_or.status();
    destinations.push_back(lane_point_or.value());
  }
  if (destinations.empty()) {
    return absl::NotFoundError(
        absl::StrCat("No route destination found from request: ",
                     routing_request.ShortDebugString()));
  }
  return destinations;
}

absl::StatusOr<RoutingRequestProto> ConvertRoutingRequestToGlobalPoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const RoutingRequestProto& raw_routing_request) {
  RoutingRequestProto proto;
  *proto.mutable_avoid_lanes() = raw_routing_request.avoid_lanes();
  *proto.mutable_lane_cost_modifiers() =
      raw_routing_request.lane_cost_modifiers();

  if (raw_routing_request.has_multi_stops()) {
    proto.mutable_multi_stops()->set_infinite_loop(
        raw_routing_request.multi_stops().infinite_loop());

    for (const auto& raw_stop : raw_routing_request.multi_stops().stops()) {
      MultipleStopsRequestProto::StopProto stop_global;
      stop_global.set_stop_name(raw_stop.stop_name());
      if (raw_stop.has_stop_point()) {
        ASSIGN_OR_RETURN(auto global_point,
                         internal::ConvertToGlobal(semantic_map_manager,
                                                   raw_stop.stop_point()));
        *stop_global.mutable_stop_point()->mutable_global_point() =
            std::move(global_point);

      } else {
        RoutingDestinationProto tmp_dest;
        *tmp_dest.mutable_named_spot() = raw_stop.stop_name();
        ASSIGN_OR_RETURN(
            auto global_point,
            internal::ConvertToGlobal(semantic_map_manager, tmp_dest));
        *stop_global.mutable_stop_point()->mutable_global_point() =
            std::move(global_point);
      }

      for (const auto& via_p : raw_stop.via_points()) {
        ASSIGN_OR_RETURN(auto global_point, internal::ConvertToGlobal(
                                                semantic_map_manager, via_p));
        *stop_global.add_via_points()->mutable_global_point() =
            std::move(global_point);
      }

      *proto.mutable_multi_stops()->add_stops() = std::move(stop_global);
    }
  }

  for (const auto& dest : raw_routing_request.destinations()) {
    ASSIGN_OR_RETURN(auto global_point,
                     internal::ConvertToGlobal(semantic_map_manager, dest));
    *proto.add_destinations()->mutable_global_point() = std::move(global_point);
  }

  return proto;
}

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, mapping::ElementId start_lane_id,
    double start_frac, double extend_len) {
  if (extend_len <= 0.0) return {};

  const auto& sections = sections_info.section_segments();
  const auto& lane_navi_info_map = route_navi_info.route_lane_info_map;
  std::vector<mapping::ElementId> lane_ids;
  mapping::ElementId cur_lane_id = start_lane_id;
  double cur_frac = start_frac;
  if (cur_frac == 1.0) lane_ids.push_back(cur_lane_id);
  int sec_idx = 0;
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, cur_lane_id);
    if (cur_frac == 1.0) {
      if (++sec_idx == sections.size()) break;
      const auto& next_sec = sections[sec_idx];
      mapping::ElementId outgoing_lane_id = mapping::kInvalidElementId;
      double outgoing_lane_cross_prod = 1.0;
      int min_lc_num = std::numeric_limits<int>::max();
      for (const auto next_lane_id : lane_info.outgoing_lanes()) {
        if (next_sec.id_idx_map.contains(next_lane_id)) {
          const auto* lane_navi_info_ptr =
              FindOrNull(lane_navi_info_map, next_lane_id);
          const double lc_num = lane_navi_info_ptr == nullptr
                                    ? std::numeric_limits<int>::max()
                                    : lane_navi_info_ptr->min_lc_num_to_target;
          if (lc_num < min_lc_num) {
            outgoing_lane_id = next_lane_id;
            min_lc_num = lc_num;
            if (!lane_ids.empty()) {
              const auto next_cross_prod_or = CalcOutgoingLaneCrossProd(
                  psmm, lane_ids.back(), next_lane_id);
              if (next_cross_prod_or.ok()) {
                outgoing_lane_cross_prod = *next_cross_prod_or;
              }
            }
          } else if (lc_num == min_lc_num && !lane_ids.empty()) {
            const auto cross_prod_or =
                CalcOutgoingLaneCrossProd(psmm, lane_ids.back(), next_lane_id);
            if (cross_prod_or.ok() && std::fabs(*cross_prod_or) <
                                          std::fabs(outgoing_lane_cross_prod)) {
              outgoing_lane_cross_prod = *cross_prod_or;
              outgoing_lane_id = next_lane_id;
            }
          }
        }
      }
      if (outgoing_lane_id == mapping::kInvalidElementId) break;
      cur_lane_id = outgoing_lane_id;
      cur_frac = 0.0;
    } else {
      lane_ids.push_back(cur_lane_id);
      const double lane_len = lane_info.length();
      if (sec_idx + 1 == sections.size()) {
        cur_frac = std::min(sections.back().end_fraction,
                            extend_len / lane_len + cur_frac);
        break;
      }
      const double len = lane_len * (sections[sec_idx].end_fraction - cur_frac);
      if (len >= extend_len) {
        cur_frac += extend_len / lane_len;
        break;
      }
      extend_len -= len;
      cur_frac = 1.0;
    }
  }
  return mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                           start_frac, cur_frac);
}

absl::StatusOr<RoutingRequestProto>
ConvertAllRouteFlagsToRoutingRequestProto() {
  RoutingRequestProto routing_request;
  StringToProtoLogCollector log_collector;

  if (!FLAGS_route.empty()) {
    bool from_route_flags =
        StringToProtoLogDetail(FLAGS_route, &routing_request, &log_collector);
    if (!from_route_flags) {
      QLOG(ERROR) << "Failed to parse routing request, "
                  << log_collector.GetLastParseError().DebugString();
      QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_ROUTING_ROUTE_RECOVER_FAILED,
              "Router initialization from FLAGS_route.");
      return absl::InvalidArgumentError(FLAGS_route);
    }
    QLOG(INFO) << "Create default routing request by " << FLAGS_route;
    return routing_request;
  }

  if (!FLAGS_multi_stops_route.empty()) {
    MultipleStopsRequestProto multiple_stops_request;
    bool from_stops_flag = StringToProtoLogDetail(
        FLAGS_multi_stops_route, &multiple_stops_request, &log_collector);
    if (!from_stops_flag) {
      QLOG(ERROR) << "Failed to parse routing request, "
                  << log_collector.GetLastParseError().DebugString();
      QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_ROUTING_ROUTE_RECOVER_FAILED,
              "Router initialization from FLAGS_multi_stops_route.");

      return absl::InvalidArgumentError(FLAGS_multi_stops_route);
    }
    QLOG(INFO) << "Create default routing request by "
               << FLAGS_multi_stops_route;
    *routing_request.mutable_multi_stops() = multiple_stops_request;
    return routing_request;
  }

  if (!FLAGS_route_str.empty()) {
    QLOG(INFO) << "Create default routing request by " << FLAGS_route_str;
    const auto tokens = absl::StrSplit(FLAGS_route_str, ',');
    std::vector<std::string> named_spots(tokens.begin(), tokens.end());
    if (FLAGS_random_route) {
      // Read random seed from /dev/random.
      std::random_device rd("/dev/random");
      std::mt19937 g(rd());
      std::shuffle(named_spots.begin(), named_spots.end(), g);
    }
    for (const auto& named_spot : named_spots) {
      if (named_spot.empty()) continue;
      routing_request.add_destinations()->set_named_spot(
          std::string(named_spot));
    }
    return routing_request;
  }

  return absl::NotFoundError(
      "Routing request is empty or invalid, please check it again.");
}

absl::flat_hash_set<mapping::ElementId> FindAvoidLanesFromAvoidRegions(
    const RoutingRequestProto& routing_request,
    const mapping::v2::SemanticMapManager& semantic_map_manager) {
  if (routing_request.avoid_regions().empty()) return {};

  absl::flat_hash_set<mapping::ElementId> avoid_lanes_in_regions;

  std::vector<Polygon2d> polygons;
  polygons.reserve(routing_request.avoid_regions_size());
  for (const auto& avoid_region : routing_request.avoid_regions()) {
    auto points = mapping::ConverterGeoPoints(avoid_region.points());
    polygons.push_back(Polygon2d(std::move(points)));
  }
  for (const auto& lane : semantic_map_manager.semantic_map().lanes) {
    for (const auto& polygon : polygons) {
      bool lane_in_polygon = true;
      for (const auto& pt : lane->Proto().polyline().points()) {
        if (!polygon.IsPointIn({pt.longitude(), pt.latitude()})) {
          lane_in_polygon = false;
          break;
        }
      }
      if (lane_in_polygon) {
        avoid_lanes_in_regions.insert(mapping::ElementId(lane->Proto().id()));
      }
    }
  }
  return avoid_lanes_in_regions;
}

/// @brief Cannot keep self-intersect,max_distance(epsilon,unit=meter), use
/// boost:geometry doglas-peuker,
/// @return Global coordinate  points seq (rad)
/// @see mapping::SampleLanePathPoints
absl::StatusOr<std::vector<Vec2d>> SimplifyPathPoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePathProto& composite_lane_path,
    double max_distance_meters) {
  if (max_distance_meters < 0.0) {
    return absl::InvalidArgumentError("distance must GE 0");
  }
  std::vector<Vec2d> points;
  constexpr double kApproximateMeterToRadRatio = 1.11E-5 * M_PI / 180;
  constexpr double kMinFractionDistance = 0.001;
  for (const auto& lane_path : composite_lane_path.lane_paths()) {
    if (lane_path.lane_ids().size() == 1 &&
        (lane_path.end_fraction() - lane_path.start_fraction()) <
            kMinFractionDistance) {
      continue;
    }
    ASSIGN_OR_RETURN(
        const auto sample_result,
        SampleLanePathProtoPoints(semantic_map_manager, lane_path));
    if (sample_result.is_partial) {
      QLOG(INFO) << "The partial result is not allowed, "
                 << sample_result.message;
      return absl::NotFoundError(sample_result.message);
    }
    const auto& global_points = sample_result.points;
    if (global_points.empty()) {
      continue;
    }
    std::vector<Vec2d> simplified;
    boost::geometry::simplify(
        global_points, simplified,
        max_distance_meters * kApproximateMeterToRadRatio);
    const int simplified_size = simplified.size();
    points.insert(points.end(), simplified.begin(), simplified.end());
    VLOG(2) << " Simplify, source points num:" << global_points.size()
            << ", after:" << simplified_size
            << ", total points:" << points.size();
  }
  return points;
}

bool HasOnlyOneOffRoadRequest(const RoutingRequestProto& routing_request) {
  return (routing_request.destinations_size() == 1 &&
          routing_request.destinations(0).has_off_road()) ||
         (routing_request.has_multi_stops() &&
          routing_request.multi_stops().stops_size() == 1 &&
          IsOffRoadDestination(routing_request.multi_stops().stops(0)));
}

std::string PoseDebugString(const CoordinateConverter& coordinate_converter,
                            const PoseProto& pose) {
  const Vec3d pos = Vec3dFromProto(pose.pos_smooth());
  const Vec3d global = coordinate_converter.SmoothToGlobal(pos);
  return absl::StrFormat("{x: %.8f, y: %.8f, heading: %.2f, speed: %.2f}",
                         global.x(), global.y(), pose.yaw(),
                         pose.vel_body().x());
}

bool IsTravelMatchedValid(const CompositeLanePath::CompositeIndex& first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex& second,
                          double second_fraction, int64_t last_time_micros,
                          int64_t cur_time_micros, double distance,
                          double speed, double reroute_threshold_meters,
                          double estimate_speed) {
  VLOG(2) << "first" << first.DebugString() << ", second"
          << second.DebugString() << ", first_fraction:" << first_fraction
          << ", second_fraction:" << second_fraction
          << ", last_time_micros:" << last_time_micros
          << ", cur_time_micros:" << cur_time_micros
          << ", current speed:" << speed
          << ", estimate_speed:" << estimate_speed;
  if (last_time_micros == -1) {
    return true;
  }
  // Retrograde is not allowed, forward direction
  if (speed >= kMinForwardSpeed && distance <= kMaxRetrogradeMeters) {
    VLOG(2) << "Retrograde is not allowed.distance:" << distance
            << ", max:" << kMaxRetrogradeMeters << ", speed:" << speed;
    return false;
  }
  constexpr double kAllowedDriftThres = 35.0 * 0.5;  // 35m/s*0.5s
  const double reroute_threshold =
      std::min(reroute_threshold_meters,
               MicroSecondsToSeconds(cur_time_micros - last_time_micros) *
                   estimate_speed);
  VLOG(2) << "RerouteThresholdMeters:" << reroute_threshold_meters
          << ", reroute_threshold:" << reroute_threshold
          << ", drift_allowed: " << kAllowedDriftThres
          << ", distance:" << distance;
  return distance <= std::max(reroute_threshold, kAllowedDriftThres);
}

absl::StatusOr<std::vector<SectionLanesQueueItem>> SearchConnectLaneInSection(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    mapping::ElementId scr_lane_id, mapping::ElementId target_lane_id,
    bool search_direction_left) {
  std::vector<std::unique_ptr<SectionLanesQueueItem>> allocated_items;
  std::deque<SectionLanesQueueItem*> queue;
  auto root = std::make_unique<SectionLanesQueueItem>();
  root->id = scr_lane_id;
  root->prev = nullptr;
  root->type = mapping::LaneBoundaryProto::UNKNOWN_TYPE;
  allocated_items.push_back(std::move(root));
  queue.push_back(allocated_items.back().get());
  bool found = false;
  SectionLanesQueueItem* cur = nullptr;
  while (!queue.empty()) {
    cur = queue.front();
    mapping::ElementId cur_id = cur->id;
    queue.pop_front();
    if (cur_id != target_lane_id) {
      SMM_LANE_PROTO_OR_CONTINUE(lane_proto, semantic_map_manager, cur_id);
      const auto& neighbors = search_direction_left
                                  ? lane_proto->lane_neighbors_on_left()
                                  : lane_proto->lane_neighbors_on_right();
      for (const auto& neighbour : neighbors) {
        auto new_item = std::make_unique<SectionLanesQueueItem>();
        new_item->id = mapping::ElementId(neighbour.other_id());
        new_item->prev = cur;
        new_item->type = GetLaneNeighborBoundary(neighbour);
        allocated_items.push_back(std::move(new_item));
        queue.push_back(allocated_items.back().get());
      }
    } else {
      found = true;
      break;
    }
  }
  if (!found) {
    return absl::NotFoundError("Cannot find connect lane");
  }
  std::vector<SectionLanesQueueItem> results;
  for (; cur != nullptr; cur = cur->prev) {
    results.push_back(*cur);
  }
  return results;
}

bool IsTwoLaneConnectedWithBrokenWhites(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    mapping::ElementId src, mapping::ElementId target) {
  const auto connect_left_or = SearchConnectLaneInSection(
      semantic_map_manager, src, target, /*search_direction_left=*/true);
  if (connect_left_or.ok()) {
    for (const SectionLanesQueueItem& item : connect_left_or.value()) {
      if (item.type != mapping::LaneBoundaryProto::UNKNOWN_TYPE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_WHITE) {
        VLOG(2) << "id:" << item.id << ",type:" << item.type << ",left";
        return false;
      }
    }
    return true;
  }
  const auto connect_right_or = SearchConnectLaneInSection(
      semantic_map_manager, src, target, /*search_direction_left=*/false);
  if (connect_right_or.ok()) {
    for (const SectionLanesQueueItem& item : connect_right_or.value()) {
      if (item.type != mapping::LaneBoundaryProto::UNKNOWN_TYPE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE &&
          item.type != mapping::LaneBoundaryProto::BROKEN_WHITE) {
        VLOG(2) << "id:" << item.id << ",type:" << item.type << ",right";
        return false;
      }
    }
    return true;
  }
  return false;
}

bool CrossSolidBoundary(mapping::LaneBoundaryProto::Type type, bool lc_left) {
  switch (type) {
    case mapping::LaneBoundaryProto::UNKNOWN_TYPE:
    case mapping::LaneBoundaryProto::VIRTUAL:
    case mapping::LaneBoundaryProto::BROKEN_WHITE:
    case mapping::LaneBoundaryProto::BROKEN_YELLOW:
    case mapping::LaneBoundaryProto::BROKEN_DOUBLE_YELLOW:
    case mapping::LaneBoundaryProto::BROKEN_DOUBLE_WHITE:
      return false;
    case mapping::LaneBoundaryProto::SOLID_WHITE:
    case mapping::LaneBoundaryProto::SOLID_YELLOW:
    case mapping::LaneBoundaryProto::SOLID_DOUBLE_YELLOW:
    case mapping::LaneBoundaryProto::CURB:
    case mapping::LaneBoundaryProto::SOLID_DOUBLE_WHITE:
      return true;
    case mapping::LaneBoundaryProto::BROKEN_LEFT_DOUBLE_WHITE:
      return lc_left;
    case mapping::LaneBoundaryProto::BROKEN_RIGHT_DOUBLE_WHITE:
      return !lc_left;
  }
}

// @see onboard/maps/semantic_map_manager.cc:InferLevelIdFromNearbyLanes
// TODO(xiang): smelly code. refactor it!!
std::pair<mapping::LevelId, std::vector<mapping::PointToLane>>
PointToNearLanesWithInferLevel(
    [[maybe_unused]] const mapping::v2::SemanticMapManager&
        semantic_map_manager,
    const route::MapIndex& index, const Vec2d& global, double z,
    double min_distance) {
  FUNC_QTRACE();
  constexpr double kMaxDistance = 15.0;  // ignore data whose dist > 15m

  // To determine if a point is on the level, the ground altitude of the level
  // must be lower than altitude + kLevelAllowance
  constexpr double kLevelZAllowance = 2.0;
  const double max_altitude = z + kLevelZAllowance;
  double min_delta_z = std::numeric_limits<double>::max();
  std::vector<mapping::PointToLane> all_lanes = index.PointToNearLanes(
      global, kMaxDistance, [](const auto*) { return true; });

  std::sort(all_lanes.begin(), all_lanes.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.dist < rhs.dist ||
                     (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
            });
  VLOG(3) << "begin inferring level by near lanes.";

  mapping::LevelId inferred_level = mapping::kMapLevel0;
  if (all_lanes.size() > 0 &&
      !all_lanes[0].lane_proto->belonging_levels().empty()) {
    inferred_level =
        mapping::LevelId(all_lanes[0].lane_proto->belonging_levels()[0]);
  }
  for (const auto& pt_lane : all_lanes) {
    const auto* lane = pt_lane.lane_proto;
    if (lane->belonging_levels().empty()) {
      break;
    }
    double nearest_point_alt;
    double lane_min_dist = std::numeric_limits<double>::max();
    bool valid_nearest_point_found = false;
    for (const auto& geo_pt : lane->polyline().points()) {
      const auto altitude = geo_pt.altitude();
      if (altitude > max_altitude) {
        continue;
      }
      const auto dist_pt = route::HaversineDistance(
          {geo_pt.longitude(), geo_pt.latitude()}, global);
      if (dist_pt > kMaxDistance) {
        continue;
      } else if (dist_pt < lane_min_dist) {
        lane_min_dist = dist_pt;
        nearest_point_alt = altitude;
        valid_nearest_point_found = true;
      }
    }
    if (valid_nearest_point_found &&
        std::fabs(nearest_point_alt - z) < min_delta_z) {
      min_delta_z = std::fabs(nearest_point_alt - z);
      inferred_level = mapping::LevelId(lane->belonging_levels()[0]);
    }
  }

  std::vector<mapping::PointToLane> filter_lanes;
  filter_lanes.reserve(all_lanes.size());
  for (const auto& pt_lane : all_lanes) {
    const auto* lane = pt_lane.lane_proto;
    const auto& levels = lane->belonging_levels();
    VLOG(3) << "lane_id: " << lane->id() << ",dist:" << pt_lane.dist
            << ", levels:" << absl::StrJoin(levels, ",");
    if ((levels.empty() || std::find(levels.begin(), levels.end(),
                                     inferred_level.value()) != levels.end()) &&
        pt_lane.dist <= min_distance) {
      filter_lanes.push_back(pt_lane);
    }
  }
  VLOG(3) << "point:" << global.DebugStringFullPrecision() << " smooth:"
          << "z: " << z << ",inferred_level:" << inferred_level
          << ",lanes:" << filter_lanes.size();
  return std::make_pair(inferred_level, std::move(filter_lanes));
}

std::vector<mapping::PointToLane> PointToNearLanesAtLevel(
    const route::MapIndex& map_index, mapping::LevelId level,
    const Vec2d& global, double min_distance) {
  FUNC_QTRACE();
  auto match_lanes = map_index.PointToNearLanes(
      global, min_distance, [level](const auto* lane_proto) {
        const auto& levels = lane_proto->belonging_levels();
        return levels.empty() || std::find(levels.begin(), levels.end(),
                                           level.value()) != levels.end();
      });
  std::sort(match_lanes.begin(), match_lanes.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.dist < rhs.dist ||
                     (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
            });
  return match_lanes;
}

// Can be optimized into MapIndex match class.
std::vector<mapping::PointToLane> PointToNearLanesWithHeadingAtLevel(
    const route::MapIndex& map_index, mapping::LevelId level,
    const Vec2d& global, double max_distance, double heading,
    double max_heading_diff) {
  FUNC_QTRACE();
  auto match_lanes = map_index.PointToNearLanes(
      global, max_distance,
      [level](const auto* lane_proto) {
        const auto& levels = lane_proto->belonging_levels();
        return levels.empty() || std::find(levels.begin(), levels.end(),
                                           level.value()) != levels.end();
      },
      [heading, max_heading_diff](const Vec2d& p1, const Vec2d& p2) {
        const Segment2d seg(p1, p2);
        return std::fabs(NormalizeAngle(seg.heading() - heading)) <
               max_heading_diff;
      });
  std::sort(match_lanes.begin(), match_lanes.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.dist < rhs.dist ||
                     (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
            });
  return match_lanes;
}

std::vector<mapping::LanePathProto> FindAvailableSimplePaths(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    absl::Span<const mapping::PointToLane> origins,
    absl::Span<const mapping::PointToLane> dests) {
  std::vector<mapping::LanePathProto> available_paths;
  constexpr int kDefaultMaxPathCount = 4;
  available_paths.reserve(kDefaultMaxPathCount);
  for (const auto& origin : origins) {
    for (const auto& dest : dests) {
      auto simple_path_or = route::FindConnectedLanePathWithin(
          semantic_map_manager,
          {mapping::ElementId(origin.lane_proto->id()), origin.fraction},
          {mapping::ElementId(dest.lane_proto->id()), dest.fraction},
          /*allow_distance=*/100.0);
      if (simple_path_or.ok()) {
        available_paths.push_back(std::move(simple_path_or).value());
      }
    }
  }
  return available_paths;
}

Vec3d ComputePointOnLane(const google::protobuf::RepeatedPtrField<
                             mapping::GeoPointProto>& geo_points,
                         double fraction) {
  const auto points = mapping::ConverterGeoPoints(geo_points);
  const auto [pos, t] = mapping::ComputePosition(points, fraction);
  const auto z =
      Lerp(geo_points[pos].altitude(), geo_points[pos + 1].altitude(), t);
  return Vec3d((pos >= points.size() - 1)
                   ? points[points.size() - 1]
                   : Lerp(points[pos], points[pos + 1], t),
               z);
}

bool IsCityExpressOrHighway(mapping::SectionProto::RoadClass road_class) {
  switch (road_class) {
    case (mapping::SectionProto_RoadClass_CITY_EXPRESS):
    case (mapping::SectionProto_RoadClass_HIGHWAY):
      return true;
    case mapping::SectionProto_RoadClass_NORMAL:
      return false;
  }
  return false;
}

RouteParamProto CreateDefaultRouteParam() {
  RouteParamProto route_param_proto;
  file_util::TextFileToProto(FLAGS_route_default_params_file,
                             &route_param_proto);
  return route_param_proto;
}

double AccumRouteSectionsLengthBetween(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& sections, int start_idx, int end_idx) {
  QCHECK_GE(start_idx, 0);
  QCHECK_LE(end_idx, sections.section_id_size());
  double length = 0.0;
  for (int i = start_idx; i < end_idx; ++i) {
    const double start_frac = i == 0 ? sections.start_fraction() : 0.0;
    const double end_frac =
        i + 1 == sections.section_id_size() ? sections.end_fraction() : 1.0;
    SMM_SECTION_PROTO_OR_BREAK(sec_proto, smm, sections.section_id()[i]);
    length += sec_proto->average_length() * (end_frac - start_frac);
  }
  return length;
}

absl::StatusOr<RouteSectionsInfo> BuildRouteSectionsInfoFromData(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSections* route_sections, double planning_horizon) {
  if (route_sections == nullptr || route_sections->empty()) {
    return absl::InvalidArgumentError("Input route sections is empty!");
  }

  std::vector<RouteSectionsInfo::RouteSectionSegmentInfo> section_segs;
  section_segs.reserve(route_sections->size());
  int i = 0;
  for (; i < route_sections->size(); ++i) {
    const auto section_info =
        FindSectionProto(smm, route_sections->section_ids()[i]);
    if (section_info == nullptr) {
      section_segs.emplace_back(RouteSectionsInfo::RouteSectionSegmentInfo{
          .id = route_sections->section_ids()[i],
          .start_fraction = 0.0,
          .end_fraction = 1.0,
          .average_length = 0.0,
          .lane_ids = std::vector<mapping::ElementId>()});
      continue;
    }
    section_segs.emplace_back(RouteSectionsInfo::RouteSectionSegmentInfo{
        .id = route_sections->section_ids()[i],
        .start_fraction = 0.0,
        .end_fraction = 1.0,
        .average_length = section_info->average_length(),
        .lane_ids = internal::ToElementIds(section_info->lanes())});
  }

  // Refine first and last section fraction.
  section_segs.front().start_fraction = route_sections->start_fraction();
  section_segs.back().end_fraction = route_sections->end_fraction();

  // Build id_idx_map.
  for (auto& seg : section_segs) {
    absl::flat_hash_map<mapping::ElementId, int> id_idx_map;
    for (int i = 0; i < seg.lane_ids.size(); ++i) {
      id_idx_map.insert({seg.lane_ids[i], i});
    }
    seg.id_idx_map = std::move(id_idx_map);
  }
  return RouteSectionsInfo(route_sections, std::move(section_segs),
                           planning_horizon);
}

std::vector<Polygon2d> SampleRouteSectionsPolygons(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections) {
  std::vector<Polygon2d> polygons;
  for (int i = 0; i < route_sections.size(); ++i) {
    const double start_fraction =
        i == 0 ? route_sections.start_fraction() : 0.0;
    const double end_fraction =
        i + 1 == route_sections.size() ? route_sections.end_fraction() : 1.0;

    SMM_ASSIGN_SECTION_OR_CONTINUE(section_info, psmm,
                                   route_sections.section_ids()[i]);
    SMM_ASSIGN_LANE_OR_CONTINUE(left_lane_info, psmm,
                                section_info.lane_ids.front());
    SMM_ASSIGN_LANE_OR_CONTINUE(right_lane_info, psmm,
                                section_info.lane_ids.back());

    for (auto& polygon : mapping::SampleLaneBoundaryPolyLine(
             left_lane_info, right_lane_info, start_fraction, end_fraction)) {
      polygons.emplace_back(std::move(polygon));
    }
  }
  return polygons;
}

bool IsValidRouteOutputProto(const RouteManagerOutputProto& route_out_proto) {
  if (route_out_proto.has_route_status()) {
    return IsValidRouteOutputStatus(route_out_proto.route_status());
  }
  if (route_out_proto.has_is_valid()) {
    return route_out_proto.is_valid();
  }
  if (route_out_proto.has_destination_stop() &&
      IsOffRoadDestination(route_out_proto.destination_stop()))
    return true;
  if (route_out_proto.has_reset() && route_out_proto.reset()) {
    return true;
  }
  return route_out_proto.has_route_sections_from_current() ||
         route_out_proto.has_route_from_current();
}

bool IsValidLocalization(
    const std::shared_ptr<const LocalizationTransformProto>&
        localization_transform) {
  if (localization_transform == nullptr) return false;
  // BANDAID(zuowei): There may be not have loc status in simulation.
  if (!localization_transform->has_loc_status()) return true;
  switch (localization_transform->loc_status()) {
    case LocalizationTransformProto::kInvalid:
    case LocalizationTransformProto::kInit:
      return false;
    case LocalizationTransformProto::kLowPrecision:
    case LocalizationTransformProto::kNormal:
      return true;
  }
}

mapping::ElementId FindMostReasonableOutgoingLaneIdOrInvalid(
    const PlannerSemanticMapManager& psmm,
    const mapping::LaneInfo& source_lane_info,
    const RouteNaviInfo& route_navi_info,
    const RouteSectionsInfo::RouteSectionSegmentInfo& target_section_seg) {
  mapping::ElementId result = mapping::kInvalidElementId;
  const auto& outgoing_lanes = source_lane_info.proto->outgoing_lanes();
  if (outgoing_lanes.empty()) return result;

  std::vector<std::pair<double, const mapping::LaneInfo*>> candidate_lanes;
  candidate_lanes.reserve(target_section_seg.lane_ids.size());
  for (const auto outgoing_id : outgoing_lanes) {
    if (!target_section_seg.id_idx_map.contains(
            mapping::ElementId(outgoing_id))) {
      continue;
    }
    if (result == mapping::kInvalidElementId) {
      result = mapping::ElementId(outgoing_id);
    }
    const auto lane_info_iter = route_navi_info.route_lane_info_map.find(
        mapping::ElementId(outgoing_id));
    if (lane_info_iter == route_navi_info.route_lane_info_map.end()) {
      continue;
    }
    SMM_ASSIGN_LANE_OR_CONTINUE(out_lane_info, psmm,
                                mapping::ElementId(outgoing_id));
    candidate_lanes.push_back(
        {lane_info_iter->second.recommend_reach_length, &out_lane_info});
  }

  if (candidate_lanes.empty()) return result;

  std::stable_sort(
      candidate_lanes.begin(), candidate_lanes.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });

  constexpr double kEpsilon = 1e-10;
  const double max_recommend_length = candidate_lanes.front().first;
  while (candidate_lanes.size() > 1) {
    if (std::fabs(candidate_lanes.back().first - max_recommend_length) >
        kEpsilon) {
      candidate_lanes.pop_back();
    } else {
      break;
    }
  }

  if (candidate_lanes.size() == 1) return candidate_lanes.front().second->id;

  // (abs_lateral_offset, candidate_index)
  std::vector<std::pair<double, int>> lateral_vec;
  lateral_vec.reserve(candidate_lanes.size());

  constexpr double kQueryLength = 5.0;  // m.
  const double frac = std::max(0.0, (source_lane_info.length() - kQueryLength) /
                                        source_lane_info.length());
  const Vec2d anchor_point = source_lane_info.LerpPointFromFraction(frac);
  const Vec2d tangent = source_lane_info.GetTangent(frac);

  for (int i = 0; i < static_cast<int>(candidate_lanes.size()); ++i) {
    const auto* out_lane_ptr = candidate_lanes[i].second;
    const auto out_lane_point = out_lane_ptr->LerpPointFromFraction(
        std::min(1.0, kQueryLength / out_lane_ptr->length()));
    const double cur_abs_lateral_offset =
        std::fabs((out_lane_point - anchor_point).CrossProd(tangent));
    lateral_vec.push_back({cur_abs_lateral_offset, i});
  }

  std::stable_sort(lateral_vec.begin(), lateral_vec.end());
  // Assume a section has only one direction.
  const bool target_is_turning =
      candidate_lanes.front().second->direction != mapping::LaneProto::STRAIGHT;
  return target_is_turning
             ? candidate_lanes[lateral_vec.back().second].second->id
             : candidate_lanes[lateral_vec.front().second].second->id;
}

double EstimateNaviSegmentTurningRadius(absl::Span<const Vec2d> points) {
  const int n = points.size();
  if (n < 3) return DBL_MAX;

  std::vector<double> radius_vec;
  radius_vec.reserve(n - 2);

  Vec3d last_perp_bisec = {0.0, 0.0, 0.0};
  for (int i = 2; i < n; ++i) {
    if (last_perp_bisec.x() == 0.0 && last_perp_bisec.y() == 0.0) {
      last_perp_bisec = ComputePerpBisectorCoeff(points[0], points[1]);
    }
    const auto new_perp_bisec =
        ComputePerpBisectorCoeff(points[i - 1], points[i]);
    const double denominator =
        new_perp_bisec.xy().CrossProd(last_perp_bisec.xy());
    if (std::fabs(denominator) < kEpsilon) {
      last_perp_bisec = new_perp_bisec;
      continue;
    }
    const double denominator_inv = 1 / denominator;
    const double intersection_point_x =
        (new_perp_bisec.y() * last_perp_bisec.z() -
         last_perp_bisec.y() * new_perp_bisec.z()) *
        denominator_inv;
    const double intersection_point_y =
        (last_perp_bisec.x() * new_perp_bisec.z() -
         new_perp_bisec.x() * last_perp_bisec.z()) *
        denominator_inv;
    radius_vec.push_back(Hypot(intersection_point_x - points[i].x(),
                               intersection_point_y - points[i].y()));
    last_perp_bisec = new_perp_bisec;
  }
  if (radius_vec.empty()) return DBL_MAX;

  constexpr double kWithinCircleBuffer = 10.0;  // m.
  std::vector<std::vector<double>> assigned_votes(radius_vec.size());
  for (int i = 0; i < radius_vec.size(); ++i) {
    for (const double radius : radius_vec) {
      if (std::abs(radius_vec[i] - radius) < kWithinCircleBuffer) {
        assigned_votes[i].push_back(radius);
      }
    }
  }
  int best_idx = 0;
  for (int i = 1; i < radius_vec.size(); ++i) {
    if (assigned_votes[i].size() > assigned_votes[best_idx].size() ||
        (assigned_votes[i].size() == assigned_votes[best_idx].size() &&
         radius_vec[i] < radius_vec[best_idx])) {
      best_idx = i;
    }
  }

  return std::accumulate(assigned_votes[best_idx].begin(),
                         assigned_votes[best_idx].end(), 0.0) /
         assigned_votes[best_idx].size();
}

double EstimateNaviSegmentCurvatureSpeedLimit(absl::Span<const Vec2d> points) {
  const double turning_radius = EstimateNaviSegmentTurningRadius(points);
  return kRadiusToSpeedLimitKph(turning_radius);
}

absl::StatusOr<double> CalcOutgoingLaneCrossProd(
    const PlannerSemanticMapManager& psmm, mapping::ElementId base_lane_id,
    mapping::ElementId outgoing_lane_id) {
  SMM_ASSIGN_LANE_OR_RETURN(base_lane_info, psmm, base_lane_id,
                            absl::NotFoundError(absl::StrFormat(
                                "Can not find lane id: %d", base_lane_id)));
  SMM_ASSIGN_LANE_OR_RETURN(outgoing_lane_info, psmm, outgoing_lane_id,
                            absl::NotFoundError(absl::StrFormat(
                                "Can not find lane id: %d", base_lane_id)));
  constexpr double kQueryDistance = 5.0;  // m.
  const double frac = std::max(0.0, (base_lane_info.length() - kQueryDistance) /
                                        base_lane_info.length());
  const Vec2d anchor_point = base_lane_info.LerpPointFromFraction(frac);
  const Vec2d tangent = base_lane_info.GetTangent(frac);

  const Vec2d outgoing_point = outgoing_lane_info.LerpPointFromFraction(
      std::min(1.0, kQueryDistance / outgoing_lane_info.length()));

  return (outgoing_point - anchor_point).normalized().CrossProd(tangent);
}

}  // namespace planner
}  // namespace qcraft
