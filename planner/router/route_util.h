#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_UTIL_H_

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>  // IWYU pragma: keep
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_util.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/level_id.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {
namespace internal {

std::vector<mapping::ElementId> ToElementIds(
    absl::Span<const std::int64_t> ids);

// Convert different types of routing destination to global point
// representation.
absl::StatusOr<mapping::GeoPointProto> ConvertToGlobal(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const RoutingDestinationProto& dest);
}  // namespace internal

std::vector<Vec2d> GlobalToSmoothPoints(
    const CoordinateConverter& cc,
    const google::protobuf::RepeatedPtrField<mapping::GeoPointProto>&
        points_global);

// TODO(xiang): Move to the map_match module.
std::pair<mapping::LevelId, std::vector<mapping::PointToLane>>
PointToNearLanesWithInferLevel(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex& index, const Vec2d& global, double z,
    double min_distance);

std::vector<mapping::PointToLane> PointToNearLanesAtLevel(
    const route::MapIndex& index, mapping::LevelId level, const Vec2d& global,
    double min_distance);

std::vector<mapping::PointToLane> PointToNearLanesWithHeadingAtLevel(
    const route::MapIndex& index, mapping::LevelId level, const Vec2d& global,
    double max_distance, double heading, double max_heading_diff);

absl::StatusOr<mapping::LanePoint> ParseDestinationProtoToLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, const RoutingDestinationProto& proto,
    double min_distance = 10.0,
    RouteMatchType match_type = RouteMatchType::STATIONARY);

absl::StatusOr<std::vector<mapping::LanePoint>>
ParseDestinationProtoToLanePoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index,
    const RoutingRequestProto& routing_request);

absl::StatusOr<RoutingRequestProto> ConvertRoutingRequestToGlobalPoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const RoutingRequestProto& raw_routing_request);

mapping::LanePath FindLanePathFromLaneAlongRouteSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, mapping::ElementId start_lane_id,
    double start_frac, double extend_len);

absl::StatusOr<RoutingRequestProto> ConvertAllRouteFlagsToRoutingRequestProto();

absl::flat_hash_set<mapping::ElementId> FindAvoidLanesFromAvoidRegions(
    const RoutingRequestProto& routing_request,
    const mapping::v2::SemanticMapManager& semantic_map_manager);

absl::StatusOr<std::vector<Vec2d>> SimplifyPathPoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePathProto& composite_lane_path,
    double max_distance_meters);

bool HasOnlyOneOffRoadRequest(const RoutingRequestProto& routing_request);

inline bool IsOffRoadDestination(
    const MultipleStopsRequestProto::StopProto& next_stop) {
  return next_stop.stop_point().has_off_road();
}

inline bool IsValidRouteOutputStatus(
    const RouteManagerOutputProto::RouteStatus& route_mgr_status) {
  switch (route_mgr_status) {
    case RouteManagerOutputProto::NORMAL:
    case RouteManagerOutputProto::RESET:
      return true;
    case RouteManagerOutputProto::INVALID:
      return false;
  }
}

bool IsValidRouteOutputProto(const RouteManagerOutputProto& route_out_proto);

inline bool IsResetRouteOutputProto(
    const RouteManagerOutputProto& route_out_proto) {
  if (route_out_proto.has_reset()) {
    return route_out_proto.reset();
  }
  if (route_out_proto.has_route_status()) {
    return route_out_proto.route_status() == RouteManagerOutputProto::RESET;
  }
  return false;
}

inline bool HasValidRouteResults(const RouteManagerOutput& route_mgr_output) {
  return route_mgr_output.status == RouteManagerOutputProto::NORMAL &&
         !route_mgr_output.route_sections_from_current.empty() &&
         !route_mgr_output.route_navi_info.route_lane_info_map.empty();
}

std::string PoseDebugString(const CoordinateConverter& coordinate_converter,
                            const PoseProto& pose);

bool IsTravelMatchedValid(const CompositeLanePath::CompositeIndex& first,
                          double first_fraction,
                          const CompositeLanePath::CompositeIndex& second,
                          double second_fraction, int64_t last_time_micros,
                          int64_t cur_time_micros, double distance,
                          double speed, double reroute_threshold_meters,
                          double estimate_speed);

struct SectionLanesQueueItem {
  mapping::ElementId id = mapping::kInvalidElementId;
  mapping::LaneBoundaryProto::Type type =
      mapping::LaneBoundaryProto::UNKNOWN_TYPE;
  SectionLanesQueueItem* prev = nullptr;
};

/// @brief return the lane sequence from the src lane(included) to the target
/// lane(included) in the same section according to the specified direction,
/// NotFoundError if cannot reach.
absl::StatusOr<std::vector<SectionLanesQueueItem>> SearchConnectLaneInSection(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    mapping::ElementId scr_lane_id, mapping::ElementId target_lane_id,
    bool search_direction_left);

/// @brief true if all lanes passed is separated with broke-white or unknown
/// type. The two lanes no need to be adjacent and ignore the fraction. We
/// assume all lanes are almost with same length, better if project to the
/// center line of the path boundary.
bool IsTwoLaneConnectedWithBrokenWhites(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    mapping::ElementId src, mapping::ElementId target);

bool CrossSolidBoundary(mapping::LaneBoundaryProto::Type type, bool lc_left);

template <typename CompositeLanePathType,
          std::enable_if_t<
              std::is_same_v<CompositeLanePathType, CompositeLanePath> ||
                  std::is_same_v<CompositeLanePathType, CompositeLanePathProto>,
              bool> = true>
double TravelDistanceBetween(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePathType& composite_lane_path,
    const CompositeLanePath::CompositeIndex& first, double first_fraction,
    const CompositeLanePath::CompositeIndex& second, double second_fraction) {
  VLOG(3) << composite_lane_path.DebugString()
          << " first:" << first.DebugString()
          << "first fraction:" << first_fraction
          << " second fraction:" << second_fraction
          << ", second:" << second.DebugString();

  const auto length_fn = [&semantic_map_manager, &composite_lane_path](
                             int lane_path_idx, int first_lane_idx,
                             int last_lane_idx, double first_frac,
                             double last_frac) {
    double accum_len = 0.0;
    const auto& cur_lane_path = composite_lane_path.lane_paths()[lane_path_idx];
    for (int i = first_lane_idx; i <= last_lane_idx; ++i) {
      const double start_frac = i == first_lane_idx ? first_frac : 0.0;
      const double end_frac = i == last_lane_idx ? last_frac : 1.0;
      SMM_ASSIGN_LANE_OR_CONTINUE(
          cur_lane_info, semantic_map_manager,
          mapping::ElementId(cur_lane_path.lane_ids()[i]));
      accum_len += cur_lane_info.length() * (end_frac - start_frac);
    }
    return accum_len;
  };

  // Reversing is allowed here, therefore the distance could be negative.
  const auto& first_lane_path =
      composite_lane_path.lane_paths()[first.lane_path_index];
  const auto& last_lane_path =
      composite_lane_path.lane_paths()[second.lane_path_index];

  double travel_distance = 0.0;
  if (first.lane_path_index == second.lane_path_index) {
    travel_distance +=
        length_fn(first.lane_path_index, first.lane_index, second.lane_index,
                  first_fraction, second_fraction);
  } else {
    // first lane path
    travel_distance +=
        length_fn(first.lane_path_index, first.lane_index,
                  std::max(0, first_lane_path.lane_ids_size() - 1),
                  first_fraction, first_lane_path.end_fraction());
    // last lane path
    travel_distance +=
        length_fn(second.lane_path_index, 0, second.lane_index,
                  last_lane_path.start_fraction(), second_fraction);
    // all mid lane path
    for (int i = first.lane_path_index + 1; i < second.lane_path_index; i++) {
      const auto& lane_path = composite_lane_path.lane_paths()[i];
      travel_distance +=
          mapping::GetLanePathLength(semantic_map_manager, lane_path);
    }
  }
  return travel_distance;
}

std::vector<mapping::LanePathProto> FindAvailableSimplePaths(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    absl::Span<const mapping::PointToLane> origins,
    absl::Span<const mapping::PointToLane> dests);

Vec3d ComputePointOnLane(const google::protobuf::RepeatedPtrField<
                             mapping::GeoPointProto>& geo_points,
                         double fraction);

bool IsCityExpressOrHighway(mapping::SectionProto::RoadClass road_class);

RouteParamProto CreateDefaultRouteParam();

double AccumRouteSectionsLengthBetween(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& sections, int start_idx, int end_idx);

template <typename CompositeLanePathType,
          std::enable_if_t<
              std::is_same_v<CompositeLanePathType, CompositeLanePath> ||
                  std::is_same_v<CompositeLanePathType, CompositeLanePathProto>,
              bool> = true>
double ComputeEtaToRouteEnd(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePathType& composite_lane_path) {
  double cum_time_secs = 0.0;
  for (const auto& lane_path : composite_lane_path.lane_paths()) {
    for (int i = 0; i < lane_path.lane_ids_size(); ++i) {
      SMM_ASSIGN_LANE_OR_BREAK(lane_info, semantic_map_manager,
                               mapping::ElementId(lane_path.lane_ids()[i]));
      const double start_frac = i == 0 ? lane_path.start_fraction() : 0.0;
      const double end_frac =
          i + 1 == lane_path.lane_ids_size() ? lane_path.end_fraction() : 1.0;
      cum_time_secs += lane_info.length() * (end_frac - start_frac) /
                       Kph2Mps(lane_info.Proto().speed_limit_kph());
    }
  }
  return cum_time_secs;
}

template <typename RouteSectionsType,
          std::enable_if_t<
              std::is_same_v<RouteSectionsType, RouteSections> ||
                  std::is_same_v<RouteSectionsType, RouteSectionSequenceProto>,
              bool> = true>
double CalcRouteSectionsLength(const mapping::v2::SemanticMapManager& smm,
                               const RouteSectionsType& sections) {
  double length = 0.0;
  for (int i = 0; i < sections.section_id_size(); ++i) {
    const double start_frac = i == 0 ? sections.start_fraction() : 0.0;
    const double end_frac =
        i + 1 == sections.section_id_size() ? sections.end_fraction() : 1.0;
    SMM_ASSIGN_SECTION_OR_RETURN(
        sec_info, smm, mapping::SectionId(sections.section_id(i)), length);
    length += sec_info.proto().average_length() *
              std::max(0.0, end_frac - start_frac);
  }
  return length;
}

// TODO(zuowei): Adapt to smm v2.
absl::StatusOr<RouteSectionsInfo> BuildRouteSectionsInfoFromData(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSections* route_sections, double planning_horizon);

// TODO(zuowei): Delete when lane proto has length.
inline double GetLaneProtoLength(const mapping::v2::SemanticMapManager& smm,
                                 const mapping::LaneProto& lane_proto) {
  return QCHECK_NOTNULL(
             mapping::FindLanePtr(smm, mapping::ElementId(lane_proto.id())))
      ->length();
}

// TODO(zuowei): Delete when router replaces lane info with lane proto totally.
inline std::vector<mapping::ElementId> ConvertToElementIdFrom(
    const ::google::protobuf::RepeatedField<int64_t>& lane_ids) {
  std::vector<mapping::ElementId> res;
  res.reserve(lane_ids.size());
  std::transform(lane_ids.begin(), lane_ids.end(), std::back_inserter(res),
                 [](int64_t lane_id) { return mapping::ElementId(lane_id); });
  return res;
}

inline mapping::LaneBoundaryProto::Type GetLaneNeighborBoundary(
    const mapping::LaneNeighborProto& lane_neighbor) {
  return lane_neighbor.has_lane_boundary_type()
             ? lane_neighbor.lane_boundary_type()
             : mapping::LaneBoundaryProto::BROKEN_WHITE;
}

std::vector<Polygon2d> SampleRouteSectionsPolygons(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections);

bool IsValidLocalization(
    const std::shared_ptr<const LocalizationTransformProto>&
        localization_transform);

mapping::ElementId FindMostReasonableOutgoingLaneIdOrInvalid(
    const PlannerSemanticMapManager& psmm,
    const mapping::LaneInfo& source_lane_info,
    const RouteNaviInfo& route_navi_info,
    const RouteSectionsInfo::RouteSectionSegmentInfo& target_section_seg);

double EstimateNaviSegmentTurningRadius(absl::Span<const Vec2d> points);

// Unit: Kph.
double EstimateNaviSegmentCurvatureSpeedLimit(absl::Span<const Vec2d> points);

absl::StatusOr<double> CalcOutgoingLaneCrossProd(
    const PlannerSemanticMapManager& psmm, mapping::ElementId base_lane_id,
    mapping::ElementId outgoing_lane_id);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_UTIL_H_
