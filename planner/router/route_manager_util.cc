#include "onboard/planner/router/route_manager_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <ostream>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/lane_point.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/compatibility_layer.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/geometry/gfc.h"
#include "onboard/planner/router/map_match.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {
constexpr double kJudgeNearDistance = 20.0;  // m.
constexpr double kSectionLengthError = 1.0;  // m.

bool IsRealMotorWay(const mapping::LaneProto& lane_proto) {
  return !mapping::compat::IsPassengerVehicleAvoidLaneType(lane_proto.type()) &&
         !mapping::compat::IsVirtual(lane_proto);
}
void FindAndSupplyBackwardNaviInfo(const mapping::v2::SemanticMapManager& v2smm,
                                   mapping::SectionId start_sec_id,
                                   double extend_length,
                                   RouteNaviInfo* route_navi_info) {
  absl::flat_hash_map<mapping::ElementId, RouteNaviInfo::RouteLaneInfo>
      extra_lane_info_map;
  absl::flat_hash_map<mapping::SectionId, RouteNaviInfo::NaviSectionInfo>
      extra_section_info_map;
  SMM_ASSIGN_SECTION_OR_RETURN(start_sec_info, v2smm, start_sec_id, void());
  const auto start_sec_navi_iter =
      route_navi_info->navi_section_info_map.find(start_sec_id);
  if (start_sec_navi_iter == route_navi_info->navi_section_info_map.end()) {
    QLOG(ERROR) << "Can not find section navi info for: " << start_sec_id;
    return;
  }
  extra_section_info_map.insert({start_sec_id, start_sec_navi_iter->second});

  std::queue<std::pair<const mapping::v2::Lane*, double>> que;
  for (const auto lane_id : start_sec_info.proto().lanes()) {
    SMM_ASSIGN_LANE_OR_RETURN(lane, v2smm, mapping::ElementId(lane_id), void());
    que.push({&lane, 0.0});
    const auto lane_navi_info_iter =
        route_navi_info->route_lane_info_map.find(mapping::ElementId(lane_id));
    if (lane_navi_info_iter == route_navi_info->route_lane_info_map.end()) {
      QLOG(ERROR) << "Can not find lane navi info for: " << lane_id;
      return;
    }
    extra_lane_info_map.insert(
        {mapping::ElementId(lane_id), lane_navi_info_iter->second});
  }

  while (!que.empty()) {
    const auto [cur_lane_info_ptr, addition_len] = que.front();
    que.pop();
    if (addition_len > extend_length) continue;

    const auto cur_lane_navi_iter = extra_lane_info_map.find(
        mapping::ElementId(cur_lane_info_ptr->proto().id()));
    if (cur_lane_navi_iter == extra_lane_info_map.end()) continue;
    const auto cur_route_lane_info = cur_lane_navi_iter->second;

    const auto cur_sec_navi_iter = extra_section_info_map.find(
        mapping::SectionId(cur_lane_info_ptr->proto().section_id()));
    if (cur_sec_navi_iter == extra_section_info_map.end()) continue;
    const auto cur_route_sec_info = cur_sec_navi_iter->second;

    for (const auto incoming_id : cur_lane_info_ptr->proto().incoming_lanes()) {
      SMM_ASSIGN_LANE_OR_RETURN(incoming_lane, v2smm,
                                mapping::ElementId(incoming_id), void());
      const auto* incoming_lane_info = &incoming_lane;
      SMM_ASSIGN_SECTION_OR_RETURN(
          incoming_sec, v2smm,
          mapping::SectionId(incoming_lane_info->proto().section_id()), void());
      const auto* incoming_sec_ptr = &incoming_sec;
      const double len =
          addition_len + (incoming_sec_ptr == nullptr
                              ? incoming_lane_info->length()
                              : incoming_sec_ptr->proto().average_length());

      // Lane navi info.
      auto& incoming_lane_navi_info = extra_lane_info_map[mapping::ElementId(
          incoming_lane_info->proto().id())];
      incoming_lane_navi_info.max_driving_distance =
          std::max(incoming_lane_navi_info.max_driving_distance,
                   cur_route_lane_info.max_driving_distance + len);
      incoming_lane_navi_info.max_reach_length =
          std::max(incoming_lane_navi_info.max_reach_length,
                   cur_route_lane_info.max_reach_length + len);
      incoming_lane_navi_info.recommend_reach_length =
          std::max(incoming_lane_navi_info.recommend_reach_length,
                   cur_route_lane_info.recommend_reach_length + len);
      incoming_lane_navi_info.min_lc_num_to_target =
          std::min(incoming_lane_navi_info.min_lc_num_to_target,
                   cur_route_lane_info.min_lc_num_to_target);
      incoming_lane_navi_info.lc_num_within_driving_dist =
          std::min(incoming_lane_navi_info.lc_num_within_driving_dist,
                   cur_route_lane_info.lc_num_within_driving_dist);
      incoming_lane_navi_info.len_before_merge_lane =
          std::max(incoming_lane_navi_info.len_before_merge_lane,
                   cur_route_lane_info.len_before_merge_lane);

      // Section navi info.
      const auto incoming_section_navi_iter = extra_section_info_map.find(
          mapping::SectionId(incoming_lane_info->proto().section_id()));
      if (incoming_section_navi_iter == extra_section_info_map.end()) {
        auto& incoming_sec_navi = extra_section_info_map[mapping::SectionId(
            incoming_lane_info->proto().section_id())];
        incoming_sec_navi.length_before_intersection =
            cur_route_sec_info.length_before_intersection + len;
        incoming_sec_navi.intersection_direction =
            cur_route_sec_info.intersection_direction;
      }

      if (len < extend_length) {
        que.push({incoming_lane_info, len});
      }
    }
  }

  route_navi_info->route_lane_info_map.merge(extra_lane_info_map);
  route_navi_info->navi_section_info_map.merge(extra_section_info_map);
}
}  // namespace

absl::StatusOr<int> FindNextDestinationIndexViaLanePoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MultipleStopsRequest& multi_stops_request,
    const std::function<bool(const RouteSections&)>& point_on_route_func,
    route::RouteCore* route_core) {
  FUNC_QTRACE();
  if (multi_stops_request.destination_size() < 2) return 0;
  if (!multi_stops_request.skip_past_stops()) return 0;

  int origin_idx = multi_stops_request.infinite_loop()
                       ? multi_stops_request.destination_size() - 1
                       : 0;
  int next_idx = multi_stops_request.infinite_loop() ? 0 : 1;

  const auto find_lane_point_fn = [&multi_stops_request](int index) {
    const auto index_pair = multi_stops_request.GetIndexPair(index);
    return multi_stops_request
        .lane_point_destinations()[index_pair.first][index_pair.second];
  };

  while (next_idx < multi_stops_request.destination_size()) {
    const auto origin = find_lane_point_fn(origin_idx);
    const auto destination = find_lane_point_fn(next_idx);
    if (origin.Valid() && destination.Valid()) {
      route::RouteCorePrecondition rc_pre{.is_bus = true, .use_time = true};
      SMM_LANE_PROTO_OR_BREAK(origin_lane, semantic_map_manager,
                              origin.lane_id());
      SMM_LANE_PROTO_OR_BREAK(dest_lane, semantic_map_manager,
                              destination.lane_id());
      std::vector<mapping::PointToLane> origins{
          mapping::PointToLane{.lane_proto = origin_lane,
                               .dist = 0.0,
                               .fraction = origin.fraction()}};
      std::vector<mapping::PointToLane> destinations{
          mapping::PointToLane{.lane_proto = dest_lane,
                               .dist = 0.0,
                               .fraction = destination.fraction()}};

      const auto sections_lanes_or =
          route_core->SearchForRoutePathFromLanePoint(rc_pre, origins,
                                                      destinations);

      if (sections_lanes_or.ok()) {
        VLOG(4) << "Sections to destination No." << next_idx << std::endl
                << sections_lanes_or->first.DebugString();
        if (point_on_route_func(sections_lanes_or->first)) {
          return next_idx;
        }
      }
    }
    ++next_idx;
    ASSIGN_OR_RETURN(const auto origin_next_index,
                     multi_stops_request.GetNextDestinationIndex(origin_idx));
    origin_idx = origin_next_index;
  }

  return 0;
}

absl::StatusOr<mapping::LanePoint> FindLanePointFromPose(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex& map_index,
    const CoordinateConverter& coordinate_converter,
    const RouteParamProto& route_param_proto, const PoseProto& pose) {
  const auto car_global = coordinate_converter.SmoothToGlobal(
      {pose.pos_smooth().x(), pose.pos_smooth().y()});
  const auto heading = coordinate_converter.SmoothYawToGlobal(pose.yaw());
  // (1) Map matching
  ASSIGN_OR_RETURN(
      const auto point_to_lane,
      route::map_match::GetNearestLaneOnDriving(
          semantic_map_manager, &map_index, coordinate_converter.GetLevel(),
          car_global, heading, route_param_proto));
  // (2) Handle start point (match lane, add lane cost)
  // 2.1 fail if on  bicycle lane
  const mapping::LaneProto* lane_proto = point_to_lane.lane_proto;
  if (!IsRealMotorWay(*lane_proto)) {
    return absl::FailedPreconditionError("Must on motorway when reroute.");
  }
  return mapping::LanePoint(mapping::ElementId(lane_proto->id()),
                            point_to_lane.fraction);
}

absl::StatusOr<MultipleStopsRequest> RecoverRoutingRequest(
    const RoutingRequestProto& log_routing_request,
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index) {
  if (FLAGS_route_recover_destinations_from_log) {
    if (log_routing_request.destinations().empty()) {
      if (!log_routing_request.has_multi_stops()) {
        return absl::NotFoundError(
            "When recover destination from log, request's "
            "destinations are empty and no multi stops.");
      }
      VLOG(2) << "Recover routing request with recorded multiple stops: "
              << log_routing_request.multi_stops().DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager, map_index,
                                       log_routing_request);
    } else {
      VLOG(2) << "Recover routing request with recorded destinations: "
              << log_routing_request.DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager, map_index,
                                       log_routing_request);
    }
  } else {
    if (!log_routing_request.has_multi_stops()) {
      if (log_routing_request.destinations().empty()) {
        return absl::NotFoundError(
            "When not recover destination from log, request has no "
            "multi stops and destinations are empty.");
      }

      QLOG(INFO) << "Recover routing request with recorded destinations: "
                 << log_routing_request.DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager, map_index,
                                       log_routing_request);
    } else {
      QLOG(INFO) << "Recover routing request with recorded multiple stops: "
                 << log_routing_request.multi_stops().DebugString();
      return BuildMultipleStopsRequest(semantic_map_manager, map_index,
                                       log_routing_request);
    }
  }
}
absl::StatusOr<RoutingRequestProto>
GenerateAndCheckRoutingRequestProtoToNextStop(
    const mapping::v2::SemanticMapManager& smm,
    const route::MapIndex* map_index, const RouteParamProto& route_param_proto,
    const MultipleStopsRequest& multi_stops, const Vec2d& car_global,
    double heading, int next_destination_index) {
  FUNC_QTRACE();
  ASSIGN_OR_RETURN(const auto request_proto,
                   multi_stops.GenerateRoutingRequestProtoToNextStop(
                       smm, next_destination_index));

  if (request_proto.destinations().size() <= 1 ||
      !request_proto.destinations()[0].has_global_point()) {
    return request_proto;
  }
  const auto& geo_point = request_proto.destinations()[0].global_point();
  const Vec2d first_via_global = {geo_point.longitude(), geo_point.latitude()};
  const double z = geo_point.altitude();

  if (route::HaversineDistance(car_global, first_via_global) <
      kJudgeNearDistance) {
    const auto av_near_lanes = route::map_match::GetNearLanesFromPose(
        smm, map_index, smm.coordinate_converter().GetLevel(), car_global,
        heading, route_param_proto.on_driving_param());
    const auto level_lanes_pair = PointToNearLanesWithInferLevel(
        smm, *map_index, first_via_global, z,
        /*min_distance=*/
        route_param_proto.poi_match_param().filter().radius_error());
    const auto& first_via_near_lanes = level_lanes_pair.second;
    // Do both the nearest lanes belong to the same section?
    if (!av_near_lanes.empty() && !first_via_near_lanes.empty() &&
        first_via_near_lanes[0].lane_proto->section_id() ==
            first_via_near_lanes[0].lane_proto->section_id()) {
      RoutingRequestProto request = request_proto;
      request.clear_destinations();
      for (int i = 1; i < request_proto.destinations_size(); ++i) {
        *request.add_destinations() = request_proto.destinations()[i];
      }
      return request;
    }
  }
  return request_proto;
}

absl::StatusOr<RouteProto> TrackAlternateRoute(
    const mapping::v2::SemanticMapManager& smm, const RouteProto& last_route,
    mapping::LanePoint cur_match_point, double travel_dist) {
  SMM_ASSIGN_LANE_OR_RETURN(
      match_lane_info, smm, cur_match_point.lane_id(),
      absl::NotFoundError(absl::StrFormat("Can not find element id: %d",
                                          cur_match_point.lane_id())));

  int lane_path_idx = 0, lane_idx = 0, sec_idx = -1;
  double accum_length = 0.0;
  mapping::SectionId last_sec_id = mapping::kInvalidSectionId;
  bool track_success = false;

  const auto& clp_proto = last_route.lane_path();
  for (; lane_path_idx < clp_proto.lane_paths_size(); ++lane_path_idx) {
    const auto& cur_lane_path = clp_proto.lane_paths(lane_path_idx);
    for (int i = 0; i < cur_lane_path.lane_ids_size(); ++i) {
      SMM_ASSIGN_LANE_OR_RETURN(
          cur_lane_info, smm, mapping::ElementId(cur_lane_path.lane_ids(i)),
          absl::NotFoundError(absl::StrFormat("Can not find element id: %d",
                                              cur_lane_path.lane_ids(i))));
      if (cur_lane_info.Proto().section_id() != last_sec_id.value()) {
        last_sec_id = mapping::SectionId(cur_lane_info.Proto().section_id());
        ++sec_idx;
      }
      const double start_frac = i == 0 ? cur_lane_path.start_fraction() : 0.0;
      const double end_frac = i + 1 == cur_lane_path.lane_ids_size()
                                  ? cur_lane_path.end_fraction()
                                  : 1.0;
      if (cur_lane_info.Proto().section_id() ==
          match_lane_info.Proto().section_id()) {
        SMM_SECTION_PROTO_OR_RETURN(cur_sec_info, smm,
                                    cur_lane_info.Proto().section_id(),
                                    absl::NotFoundError(absl::StrFormat(
                                        "Can not find section id: %ld",
                                        cur_lane_info.Proto().section_id())));
        const double frac_error =
            kSectionLengthError / cur_sec_info->average_length();
        if (cur_match_point.fraction() >=
                std::max(0.0, start_frac - frac_error) &&
            cur_match_point.fraction() <=
                std::min(1.0, end_frac + frac_error)) {
          track_success = true;
          lane_idx = i;
          break;
        }
      }
      accum_length +=
          std::max(cur_lane_info.length() * (end_frac - start_frac), 0.0);
      if (accum_length > travel_dist) {
        return absl::NotFoundError(
            "Can not track alternate route successfully.");
      }
    }
    if (track_success) {
      break;
    }
  }

  if (track_success) {
    auto section_seq_proto = last_route.route_section_sequence();
    section_seq_proto.mutable_section_id()->erase(
        section_seq_proto.section_id().begin(),
        section_seq_proto.section_id().begin() + sec_idx);
    section_seq_proto.set_start_fraction(cur_match_point.fraction());

    CompositeLanePathProto::CompositeIndexProto idx_proto;
    idx_proto.set_lane_path_index(lane_path_idx);
    idx_proto.set_lane_index(lane_idx);

    mapping::LanePointProto lp_proto;
    lp_proto.set_lane_id(cur_match_point.lane_id().value());
    lp_proto.set_fraction(cur_match_point.fraction());

    auto updated_clp_proto = AfterCompositeIndexAndLanePoint(
        last_route.lane_path(), idx_proto, lp_proto);

    RouteProto route;
    *route.mutable_lane_path() = std::move(updated_clp_proto);
    *route.mutable_route_section_sequence() = std::move(section_seq_proto);
    route.set_update_id(last_route.update_id());
    *route.mutable_routing_request() = last_route.routing_request();
    *route.mutable_avoid_lanes() = last_route.avoid_lanes();
    return route;
  }
  return absl::NotFoundError("Can not track alternate route successfully.");
}

absl::StatusOr<std::pair<RouteSections, RouteSections>>
BackWardExtendSectionsFromCurrentAlongRoute(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionSequenceProto& global_sections,
    mapping::LanePoint start_point, int start_sec_idx, double extend_length) {
  double start_frac = start_point.fraction();
  int idx = start_sec_idx;
  double residual_length = extend_length;
  bool success_extend = false;
  for (; idx >= 0; --idx) {
    SMM_ASSIGN_SECTION_OR_BREAK(
        cur_sec_info, v2smm,
        mapping::SectionId(global_sections.section_id(idx)));
    const double cur_end_frac =
        idx == 0 ? global_sections.start_fraction() : 0.0;
    const double cur_length =
        cur_sec_info.proto().average_length() * (start_frac - cur_end_frac);
    residual_length -= cur_length;
    if (residual_length <= 0.0) {
      start_frac = std::min(
          std::abs(residual_length) / cur_sec_info.proto().average_length() +
              cur_end_frac,
          1.0);
      success_extend = true;
      break;
    }
    start_frac = 1.0;
  }

  if (!success_extend) {
    return absl::OutOfRangeError("Extend length is out of route.");
  }

  std::vector<mapping::SectionId> section_ids;
  section_ids.reserve(global_sections.section_id_size() - idx);
  std::vector<mapping::SectionId> clamp_section_ids;
  clamp_section_ids.reserve(start_sec_idx - idx + 1);
  for (int i = idx; i < global_sections.section_id_size(); ++i) {
    if (i <= start_sec_idx) {
      clamp_section_ids.push_back(
          mapping::SectionId(global_sections.section_id(i)));
    }
    section_ids.push_back(mapping::SectionId(global_sections.section_id(i)));
  }
  const mapping::LanePoint dest =
      global_sections.has_destination()
          ? mapping::LanePoint(global_sections.destination())
          : mapping::LanePoint(mapping::kInvalidElementId, 0.0);
  return std::make_pair(
      RouteSections(start_frac, start_point.fraction(),
                    std::move(clamp_section_ids), start_point),
      RouteSections(start_frac, global_sections.end_fraction(),
                    std::move(section_ids), dest));
}

absl::StatusOr<RouteNaviInfo> CalcCurRouteNaviInfo(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionSequenceProto& global_sections,
    const RouteSectionSequenceProto& cur_sections,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    mapping::LanePoint start_lp, double extend_length, double preview_dist) {
  SCOPED_QTRACE("CalcCurRouteNaviInfo");

  const int start_sec_idx =
      global_sections.section_id_size() - cur_sections.section_id_size();
  if (start_sec_idx < 0) {
    return absl::InvalidArgumentError(
        "Current sections is larger than global sections.");
  }

  auto back_extend_sections_pair_or =
      BackWardExtendSectionsFromCurrentAlongRoute(
          v2smm, global_sections, start_lp, start_sec_idx, extend_length);
  if (back_extend_sections_pair_or.ok()) {
    const RouteSectionsInfo sections_info(
        v2smm, &back_extend_sections_pair_or->second);
    ASSIGN_OR_RETURN(auto route_navi_info,
                     CalcNaviInfoByLaneGraph(v2smm, sections_info, avoid_lanes,
                                             preview_dist));
    route_navi_info.back_extend_sections =
        std::move(back_extend_sections_pair_or->first);
    return route_navi_info;
  }

  // If backward extend failed, consider all lanes behind, usually happened in
  // the beginning.
  const auto cur_route_sections = RouteSections::BuildFromProto(cur_sections);
  const RouteSectionsInfo cur_sections_info(v2smm, &cur_route_sections);
  ASSIGN_OR_RETURN(auto route_navi_info,
                   CalcNaviInfoByLaneGraph(v2smm, cur_sections_info,
                                           avoid_lanes, preview_dist));
  FindAndSupplyBackwardNaviInfo(v2smm, cur_route_sections.front().id,
                                extend_length, &route_navi_info);
  return route_navi_info;
}

}  // namespace planner
}  // namespace qcraft
