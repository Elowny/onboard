#include "onboard/planner/router/route_core_searcher.h"

#include <algorithm>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/map_match.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route {
absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchForRouteCorePathFromGlobalPose(
    const RoutingRequestContext& route_context,
    const CoordinateConverter& coordinate_converter,
    const RoutingRequestProto& routing_request, const Vec2d& global,
    const RouteCorePrecondition& route_pre, double heading,
    RouteCore* route_core) {
  SCOPED_QTRACE("SearchForRouteCorePathFromPose");
  QCHECK_NOTNULL(route_context.map_index);

  const auto& smm = *route_context.semantic_map_manager;
  route_core->set_semantic_map_manager(route_context.semantic_map_manager);
  const auto& route_params = *route_context.route_param_proto;
  route_core->set_route_param_proto(route_context.route_param_proto);
  const auto level_id = coordinate_converter.GetLevel();

  std::vector<mapping::PointToLane> origin;
  std::vector<mapping::LanePath> simple_lane_path;

  const auto cur_point_to_lanes = map_match::GetNearLanesFromPose(
      smm, route_context.map_index, level_id, global, heading,
      route_params.on_driving_param());
  if (cur_point_to_lanes.empty()) {
    return absl::NotFoundError(absl::StrCat(
        "The AV project to lanes failed. pose:",
        global.DebugStringFullPrecision(), ", ", QCRAFT_LOC.ToString()));
  }
  // Infer the routing start lane.
  if (route_context.pred_trajectory_point.has_value()) {
    const auto pred_global = coordinate_converter.SmoothToGlobal(
        *route_context.pred_trajectory_point);
    const auto infer_point_to_lanes = PointToNearLanesWithHeadingAtLevel(
        *route_context.map_index, level_id, pred_global,
        route_params.on_driving_param().filter().radius_error(), heading,
        route_params.on_driving_param().filter().heading_error());
    QLOG(INFO) << "Infer trajectory point:"
               << pred_global.DebugStringFullPrecision()
               << ", the matched lanes size:" << infer_point_to_lanes.size();
    if (infer_point_to_lanes.empty()) {
      QLOG(ERROR) << "Cannot find the proper lane, pred pos:"
                  << pred_global.DebugStringFullPrecision();
    } else {
      const auto& lane_paths = FindAvailableSimplePaths(smm, cur_point_to_lanes,
                                                        infer_point_to_lanes);
      if (lane_paths.empty()) {
        return absl::NotFoundError(absl::StrCat(
            "Infer lane path from pose to predication trajectory "
            "point failed. car_global_pose: ",
            global.DebugString(),
            ", trajectory point: ", pred_global.DebugStringFullPrecision(),
            ", ", QCRAFT_LOC.ToString()));
      }

      for (const auto& lane_path_proto : lane_paths) {
        const mapping::ElementId infer_id =
            mapping::ElementId(*lane_path_proto.lane_ids().rbegin());
        const auto iter = std::find_if(
            infer_point_to_lanes.begin(), infer_point_to_lanes.end(),
            [&infer_id](const mapping::PointToLane& point_to_lane) {
              return point_to_lane.lane_proto->id() == infer_id.value();
            });
        if (iter != infer_point_to_lanes.end()) {
          origin.push_back(*iter);
          simple_lane_path.emplace_back(&smm, lane_path_proto);
        }
      }
    }
  }

  if (origin.empty()) {
    QLOG(INFO) << "No Infer trajectory point, use search directly";
    return SearchForRouteCorePathToRoutingRequest(
        route_context, routing_request, cur_point_to_lanes, route_pre,
        route_core);
  }

  ASSIGN_OR_RETURN(auto sections_lanes, SearchForRouteCorePathToRoutingRequest(
                                            route_context, routing_request,
                                            origin, route_pre, route_core));
  double min_length = std::numeric_limits<double>::infinity();
  mapping::LanePath connect_lane_path;
  for (const auto& lane_path : simple_lane_path) {
    if (lane_path.back() != sections_lanes.second.front()) {
      continue;
    }
    if (lane_path.length() < min_length) {
      connect_lane_path = lane_path;
    }
  }
  if (connect_lane_path.IsEmpty()) {
    return sections_lanes;
  }

  const CompositeLanePath front(connect_lane_path);
  const CompositeLanePath merge_results =
      front.Connect(&smm, sections_lanes.second);
  return std::pair<RouteSections, CompositeLanePath>(
      RouteSectionsFromCompositeLanePath(smm, merge_results), merge_results);
}

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchForRouteCorePathToRoutingRequest(
    const RoutingRequestContext& route_context,
    const RoutingRequestProto& routing_request,
    const std::vector<mapping::PointToLane>& origin,
    const RouteCorePrecondition& route_pre, RouteCore* route_core) {
  SCOPED_QTRACE("SearchForRouteCorePathToRoutingRequest");
  const auto& smm = *route_context.semantic_map_manager;
  route_core->set_semantic_map_manager(route_context.semantic_map_manager);
  route_core->set_route_param_proto(route_context.route_param_proto);
  ASSIGN_OR_RETURN(const auto destinations,
                   ParseDestinationProtoToLanePoints(
                       smm, route_context.map_index, routing_request));

  for (int i = 0; i < destinations.size(); ++i) {
    const auto& destination = destinations[i];
    if (destination.lane_id() == mapping::kInvalidElementId) {
      if (routing_request.destinations(i).has_off_road()) {
        QLOG(INFO) << "Destination " << i << " is off road end."
                   << " Now  not support  off road on route.";
      }
      return absl::UnavailableError(
          absl::StrFormat("Can not locate destination {%s} on map!",
                          routing_request.destinations(i).DebugString()));
    }
  }
  std::vector<std::vector<mapping::PointToLane>> dests;
  dests.reserve(destinations.size());
  for (int i = 0; i < destinations.size(); ++i) {
    const auto& cur_dest_lp = destinations[i];
    SMM_LANE_PROTO_OR_CONTINUE(lane_proto, smm, cur_dest_lp.lane_id());

    auto& dest = dests.emplace_back();
    // Consider all lanes in the section of via points.
    if (i + 1 != destinations.size()) {
      SMM_SECTION_PROTO_OR_ERROR(cur_sec_proto, smm, lane_proto->section_id());
      for (const auto& lane_id : cur_sec_proto->lanes()) {
        if (lane_id == cur_dest_lp.lane_id().value()) continue;
        SMM_LANE_PROTO_OR_CONTINUE(via_lane_proto, smm, lane_id);
        dest.push_back(
            mapping::PointToLane{.lane_proto = via_lane_proto,
                                 .dist = 0.0,
                                 .fraction = cur_dest_lp.fraction()});
      }
    }
    dest.push_back(mapping::PointToLane{.lane_proto = lane_proto,
                                        .dist = 0.0,
                                        .fraction = cur_dest_lp.fraction()});
  }
  return route_core->SearchForRoutePathAlongLanePoints(route_pre, origin,
                                                       dests);
}

}  // namespace qcraft::planner::route
