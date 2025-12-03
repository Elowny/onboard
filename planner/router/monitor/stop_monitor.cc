#include "onboard/planner/router/monitor/stop_monitor.h"

#include <stdint.h>

#include <ostream>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/geometry/gfc.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_generator.h"
#include "onboard/planner/router/route_manager_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route {

namespace internal {
bool IsNearBusStop(const mapping::v2::SemanticMapManager& semantic_map_manager,
                   mapping::ElementId lane_id, double fraction,
                   const Vec2d& current_pos, double radius) {
  SMM_LANE_PROTO_OR_RETURN(lane_proto, semantic_map_manager, lane_id, false);
  const auto global3d =
      ComputePointOnLane(lane_proto->polyline().points(), fraction);
  return route::HaversineDistance({global3d.x(), global3d.y()}, current_pos) <
         radius;
}

absl::Status CheckPrecondition(const StopMonitorInput& input) {
  if (input.context.semantic_map_manager == nullptr) {
    return absl::InvalidArgumentError("semantic_map_manager cannot be null.");
  }
  if (input.multi_stops == nullptr) {
    return absl::InvalidArgumentError("multi_stops cannot be null.");
  }
  if (input.next_stop_index < 0) {
    return absl::InvalidArgumentError("next_stop_index cannot be negative.");
  }
  return absl::OkStatus();
}

}  // namespace internal

bool ShouldGoNextStop(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, const MultipleStopsRequest& multi_stops,
    int next_stop_index, const Vec2d& av_global2d) {
  const auto next_stop_lane_point_or =
      multi_stops.ComputeDestinationLanePointByTotalIndex(
          semantic_map_manager, map_index, next_stop_index);
  if (!next_stop_lane_point_or.ok()) {
    QLOG(ERROR) << "ComputeDestinationLanePointByTotalIndex failed "
                << next_stop_lane_point_or.status().ToString();
    return false;
  }

  bool jump_next_stop =
      FLAGS_route_auto_next_stop &&
      internal::IsNearBusStop(semantic_map_manager,
                              next_stop_lane_point_or->lane_id(),
                              next_stop_lane_point_or->fraction(), av_global2d,
                              FLAGS_route_arrive_distance);
  // Reroute to next stop if necessary
  return jump_next_stop;
}

absl::StatusOr<RoutingResultProto> GotoNextStop(const StopMonitorInput& input,
                                                const CoordinateConverter& cc,
                                                int new_next_dest_idx,
                                                RouteCore* route_core) {
  const mapping::v2::SemanticMapManager& semantic_map_manager =
      *input.context.semantic_map_manager;
  const route::MapIndex* map_index = input.context.map_index;
  const RouteParamProto& route_param_proto = *input.context.route_param_proto;
  const MultipleStopsRequest& multi_stops = *input.multi_stops;
  int64_t request_id = input.context.request_id + 1;
  bool is_bus = input.is_bus;
  auto av_global2d = cc.SmoothToGlobal(
      {input.pose->pos_smooth().x(), input.pose->pos_smooth().y()});
  double heading = cc.SmoothYawToGlobal(input.pose->yaw());

  // Once reached here, we must already have a valid
  // route. next_destination_index must also be valid.
  ASSIGN_OR_RETURN(auto new_request_proto,
                   GenerateAndCheckRoutingRequestProtoToNextStop(
                       semantic_map_manager, map_index, route_param_proto,
                       multi_stops, av_global2d, heading, new_next_dest_idx));

  QLOG(INFO) << "new route to next stop "
             << new_request_proto.ShortDebugString();
  PlannerRoutingRequestProto active_routing_request;
  *(active_routing_request.mutable_init()->mutable_pose()) = *input.pose;
  *(active_routing_request.mutable_init()->mutable_routing_request()) =
      std::move(new_request_proto);
  active_routing_request.set_request_id(request_id);
  return GenerateRoutingResultProto(
      input.context, cc, multi_stops.route_restrict_district(),
      &active_routing_request, is_bus, route_core);
}

absl::StatusOr<StopMonitorOutput> MonitorStopStatus(
    const StopMonitorInput& input, const CoordinateConverter& cc,
    RouteCore* route_core) {
  auto status = internal::CheckPrecondition(input);
  if (!status.ok()) {
    return status;
  }

  if (input.res_button_pressed) {
    QLOG(INFO) << "RES BUTTON pressed.";
  }
  bool goto_next_stop = false;

  auto av_global2d = cc.SmoothToGlobal(
      {input.pose->pos_smooth().x(), input.pose->pos_smooth().y()});
  if (input.res_button_pressed ||
      ShouldGoNextStop(*input.context.semantic_map_manager,
                       input.context.map_index, *input.multi_stops,
                       input.next_stop_index, av_global2d)) {
    QLOG(INFO) << "Go to stop:" << input.next_stop_index;
    goto_next_stop = true;
  }
  if (!goto_next_stop) {
    return StopMonitorOutput{.goto_next_stop = false};
  } else {
    int new_next_dest_idx;
    if (!input.multi_stops->HasNextDestinationIndex(input.next_stop_index,
                                                    &new_next_dest_idx)) {
      return absl::NotFoundError("Cannot find the next stop index.");
    }
    ASSIGN_OR_RETURN(auto next_stop_result,
                     GotoNextStop(input, cc, new_next_dest_idx, route_core));
    return StopMonitorOutput{
        .goto_next_stop = true,
        .routing_result_proto = std::move(next_stop_result),
    };
  }
}

}  // namespace qcraft::planner::route
