#include "onboard/planner/router/sim/snapshot_sim_state.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/lane_point.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/utils/status_macros.h"
namespace qcraft::planner::route {
void RecoverRouteProto(const RoutingResultProto& routing_result_proto,
                       RouteProto* route) {
  if (routing_result_proto.has_lane_path()) {
    *route->mutable_lane_path() = routing_result_proto.lane_path();
  }
  if (routing_result_proto.has_routing_request()) {
    *route->mutable_routing_request() = routing_result_proto.routing_request();
  }
  route->set_update_id(routing_result_proto.update_id());
}

// Only recover the states used for calc route.
absl::StatusOr<RouteManagerState> RecoverPartialSnapshotState(
    const mapping::v2::SemanticMapManager& smm,
    const route::MapIndex* map_index, const RouteSnapshotSimInput& input) {
  ASSIGN_OR_RETURN(RouteManagerState data, RecoverBasicSnapshotState(input));
  if (input.routing_result_proto != nullptr) {
    ASSIGN_OR_RETURN(
        data.multi_stops,
        BuildMultipleStopsRequest(
            smm, map_index, input.routing_result_proto->routing_request()));
    RecoverRouteProto(*input.routing_result_proto, &data.route);
  }
  return data;
}

absl::StatusOr<RouteManagerState> RecoverBasicSnapshotState(
    const RouteSnapshotSimInput& input) {
  RouteManagerState data;
  if (input.routing_result_proto != nullptr) {
    RecoverRouteProto(*input.routing_result_proto, &data.route);
    data.request_id = input.routing_result_proto->update_id();
    data.routing_result_proto = *input.routing_result_proto;
  }
  if (input.rms_debug != nullptr) {
    data.rms_debug = *input.rms_debug;
    if (data.rms_debug.has_route_tracker()) {
      if (data.rms_debug.route_tracker().has_lane_point()) {
        data.last_lane_point.FromProto(
            data.rms_debug.route_tracker().lane_point());
      }
      if (data.rms_debug.route_tracker().has_composite_index()) {
        data.composite_index.FromProto(
            data.rms_debug.route_tracker().composite_index());
      }
    }
  }
  return data;
}

}  // namespace qcraft::planner::route
