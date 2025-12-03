#ifndef ONBOARD_PLANNER_ROUTER_SIM_SNAPSHOT_SIM_STATE_H_
#define ONBOARD_PLANNER_ROUTER_SIM_SNAPSHOT_SIM_STATE_H_
#include "absl/status/statusor.h"

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/route.pb.h"
namespace qcraft::planner::route {
struct RouteSnapshotSimInput {
  const RoutingResultProto* routing_result_proto;
  const RouteManagerStateDebugProto* rms_debug;
};

// Use pointer for reducing the cost of proto transformation.
void RecoverRouteProto(const RoutingResultProto& routing_result_proto,
                       RouteProto* route);

// Only recover the states used for calc route.
absl::StatusOr<RouteManagerState> RecoverPartialSnapshotState(
    const mapping::v2::SemanticMapManager& smm,
    const route::MapIndex* map_index, const RouteSnapshotSimInput& input);

absl::StatusOr<RouteManagerState> RecoverBasicSnapshotState(
    const RouteSnapshotSimInput& input);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_SIM_SNAPSHOT_SIM_STATE_H_
