#ifndef ONBOARD_PLANNER_ROUTER_MONITOR_STOP_MONITOR_H_
#define ONBOARD_PLANNER_ROUTER_MONITOR_STOP_MONITOR_H_

#include "absl/status/statusor.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
namespace internal {
bool IsNearBusStop(const mapping::v2::SemanticMapManager& semantic_map_manager,
                   mapping::ElementId lane_id, double fraction,
                   const Vec2d& current_pos, double radius);
}

struct StopMonitorInput {
  RoutingRequestContext context;
  const MultipleStopsRequest* multi_stops = nullptr;
  const RouteManagerStateDebugProto* rms_debug = nullptr;
  // TODO(xiang): Replace pose with global after PlannerRoutingRequestProto
  // removed.
  const PoseProto* pose = nullptr;
  int next_stop_index = -1;
  bool res_button_pressed = false;
  bool is_bus = false;
};
struct StopMonitorOutput {
  bool goto_next_stop = false;
  // Exist only when `goto_next_stop` is true.
  RoutingResultProto routing_result_proto;
};

bool ShouldGoNextStop(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, const MultipleStopsRequest& multi_stops,
    int next_stop_index, const Vec2d& av_global2d);

absl::StatusOr<RoutingResultProto> GotoNextStop(const StopMonitorInput& input,
                                                const CoordinateConverter& cc,
                                                int new_next_dest_idx,
                                                RouteCore* route_core);

absl::StatusOr<StopMonitorOutput> MonitorStopStatus(
    const StopMonitorInput& input, const CoordinateConverter& cc,
    RouteCore* route_core);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_MONITOR_STOP_MONITOR_H_
