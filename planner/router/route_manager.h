#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_H_

#include <stdint.h>

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "boost/circular_buffer.hpp"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/proto/route_external_command.pb.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_error.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/route_module_input.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft {
namespace planner {

namespace internal {
bool IsStopArrived(double distance);
}

// TODO(xiang): merge RoutingInput RouteManagerInput into one struct.
struct RouteManagerInput {
  explicit RouteManagerInput(const route::RoutingInput& routing_input) {
    res_button_pressed = routing_input.res_button_pressed;
    autonomy_state = routing_input.autonomy_state;
    pose = routing_input.pose;
    recorded_route = routing_input.recorded_route;
    log_routing_result = routing_input.routing_result;
    localization_transform = routing_input.localization_transform;
    planner_reroute = routing_input.external_router_command ==
                      ExternalRouterCommand::INPLACE_REROUTE;
    pred_trajectory_point = routing_input.pred_trajectory_point;
    switch_alternate_route = routing_input.external_router_command ==
                             ExternalRouterCommand::SWITCH_ROUTE;
  }

  bool res_button_pressed = false;
  bool switch_alternate_route = false;
  bool planner_reroute = false;
  // from input, constant
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const PoseProto> pose;
  std::shared_ptr<const RecordedRouteProto> recorded_route;
  std::shared_ptr<const RoutingResultProto> log_routing_result;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::optional<Vec2d> pred_trajectory_point;
};

class RouteManager {
 public:
  explicit RouteManager(
      const mapping::v2::SemanticMapManager* semantic_map_manager,
      const route::MapIndex* map_index,
      const RouteParamProto* route_param_proto)
      : route_core_(route::RouteCore(semantic_map_manager, route_param_proto)),
        semantic_map_manager_(semantic_map_manager),
        map_index_(map_index),
        route_param_proto_(route_param_proto) {}

  bool has_route() const { return data_.has_route(); }
  bool has_offroad_route() const;

  RouteContentProto GenerateRouteContent() const;

  // initialization of the route manager at the start of run
  void InitRequest(
      const std::shared_ptr<const RecordedRouteProto>& recorded_route,
      const std::shared_ptr<const RoutingResultProto>& log_routing_result,
      int64_t injected_teleop_micro_secs);

  // Return the routing result as a RouteManagerOutput struct.
  absl::StatusOr<RouteManagerOutput> Update(bool is_bus,
                                            const RouteManagerInput& input);

  void RoutingResetRequest();

  absl::Status AddManualReroutingRequest(const ReroutingRequestProto& request);

  RoutingResultProto* mutable_routing_result_proto() {
    return &data_.routing_result_proto;
  }

  RouteManagerState* mutable_route_manager_state() { return &data_; }
  const RouteManagerState& route_manager_state() const { return data_; }
  void InjectState(RouteManagerState state);

  const RouteManagerStateDebugProto& route_manager_state_debug_proto() const {
    return data_.rms_debug;
  }

  const RoutingResultProto& GetRoutingResultProto() const {
    return data_.routing_result_proto;
  }

  const route::RouteErrorCode& GetLastError() const { return last_error_; }
  void ClearLastError() { last_error_ = route::OkRouteStatus(); }
  void ClearIterationState();

 private:
  static void UpdateTurnSignalOnRouteStartOrEnd(double start_next_route_time,
                                                double distance_to_next_stop,
                                                const double now_in_seconds,
                                                TurnSignal* signal);

  bool ReceivedSuccessfulRoutingResult(const RoutingResultProto& response);

  absl::Status UpdateRouteFromCurrent(
      bool is_bus, const CompositeLanePathProto& route_path_from_current);

  absl::Status UpdateStationsInfo(
      const CompositeLanePathProto& route_path_from_current);

  void UpdateDistance();
  void ResetRoutePathInfo();  // call when track vehicle failed.

  void UpdateAlternativeRoute(bool is_bus);
  absl::Status Reroute(
      const mapping::v2::SemanticMapManager& semantic_map_manager,
      const route::MapIndex& map_index,
      const CoordinateConverter& coordinate_converter,
      const RouteParamProto& route_param_proto, const PoseProto& pose,
      const RoutingRequestProto& origin_request,
      const MultipleStopsRequest& multi_stops,
      const RouteSectionSequenceProto& route_section_sequence_proto,
      int64_t request_id, bool is_bus, route::RouteCore* route_core,
      RouteManagerState* data);

  absl::StatusOr<RoutingResultProto> InitiateRoutingRequest(
      const RouteManagerInput& input, bool is_bus);

  RouteManagerState data_;
  boost::circular_buffer<double> vx_history_ =
      boost::circular_buffer<double>(10);
  route::RouteErrorCode last_error_;
  route::RouteCore route_core_;

  // Not owned.
  const mapping::v2::SemanticMapManager* semantic_map_manager_;
  const route::MapIndex* map_index_;
  const RouteParamProto* route_param_proto_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_H_
