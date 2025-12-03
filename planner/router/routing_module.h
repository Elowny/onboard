#ifndef ONBOARD_PLANNER_ROUTER_ROUTING_MODULE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTING_MODULE_H_

#include <stdint.h>

#include <memory>
#include <utility>

#include "absl/status/status.h"  // for Status

#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_module.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/assist/proto/external_command_status.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_manager.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/route_module_input.h"
#include "onboard/planner/router/task/route_reset_action.h"
#include "onboard/planner/router/thirdparty/external_route_manager.h"
#include "onboard/planner/router/util/async_retry.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/q_run_event_states.pb.h"
#include "onboard/proto/q_run_events.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft {
namespace planner {

class RoutingModule : public LiteModule {
 public:
  explicit RoutingModule(LiteClientBase* client);
  virtual ~RoutingModule() = default;

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 protected:
  void HandlePose(std::shared_ptr<const PoseProto> pose) {
    input_.pose = std::move(pose);
  }

  void HandleLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform) {
    input_.localization_transform = std::move(localization_transform);
  }

  void HandleRemoteAssistToCar(
      std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car) {
    input_.remote_assist_to_car = std::move(remote_assist_to_car);
  }

  void HandleReroutingRequest(
      std::shared_ptr<const ReroutingRequestProto> rerouting_request);

  void HandleRunEventStates(std::shared_ptr<const QRunEventStatesProto> states);

  void HandleRunEvents(std::shared_ptr<const QRunEventsProto> run_events);

  void HandleRecordedRoute(
      std::shared_ptr<const RecordedRouteProto> recorded_route) {
    input_.recorded_route = std::move(recorded_route);
  }

  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy) {
    input_.autonomy_state = std::move(autonomy);
  }

  void HandleDriverAction(
      std::shared_ptr<const DriverAction> driver_action) {  // NOLINT
    if (driver_action->has_press_res_button() &&
        driver_action->press_res_button()) {
      input_.res_button_pressed = true;
    }
  }

  void HandleGnssRawReadingProto(
      std::shared_ptr<const GnssRawReadingProto> gnss);

  void HandleMppProto(std::shared_ptr<const MppSectionsProto> mpp);

  void HandleRoutingResultProto(
      std::shared_ptr<const RoutingResultProto> routing_result_proto);

  void HandleRoutingStateProto(
      std::shared_ptr<const RoutingStateProto> routing_state_proto);

  void HandlePlannerExternalCommandStatus(
      std::shared_ptr<const PlannerExternalCommandStatusProto>
          planner_external_command_status_proto);

  void HandleRouteServiceRequestProto(
      std::shared_ptr<const RouteServiceRequestProto>
          route_service_request_proto);

  void HandlePlannerStateProto(
      std::shared_ptr<const PlannerStateProto> planner_state_proto);

  void HandleOnlineSemanticMap(
      std::shared_ptr<const mapping::OnlineSemanticMapProto>
          online_semantic_map);

  void HandleSdRouteProto(std::shared_ptr<const SDRouteProto> sd_route_proto);

  void HandleSdMatchResultProto(std::shared_ptr<const SdRouteMatchResultProto>
                                    sd_route_match_result_proto);

  // -----------------teleop related with route.
  void ProcessTeleopProto(const RemoteAssistToCarProto& teleop_proto);

  void MaybeInjectTeleopProto(RouteManager* route_manager,
                              int64_t injected_teleop_micro_secs);

  void MainLoop();

 private:
  void CleanBeforeNextIteration();
  void ProcessRoutingResetRequest(const RouteManagerState& rms);
  absl::Status NoaMainLoop();
  void ProcessRouteManagerOutputError(
      const absl::Status& route_manager_output_status, int64_t now_microsec);
  void CheckFreqAndPublishRoutingResult(bool is_reroute, int64_t now_microsec);
  void RecoverSnapshotSim();

 private:
  std::shared_ptr<mapping::v2::SemanticMapSpatialIndexListenerAsnyc>
      semantic_map_listener_;
  std::shared_ptr<mapping::v2::SemanticMapManager> smm_;
  RoutingStateProto::RouteState route_state_;
  route::RoutingInput input_;
  std::unique_ptr<RouteManager> route_mgr_;
  std::unique_ptr<route::ExternalRouteManager> external_route_manager_;
  RoutingResultProto::SuggestNaviAction last_navi_action_;
  int64_t last_publish_microsecs_ = 0;
  route::RouteResetAction route_reset_action_;
  RouteParamProto route_param_proto_;
  ReroutingRequestProto last_rerouting_request_;
  RouteManagerOutputProto last_route_mgr_output_proto_;
  AsyncRetry async_retry_;
  bool is_bus_ = false;
  TrajectoryProto prev_trajectory_;
  std::unique_ptr<route::MapIndex> map_index_;
  int64_t init_time_microsec_ = 0;
  int64_t module_ok_microsec_ = 0;
};

REGISTER_LITE_MODULE(RoutingModule);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTING_MODULE_H_
