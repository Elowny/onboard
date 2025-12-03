#ifndef ONBOARD_PLANNER_PLANNER_MODULE_H_
#define ONBOARD_PLANNER_PLANNER_MODULE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"

#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/hmi/proto/alc.pb.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_module.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_output.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/parking_spot_finder.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/parking/parking_freespace.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_run_event_states.pb.h"
#include "onboard/proto/q_run_events.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/semantic_map_modification.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::planner {
class PlannerModule : public LiteModule {
 public:
  explicit PlannerModule(LiteClientBase* client);
  ~PlannerModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

  // Preprocess planner input before calling RunMainLoop, also used in snapshot
  // runner
  absl::Status PreprocessInput(PlannerInput* input);

  // The planner main loop that reads planner input and runs in planner
  // onboard/offboard environment.
  // TODO(lidong): Make this a stateless function.
  PlannerStatus RunMainLoop(const PlannerInput& input, PlannerOutput* output);

  // Publish planner outputs. Lite message publishing function requires the
  // message is a mutable.
  void PublishOutput(PlannerOutput* mutable_output,
                     const PlannerStatus& traj_status);

  // Publish planner output asynchronously.
  void PublishOutputAsync(const PlannerStatus& traj_status);

  // Getters and setters
  bool IsPlannerSnapshotMode() const { return this->planner_snapshot_mode_; }

  std::string DebugString() const;

  // Getters&Setters
  const PlannerInput& GetPlannerInput() const { return onboard_input_; }

  const PlannerOutput GetPlannerOutput() const {
    absl::MutexLock guard(&output_mutex_);
    return *onboard_output_;
  }

 private:
  void HandlePose(std::shared_ptr<const PoseProto> pose);
  void HandlePrediction(
      const std::shared_ptr<const ObjectsPredictionProto>& prediction);
  void HandlePredictionDebug(
      std::shared_ptr<const PredictionDebugProto> prediction_debug);
  void HandleLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);
  void HandleObjects(std::shared_ptr<const ObjectsProto> objects);
  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy);
  void HandleTrafficLightStates(
      std::shared_ptr<const TrafficLightStatesProto> tl_states);
  void HandleDriverAction(std::shared_ptr<const DriverAction> driver_action);
  void HandleRemoteAssistToCar(
      std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car);
  void HandleRoutingManagerOutputResult(
      std::shared_ptr<const RouteManagerOutputProto> route_manager_output);
  void HandleChassis(std::shared_ptr<const Chassis> chassis);
  void HandlePlannerState(
      std::shared_ptr<const PlannerStateProto> planner_state);
  void HandleSemanticMapPatch(
      std::shared_ptr<const SemanticMapModificationProto> semantic_map_mod);
  void HandleRunEventStates(std::shared_ptr<const QRunEventStatesProto> states);
  void HandleOraclePrediction(
      const std::shared_ptr<const ObjectsPredictionProto>& oracle_prediction);
  void HandleOracleAvTrajectory(
      std::shared_ptr<const TrajectoryProto> oracle_av_trajectory);
  void HandleSensorFovs(std::shared_ptr<const SensorFovsProto> sensor_fovs);
  // TODO(jiayu): useless delete later.
  void HandleDriverCommand(
      std::shared_ptr<const DriverCommandProto> driver_cmd);
  void HandleOnlineSemanticMap(
      std::shared_ptr<const mapping::OnlineSemanticMapProto>
          online_semantic_map);
  void HandleOnlineMapProto(
      const std::shared_ptr<const OnlineMapProto>& online_map_proto);
  void HandleParkingSpotFinder(
      std::shared_ptr<const ParkingSpotFinderProto> parking_spot_finder);
  void HandleFusionFusionParkingFreespace(
      std::shared_ptr<const FusionParkingFreespaceProto>
          fusion_parking_freespace);
  void HandleQRunEvents(
      const std::shared_ptr<const QRunEventsProto>& q_run_events_proto);
  void MaybeInjectTeleopProto();

  absl::Status CheckInput(const PlannerInput& input);

  void UpdateOnlineHDMap(PlannerInput* input);

  // The onboard wrapper function.
  void MainLoop();

  bool IsFirstSimulationFrame() const { return simulation_frame_ == 0; }

  // This function checks if a trajectory computed in non-auto mode is ready to
  // enter auto mode. Returns OK if driver can engage.
  absl::Status CheckIfDriverCanEngage(const TrajectoryProto& trajectory);

  // Planner main thread pool.
  std::unique_ptr<ThreadPool> thread_pool_;
  std::unique_ptr<ThreadPool> pub_thread_pool_;

  // Contains all the input data to start an iteration of planner.
  // NOTE(lidong): Move all the input to this data structure.
  PlannerInput onboard_input_;

  // Contains the output data of planner module. It should include published
  // proto messages, produced events, issues, canvas etc.
  // NOTE(lidong): Move all the output data to this structure.
  std::unique_ptr<PlannerOutput> onboard_output_
      ABSL_GUARDED_BY(output_mutex_) = std::make_unique<PlannerOutput>();
  mutable absl::Mutex output_mutex_;

  Future<void> publish_planner_state_future_;
  Future<std::shared_ptr<PlannerStateProto>> planner_state_to_proto_future_;
  Future<void> publish_output_future_;

  PlannerState planner_state_;
  ExternalCommandInfo ext_cmd_info_;

  bool planner_snapshot_mode_ = false;
  std::shared_ptr<const PlannerStateProto> playback_planner_state_;
  std::string semantic_map_dir_;
  int simulation_frame_ = 0;

  // Use to recover semantic_map_modifier for snapshot
  int64_t run_event_state_seq_num_ = 0;

  absl::Time last_iteration_time_;

  mutable std::shared_ptr<
      mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc>
      hd_semantic_map_listener_;
  std::shared_ptr<const SemanticMapModificationProto>
      latest_semantic_map_modification_;

  bool pending_load_hd_psmm_ = false;
  Future<std::shared_ptr<PlannerSemanticMapManager>> psmm_future_;

  std::shared_ptr<ParkingSpotFinderProto> custom_parking_spot_ = nullptr;
};

REGISTER_LITE_MODULE(PlannerModule);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_MODULE_H_
