#ifndef ONBOARD_CONTROL_VEHICLE_CONTROL_MODULE_H_
#define ONBOARD_CONTROL_VEHICLE_CONTROL_MODULE_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"

#include "onboard/control/closed_loop_acc/speed_mode_manager.h"
#include "onboard/control/control_cache_manager.h"
#include "onboard/control/control_check/lat_wire_control_check.h"
#include "onboard/control/control_check/lon_wire_control_check.h"
#include "onboard/control/control_parking/control_parking_manager.h"
#include "onboard/control/controller_agent.h"
#include "onboard/control/controllers/torque_controller.h"
#include "onboard/control/lateral_postprocess/mrac_control.h"
#include "onboard/control/lateral_postprocess/steer_calibration.h"
#include "onboard/control/longitudinal_postprocess/lon_postprocess.h"
#include "onboard/control/openloop_control/openloop_control.h"
#include "onboard/control/parameter_identification/parameter_identification.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_module.h"
#include "onboard/params/dynamic_param/dynamic_param.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {

// The vehicle control module is communication with the vehicle, including
// sending commands and receiving status reports.
class VehicleControlModule : public LiteModule {
 public:
  explicit VehicleControlModule(LiteClientBase* lite_client);
  ~VehicleControlModule();

  void Proc();
  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 private:
  struct LocalView {
    std::shared_ptr<const AutonomyStateProto> autonomy_state;
    std::shared_ptr<const Chassis> chassis;
    std::shared_ptr<const TrajectoryProto> trajectory;
    std::shared_ptr<const PoseProto> pose;
    std::shared_ptr<const LocalizationViewerDebugProto> localization_debug;
  };

  // Upon receiving pad message
  void OnAutonomyState(
      std::shared_ptr<const AutonomyStateProto> autonomy_state);
  void OnChassis(std::shared_ptr<const Chassis> chassis);
  void OnTrajectory(std::shared_ptr<const TrajectoryProto> trajectory);
  void OnPoseProto(std::shared_ptr<const PoseProto> pose);
  void OnLocalizationDebug(
      std::shared_ptr<const LocalizationViewerDebugProto> localization_debug);

  absl::Status ProduceControlCommand(
      ControlCommand* control_command,
      ControllerDebugProto* controller_debug_proto);

  absl::Status UpdateInput(const LocalView& local_view,
                           ControllerDebugProto* controller_debug_proto);

  absl::Status CheckTimestamp(const LocalView& local_view, bool is_input_ready);

  std::unique_ptr<ControllerAgent> controller_agent_;
  ControllerConf control_conf_;
  std::optional<ParameterIdentificator> parameter_identificator_;
  VehicleStateInterface vehicle_state_interface_;
  std::unique_ptr<TrajectoryInterface> trajectory_interface_;
  std::unique_ptr<LonWireControlChecker> lon_wire_control_checker_;
  std::unique_ptr<LatWireControlChecker> lat_wire_control_checker_;

  VehicleGeometryParamsProto vehicle_geometry_params_;
  VehicleDriveParamsProto vehicle_drive_params_;
  DynamicParamProto dynamic_param_;
  std::optional<SteerCalibration> steer_calibration_;
  std::unique_ptr<const SteeringConverter> steering_converter_;
  std::unique_ptr<OpenloopControl> openloop_controller_;
  std::unique_ptr<MracControl> mrac_control_;
  std::unique_ptr<ClosedLoopAcc> acc_closed_loop_;
  std::unique_ptr<LonPostProcess> lon_post_process_manager_;
  std::unique_ptr<ParkingManager> parking_manager_;

  LocalView local_view_;

  bool planner_trajectory_ready_ = false;
  bool is_input_ready_ = false;

  ControlCacheManager control_cache_mgr_;

  std::unique_ptr<TorqueController> torque_controller_;
};

REGISTER_LITE_MODULE(VehicleControlModule);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_VEHICLE_CONTROL_MODULE_H_
