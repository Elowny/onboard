#ifndef ONBOARD_CONTROL_CONTROLLERS_LAT_DM_MPC_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_LAT_DM_MPC_CONTROLLER_H_

#include <optional>

#include "absl/status/status.h"

#include "onboard/control/controllers/model/mpc_cost_constraint.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

class LatDmMpcController {
 public:
  LatDmMpcController(const VehicleGeometryParamsProto* vehicle_geometry_params,
                     const ControllerConf* control_conf,
                     const SteeringConverter* steering_converter);

  absl::Status ComputeControlCommand(
      const VehicleStateProto& vehicle_state,
      const TrajectoryInterface& trajectory_interface,
      const SteeringProtectionResult& steering_protection_result,
      const VehPose& predicted_pose_after_delay,
      const LonControllerOutputProto& lon_controller_output,
      ControlCommand* cmd, ControllerDebugProto* controller_debug_proto);

  void Reset(const VehicleStateProto& vehicle_state);

 private:
  // Controller state.
  double previous_front_wheel_angle_cmd_ = 0.0;

  // Controller configurations.
  MpcCost mpc_cost_conf_;
  const VehicleGeometryParamsProto* geo_params_ = nullptr;
  const ControllerConf* control_conf_ = nullptr;
  std::optional<const SteeringConverter*> steering_converter_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_LAT_KM_MPC_CONTROLLER_H_
