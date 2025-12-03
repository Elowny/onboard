#ifndef ONBOARD_CONTROL_CONTROLLER_AGENT_H_
#define ONBOARD_CONTROL_CONTROLLER_AGENT_H_

#include <optional>

#include "absl/status/status.h"

#include "onboard/control/control_cache_manager.h"
#include "onboard/control/control_parking/control_parking_manager.h"
#include "onboard/control/controllers/lat_dm_mpc_controller.h"
#include "onboard/control/controllers/lat_km_mpc_controller.h"
#include "onboard/control/controllers/lon_mpc_controller.h"
#include "onboard/control/controllers/lon_tob_mpc_controller.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

struct ControlConstraint {
  SteeringProtectionResult steering_protection_result;
  // TODO(zhichao): build a longitudinal control constraint.
};

struct ControllerInitPose {
  VehPose lon_pose;
  VehPose lat_pose_dm;
  VehPose lat_pose_km;
};

ControllerInitPose WrapControllerInitPose(
    int lon_delay_steps, int lat_delay_steps,
    const SteeringConverter& steering_converter,
    const VehicleStateProto& vehicle_state,
    const ControlCacheManager& control_cache,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    ControllerDebugProto* controller_debug_proto);

class ControllerAgent {
 public:
  ControllerAgent(const VehicleGeometryParamsProto* vehicle_geometry_params,
                  const VehicleDriveParamsProto* vehicle_drive_params,
                  const ControllerConf* control_conf,
                  const SteeringConverter* steering_converter);

  absl::Status ComputeControlCommand(
      const VehicleStateProto& vehicle_state,
      const TrajectoryInterface& trajectory_interface,
      const ControlConstraint& control_constraint,
      const ControllerInitPose& init_pose, ControlCommand* cmd,
      ControllerDebugProto* controller_debug_proto,
      LonControllerOutputProto* lon_controller_output);

  void MayBeReset(const VehicleStateProto& vehicle_state,
                  const ParkingState& parking_state);

 private:
  const ControllerConf* control_conf_ = nullptr;
  std::optional<LonMpcController> lon_mpc_controller_;
  std::optional<LonTobMpcController> lon_tob_mpc_controller_;
  std::optional<LatKmMpcController> lat_km_mpc_controller_;
  std::optional<LatDmMpcController> lat_dm_mpc_controller_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLER_AGENT_H_
