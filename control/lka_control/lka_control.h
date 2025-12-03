#ifndef ONBOARD_CONTROL_LKA_CONTROL_LKA_CONTROL_H_
#define ONBOARD_CONTROL_LKA_CONTROL_LKA_CONTROL_H_

#include <memory>

#include "absl/status/status.h"

#include "onboard/control/controllers/lat_km_mpc_controller.h"
#include "onboard/control/controllers/torque_controller.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/steering_protection.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

// https://qcraft.feishu.cn/docx/TbQkdlQKUoAuRzxIiHscBZrtnmb

class LKAController {
 public:
  LKAController(const ControllerConf* control_conf,
                const VehicleGeometryParamsProto* vehicle_geometry_params,
                const VehicleDriveParamsProto& vehicle_drive_params,
                const VehicleModel& vehicle_model);

  absl::Status ComputeControlCommand(
      const TrajectoryProto& trajectory, const PoseProto& pose,
      const Chassis& chassis, ControlCommand* control_command,
      ControllerDebugProto* controller_debug_proto);

  void Reset();

 private:
  LonControllerOutputProto CreateLonControllerOutput(double acc);

  VehicleStateProto vehicle_state_;
  std::unique_ptr<const SteeringConverter> steering_converter_ptr_;
  std::unique_ptr<TrajectoryInterface> trajectory_interface_ptr_;
  std::unique_ptr<SteeringProtection> steering_protection_ptr_;
  std::unique_ptr<LatKmMpcController> lat_km_mpc_controller_ptr_;
  std::unique_ptr<TorqueController> torque_controller_ptr_;
  const VehicleGeometryParamsProto* geo_param_ = nullptr;

  bool is_first_run_ = true;
  double previous_kappa_cmd_ = 0.0;
  int steer_delay_steps_ = 0;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_LKA_CONTROL_LKA_CONTROL_H_
