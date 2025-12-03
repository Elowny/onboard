#include "onboard/control/controllers/lon_mpc_controller.h"

#include <string>
// IWYU pragma: no_include <memory>
// IWYU pragma: no_include "onboard/global/buffered_logger.h"

#include "gtest/gtest.h"

#include "onboard/control/controllers/lon_tob_mpc_controller.h"
#include "onboard/control/param/control_param_integrator.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/lite/check.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::control {
namespace {

TrajectoryProto BuildTrajectoryProto() {
  TrajectoryProto trajectory;
  const std::string trajectory_file =
      "onboard/control/testdata/control_input_sample/trajectory.pb.txt";
  QCHECK(file_util::TextFileToProto(trajectory_file, &trajectory));
  return trajectory;
}

Chassis BuildChassis() {
  Chassis chassis;
  const std::string chassis_file =
      "onboard/control/testdata/control_input_sample/chassis.pb.txt";
  QCHECK(file_util::TextFileToProto(chassis_file, &chassis));
  return chassis;
}

AutonomyStateProto BuildAutonomyStateProto() {
  AutonomyStateProto autonomy_state;
  const std::string autonomy_state_file =
      "onboard/control/testdata/control_input_sample/autonomy_state.pb.txt";
  QCHECK(file_util::TextFileToProto(autonomy_state_file, &autonomy_state));
  return autonomy_state;
}

PoseProto BuildPose() {
  PoseProto pose;
  const std::string pose_file =
      "onboard/control/testdata/control_input_sample/pose.pb.txt";
  QCHECK(file_util::TextFileToProto(pose_file, &pose));
  return pose;
}

VehicleStateProto BuildVehicleStateProto(
    const AutonomyStateProto_State& autonomy_state, const PoseProto& pose,
    const Chassis& chassis, const SteeringConverter& steering_converter) {
  VehicleStateInterface vehicle_state_interface;
  ControllerDebugProto debug_proto;
  QCHECK_OK(vehicle_state_interface.Update(/*yaw_bias*/ 0.0, autonomy_state,
                                           pose, chassis, std::nullopt,
                                           steering_converter, &debug_proto));

  return vehicle_state_interface.Result();
}

struct ControllerParams {
  ControllerConf controller_conf;
  VehicleGeometryParamsProto vehicle_geo_params;
  VehicleDriveParamsProto vehicle_drive_params;
};

ControllerParams BuildControllerParams() {
  ControllerParams controller_params;
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  controller_params.vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  controller_params.vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  controller_params.controller_conf =
      run_params.vehicle_params().controller_conf();

  QCHECK_OK(IntegrateControlParam(VehicleModel::VEHICLE_LINCOLN_MKZ,
                                  &controller_params.controller_conf));

  return controller_params;
}

void CheckLonControlCmdClose(const ControlCommand& cmd_a,
                             const ControlCommand& cmd_b) {
  QCHECK_EQ(cmd_a.parking_brake(), cmd_b.parking_brake());
  QCHECK_EQ(cmd_a.gear_location(), cmd_b.gear_location());
  QCHECK_NEAR(cmd_a.acceleration(), cmd_b.acceleration(), 0.05);
  QCHECK_NEAR(cmd_a.acceleration_offset(), cmd_b.acceleration_offset(), 1e-2);
  QCHECK_NEAR(cmd_a.acceleration_calibration(),
              cmd_b.acceleration_calibration(), 0.04);
}

TEST(LonMpcControllerTest, LogDataTest) {
  ControlCommand cmd_lon_mpc, cmd_tolon_mpc;
  ControllerDebugProto debug_proto;
  LonControllerOutputProto lon_controller_output;

  // Init LonMpcController.
  ControllerParams controller_params = BuildControllerParams();
  SteeringConverter steering_converter(controller_params.vehicle_geo_params,
                                       controller_params.vehicle_drive_params);
  LonMpcController lon_mpc_controller(&controller_params.controller_conf,
                                      &controller_params.vehicle_drive_params);
  LonTobMpcController lon_tompc_controller(
      &controller_params.controller_conf,
      &controller_params.vehicle_drive_params);

  // Prepare controller input.
  VehicleStateProto vehicle_state =
      BuildVehicleStateProto(BuildAutonomyStateProto().autonomy_state(),
                             BuildPose(), BuildChassis(), steering_converter);
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  QCHECK_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, BuildTrajectoryProto(), &debug_proto));
  // Predict pose after steer delay
  const VehPose predicted_pose_after_delay(
      &controller_params.vehicle_geo_params, vehicle_state);

  lon_mpc_controller.Reset(vehicle_state);
  QCHECK_OK(lon_mpc_controller.ComputeControlCommand(
      trajectory_interface, predicted_pose_after_delay, &cmd_lon_mpc,
      &debug_proto, &lon_controller_output));

  lon_tompc_controller.Reset(vehicle_state);
  QCHECK_OK(lon_tompc_controller.ComputeControlCommand(
      trajectory_interface, predicted_pose_after_delay, &cmd_tolon_mpc,
      &debug_proto, &lon_controller_output));

  CheckLonControlCmdClose(cmd_lon_mpc, cmd_tolon_mpc);
}

}  // namespace

}  // namespace qcraft::control
