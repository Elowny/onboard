#include "onboard/control/benchmark/control_test_config_setup.h"

#include <memory>

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/utils/file_util.h"

namespace qcraft::control {

TrajectoryProto BuildTrajectoryProtoForTest() {
  TrajectoryProto trajectory;
  const std::string trajectory_file =
      "onboard/control/testdata/control_input_sample/trajectory.pb.txt";
  file_util::TextFileToProto(trajectory_file, &trajectory);
  return trajectory;
}

Chassis BuildChassisForTest() {
  Chassis chassis;
  const std::string chassis_file =
      "onboard/control/testdata/control_input_sample/chassis.pb.txt";
  file_util::TextFileToProto(chassis_file, &chassis);
  return chassis;
}

AutonomyStateProto BuildAutonomyStateProtoForTest() {
  AutonomyStateProto autonomy_state;
  const std::string autonomy_state_file =
      "onboard/control/testdata/control_input_sample/autonomy_state.pb.txt";
  file_util::TextFileToProto(autonomy_state_file, &autonomy_state);
  return autonomy_state;
}

PoseProto BuildPoseForTest() {
  PoseProto pose;
  const std::string pose_file =
      "onboard/control/testdata/control_input_sample/pose.pb.txt";
  file_util::TextFileToProto(pose_file, &pose);
  return pose;
}

ControlCommand BuildControlCommandForTest() {
  ControlCommand control_cmd;
  const std::string control_cmd_file =
      "onboard/control/testdata/control_input_sample/control_cmd.pb.txt";
  file_util::TextFileToProto(control_cmd_file, &control_cmd);
  return control_cmd;
}

ControllerDebugProto BuildControllerDebugProtoForTest() {
  ControllerDebugProto controller_debug_proto;
  const std::string controller_debug_proto_file =
      "onboard/control/testdata/control_input_sample/"
      "controller_debug_proto.pb.txt";
  file_util::TextFileToProto(controller_debug_proto_file,
                             &controller_debug_proto);
  return controller_debug_proto;
}

ControllerParams BuildControllerParams(const std::string& car_id) {
  ControllerParams controller_params;
  auto param_manager = CreateParamManagerFromCarId(car_id);
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  controller_params.vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  controller_params.vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  controller_params.controller_conf =
      run_params.vehicle_params().controller_conf();

  return controller_params;
}

}  // namespace qcraft::control
