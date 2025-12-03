#include "onboard/control/lka_control/lka_control.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/control/lka_control/path_to_trajectory.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::control {
namespace {

absl::StatusOr<TrajectoryProto> BuildTrajectory() {
  std::vector<PathPoint> path;
  for (int i = 0; i < 20; i++) {
    PathPoint path_point;
    path_point.set_x(100.0 * std::cos(0.1 * i) - 100.0);
    path_point.set_y(100.0 * std::sin(0.1 * i));
    path_point.set_z(0.0);
    path_point.set_theta(0.1 * i);
    path_point.set_kappa(0.01);
    path.push_back(path_point);
  }
  return PathToTrajectory(path);
}

PoseProto BuildPose() {
  PoseProto pose;
  pose.set_speed(10.0);
  pose.set_yaw(0.0);
  pose.set_curvature(0.01);
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.mutable_pos_smooth()->set_z(0.0);
  pose.mutable_vel_body()->set_x(10.0);
  pose.mutable_ar_body()->set_z(0.1);
  pose.mutable_accel_body()->set_x(0.0);
  return pose;
}

Chassis BuildChassis() {
  Chassis chassis;
  chassis.set_steering_percentage(0.0);
  chassis.set_steering_torque_nm(0.0);
  chassis.set_gear_location(Chassis::GEAR_DRIVE);
  chassis.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  return chassis;
}

TEST(LKAController, Compute) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  VehicleDriveParamsProto vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();
  ControllerConf controller_conf =
      run_params.vehicle_params().controller_conf();

  ControllerSteerTorqueProto control_torque_params;
  file_util::FileToProto(
      "onboard/lane_support/params/lane_support_torque_params.pb.txt",
      &control_torque_params);
  controller_conf.mutable_steer_torque_conf()->CopyFrom(control_torque_params);

  LKAController lka_control(
      &controller_conf, &vehicle_geometry_params, vehicle_drive_params,
      run_params.vehicle_params().vehicle_params().model());
  ControlCommand control_command;
  ControllerDebugProto controller_debug_proto;

  auto status = absl::InternalError("failed !");
  PoseProto pose = BuildPose();
  Chassis chassis = BuildChassis();
  auto trajectory_or = BuildTrajectory();
  if (trajectory_or.ok()) {
    status = lka_control.ComputeControlCommand(trajectory_or.value(), pose,
                                               chassis, &control_command,
                                               &controller_debug_proto);
  } else {
    lka_control.Reset();
  }
  const auto control_error = control_command.debug().control_error();
  EXPECT_EQ(status, absl::OkStatus());
  EXPECT_NEAR(control_error.lateral_error(), 0.0, 1e-3);
  EXPECT_NEAR(control_error.heading_error(), 0.0, 1e-3);
}

TEST(LKAController, Reset) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  VehicleDriveParamsProto vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();
  ControllerConf controller_conf =
      run_params.vehicle_params().controller_conf();

  ControllerSteerTorqueProto control_torque_params;
  file_util::FileToProto(
      "onboard/lane_support/params/lane_support_torque_params.pb.txt",
      &control_torque_params);
  controller_conf.mutable_steer_torque_conf()->CopyFrom(control_torque_params);

  LKAController lka_control(
      &controller_conf, &vehicle_geometry_params, vehicle_drive_params,
      run_params.vehicle_params().vehicle_params().model());
  ControlCommand control_command;
  ControllerDebugProto controller_debug_proto;

  auto status = absl::InternalError("failed !");
  PoseProto pose = BuildPose();
  Chassis chassis = BuildChassis();
  TrajectoryProto trajectory;
  if (!trajectory.trajectory_point().empty()) {
    status = lka_control.ComputeControlCommand(
        trajectory, pose, chassis, &control_command, &controller_debug_proto);
  } else {
    lka_control.Reset();
  }
  EXPECT_EQ(status, absl::InternalError("failed !"));
}

}  // namespace
}  // namespace qcraft::control
