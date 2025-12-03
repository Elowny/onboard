#include "onboard/control/benchmark/control_test_config_setup.h"

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {

TEST(LoadLiteProtoTest, LoadTrajectoryProtoTest) {
  TrajectoryProto traj = BuildTrajectoryProtoForTest();
  EXPECT_GT(traj.trajectory_point_size(), 10);
}

TEST(LoadLiteProtoTest, LoadChassisProtoTest) {
  Chassis chassis = BuildChassisForTest();
  EXPECT_TRUE(chassis.has_steering_percentage());
}

TEST(LoadLiteProtoTest, LoadAutonomyStateProtoTest) {
  AutonomyStateProto autonomy = BuildAutonomyStateProtoForTest();
  EXPECT_TRUE(autonomy.has_autonomy_state());
}

TEST(LoadLiteProtoTest, LoadPoseTest) {
  PoseProto pose = BuildPoseForTest();
  EXPECT_TRUE(pose.has_pos_smooth());
}

TEST(LoadLiteProtoTest, LoadControlCommandTest) {
  ControlCommand control_cmd = BuildControlCommandForTest();
  EXPECT_TRUE(control_cmd.has_steering_target());
}

TEST(LoadLiteProtoTest, LoadControllerDebugTest) {
  ControllerDebugProto control_debug = BuildControllerDebugProtoForTest();
  EXPECT_TRUE(control_debug.has_mpc_debug_proto());
}

TEST(LoadControlParamTest, Test0) {
  ControllerParams conf = BuildControllerParams("Q1001");
  EXPECT_TRUE(conf.controller_conf.has_max_acceleration_cmd());
  EXPECT_TRUE(conf.vehicle_geo_params.has_wheel_base());
  EXPECT_TRUE(conf.vehicle_drive_params.has_max_steer_angle());
}

}  // namespace
}  // namespace qcraft::control
