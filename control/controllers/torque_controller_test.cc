#include "onboard/control/controllers/torque_controller.h"

#include <string>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/steering_converter.h"
#include "onboard/math/util.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-3;

TEST(TorqueControllerTest, NotAutoMode) {
  SteeringConverter steering_converter(2.8, 16.0, 10.0);

  const std::string config_path =
      "onboard/control/testdata/steer_torque/steer_torque_config.pb.txt";
  ControllerSteerTorqueProto steer_torque_proto;
  file_util::TextFileToProto(config_path, &steer_torque_proto);

  TorqueController torque_controller(steer_torque_proto);
  ControllerDebugProto control_debug;

  VehicleStateProto vehicle_state;
  ControlCommand control_command;
  const TorqueControllerInput input = {
      .is_auto_steer = false,
      .is_lka = false,
      .steer_angle_target_past = 0.0,
      .vehicle_state = &vehicle_state,
      .control_cmd = &control_command,
      .steering_converter = &steering_converter};
  const double torque_cmd =
      torque_controller.ComputeSteerTorqueTarget(input, &control_debug);
  EXPECT_NEAR(0.0, torque_cmd, kEpsilon);
}

TEST(TorqueControllerTest, StaticTorque) {
  SteeringConverter steering_converter(2.8, 16.0, 10.0);

  const std::string config_path =
      "onboard/control/testdata/steer_torque/steer_torque_config.pb.txt";
  ControllerSteerTorqueProto steer_torque_proto;
  file_util::TextFileToProto(config_path, &steer_torque_proto);
  TorqueController torque_controller(steer_torque_proto);
  double torque_cmd = 0.0;
  ControllerDebugProto control_debug;

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(0.0);
  ControlCommand control_command;
  control_command.set_steering_target(10.0);
  control_command.set_steer_speed_target(0.0);
  for (int i = 0; i < FloorToInt(kControlFrequency); ++i) {
    const TorqueControllerInput input = {
        .is_auto_steer = true,
        .is_lka = false,
        .steer_angle_target_past = 0.0,
        .vehicle_state = &vehicle_state,
        .control_cmd = &control_command,
        .steering_converter = &steering_converter};
    torque_cmd =
        torque_controller.ComputeSteerTorqueTarget(input, &control_debug);
  }
  const auto debug = control_debug.torque_controller_debug_proto();

  EXPECT_NEAR(0.3, debug.feedback_gain().Get(0), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(1), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(2), kEpsilon);
  EXPECT_NEAR(0.1, debug.feedback_gain().Get(3), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(4), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(5), kEpsilon);

  EXPECT_NEAR(0.0, debug.steer_speed_error(), kEpsilon);
  EXPECT_NEAR(1.0, debug.steer_angle_error(), kEpsilon);
  EXPECT_NEAR(0.15, debug.angle_pid_out(), kEpsilon);

  EXPECT_NEAR(0.15, debug.torque_sat(), kEpsilon);
  EXPECT_NEAR(0.3, debug.torque_forward(), kEpsilon);
  EXPECT_NEAR(0.45, torque_cmd, kEpsilon);
}

TEST(TorqueControllerTest, DynamicTorque) {
  SteeringConverter steering_converter(2.8, 16.0, 10.0);
  const std::string config_path =
      "onboard/control/testdata/steer_torque/steer_torque_config.pb.txt";
  ControllerSteerTorqueProto steer_torque_proto;
  file_util::TextFileToProto(config_path, &steer_torque_proto);
  TorqueController torque_controller(steer_torque_proto);
  double torque_cmd = 0.0;
  ControllerDebugProto control_debug;

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(5.0);
  ControlCommand control_command;
  control_command.set_steering_target(50.0);
  control_command.set_steer_speed_target(0.0);
  for (int i = 0; i < FloorToInt(kControlFrequency); ++i) {
    const TorqueControllerInput input = {
        .is_auto_steer = true,
        .is_lka = false,
        .steer_angle_target_past = 0.0,
        .vehicle_state = &vehicle_state,
        .control_cmd = &control_command,
        .steering_converter = &steering_converter};
    torque_cmd =
        torque_controller.ComputeSteerTorqueTarget(input, &control_debug);
  }
  const auto debug = control_debug.torque_controller_debug_proto();

  EXPECT_NEAR(0.5, debug.feedback_gain().Get(0), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(1), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(2), kEpsilon);
  EXPECT_NEAR(0.1, debug.feedback_gain().Get(3), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(4), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(5), kEpsilon);

  EXPECT_NEAR(0.0, debug.steer_speed_error(), kEpsilon);
  EXPECT_NEAR(5.0, debug.steer_angle_error(), kEpsilon);
  EXPECT_NEAR(0.25, debug.angle_pid_out(), kEpsilon);

  EXPECT_NEAR(0.25, debug.torque_sat(), kEpsilon);
  EXPECT_NEAR(0.3, debug.torque_forward(), kEpsilon);

  EXPECT_NEAR(0.55, debug.torque_requested(), kEpsilon);
  EXPECT_NEAR(0.55, torque_cmd, kEpsilon);
}

TEST(TorqueControllerTest, DynamicTorqueByGflags) {
  SteeringConverter steering_converter(2.8, 16.0, 10.0);
  const std::string config_path =
      "onboard/control/testdata/steer_torque/steer_torque_config.pb.txt";
  TorqueController torque_controller(config_path);
  double torque_cmd = 0.0;
  ControllerDebugProto control_debug;

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(5.0);
  ControlCommand control_command;
  control_command.set_steering_target(50.0);
  control_command.set_steer_speed_target(0.0);
  for (int i = 0; i < FloorToInt(kControlFrequency); ++i) {
    const TorqueControllerInput input = {
        .is_auto_steer = true,
        .is_lka = false,
        .steer_angle_target_past = 0.0,
        .vehicle_state = &vehicle_state,
        .control_cmd = &control_command,
        .steering_converter = &steering_converter};
    torque_cmd =
        torque_controller.ComputeSteerTorqueTarget(input, &control_debug);
  }
  const auto debug = control_debug.torque_controller_debug_proto();

  EXPECT_NEAR(0.5, debug.feedback_gain().Get(0), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(1), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(2), kEpsilon);
  EXPECT_NEAR(0.1, debug.feedback_gain().Get(3), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(4), kEpsilon);
  EXPECT_NEAR(0.0, debug.feedback_gain().Get(5), kEpsilon);

  EXPECT_NEAR(0.0, debug.steer_speed_error(), kEpsilon);
  EXPECT_NEAR(5.0, debug.steer_angle_error(), kEpsilon);
  EXPECT_NEAR(0.25, debug.angle_pid_out(), kEpsilon);

  EXPECT_NEAR(0.25, debug.torque_sat(), kEpsilon);
  EXPECT_NEAR(0.3, debug.torque_forward(), kEpsilon);

  EXPECT_NEAR(0.55, debug.torque_requested(), kEpsilon);
  EXPECT_NEAR(0.55, torque_cmd, kEpsilon);
}

}  // namespace
}  // namespace qcraft::control
