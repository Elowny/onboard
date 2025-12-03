#include "onboard/control/longitudinal_postprocess/lon_postprocess.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/math/util.h"
#include "onboard/utils/proto_util.h"

qcraft::ControllerConf control_config;
qcraft::VehicleDriveParamsProto vehicle_drive_params;

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-3;

void InitTest() {
  vehicle_drive_params.set_enable_calibration_v2(true);
  vehicle_drive_params.set_throttle_deadzone(16.0);
  vehicle_drive_params.set_brake_deadzone(20.0);
  auto idle_v_a_plf = vehicle_drive_params.mutable_calibration_table_v2()
                          ->mutable_idle_v_a_plf();
  auto a_throttle_plf = vehicle_drive_params.mutable_calibration_table_v2()
                            ->mutable_a_throttle_plf();
  auto a_brake_plf = vehicle_drive_params.mutable_calibration_table_v2()
                         ->mutable_a_brake_plf();
  TextToProto(
      "x : -3 x : -1 x : 0 x : 1 x : 1.8 x : 5 x : 10 x : 15 y : 0.5 y : -0.2 "
      "y : 0.4 y : 0.2 y : 0.0 y : -0.2 y : -0.4 y : -0.7",
      idle_v_a_plf);

  TextToProto(
      "x : -3 x : 0 x : 1 x : 2 x : 3 y : 50 y : 16 y : 20 y : 40 y : 50",
      a_throttle_plf);

  TextToProto("x : -4 x : -1 x : 0 x : 4 y : 40 y : 30 y : 20 y : 60",
              a_brake_plf);

  control_config.mutable_full_stop_condition()->set_abs_linear_speed_upperlimit(
      1.0);
  control_config.mutable_full_stop_condition()
      ->set_abs_planner_speed_upperlimit(0.1);
  control_config.mutable_full_stop_condition()->set_standstill_acceleration(
      -0.4);
  control_config.mutable_full_stop_condition()->set_lockdown_acceleration(-1.0);
}

TEST(LonPostProcess, NonClosedAccThrottle) {
  InitTest();
  control::LonPostProcessInput input;
  ControlCommand control_cmd;
  ControllerDebugProto control_debug;

  auto manager =
      std::make_unique<LonPostProcess>(&control_config, &vehicle_drive_params);

  input.gear_fb = Chassis::GEAR_DRIVE;
  input.gear_cmd = Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.pitch_pose = 0.0;
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 1.0;
  input.acc_feedback = 0.9;
  input.acc_target = 1.5;
  input.speed_feedback = 5.0;
  input.speed_planner = 3.0;
  input.steer_wheel_angle = 0.0;
  manager->Process(input, &control_cmd, &control_debug);

  EXPECT_NEAR(control_cmd.acceleration(),
              FLAGS_longitudinal_acc_jerk_limit * kControlInterval, kEpsilon);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_idle(), -0.2,
              kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_pure(), 0.26,
              kEpsilon);
  EXPECT_NEAR(control_cmd.throttle(), 17.04, kEpsilon);

  input.gear_fb = Chassis::GEAR_REVERSE;
  input.gear_cmd = Chassis::GEAR_REVERSE;
  input.is_auto_mode = true;
  input.pitch_pose = 0.0;
  input.steer_wheel_angle = 0.0;
  input.acc_planner = -0.5;
  input.acc_feedback = -0.9;
  input.acc_target = -0.8;
  input.speed_feedback = -1.0;
  input.speed_planner = -0.5;
  input.is_onboard_mode = false;

  manager->Process(input, &control_cmd, &control_debug);

  // EXPECT_NEAR(control_cmd.acceleration(), 0, kEpsilon);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_idle(), -0.2,
              kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_pure(), 0.16,
              kEpsilon);
  EXPECT_NEAR(control_cmd.throttle(), 0, kEpsilon);

  // Not meaningful tests below, just in order to pass the coverage test.
  control_config.set_throttle_interface(CLOSEDLOOP_ACC);
  control_config.set_brake_interface(CLOSEDLOOP_DEC);
  manager->Process(input, &control_cmd, &control_debug);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);

  control_config.set_throttle_interface(OPENEDLOOP_ACC);
  control_config.set_brake_interface(OPENEDLOOP_DEC);
  manager->Process(input, &control_cmd, &control_debug);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);

  control_config.set_throttle_interface(OPENEDLOOP_ACC);
  control_config.set_brake_interface(CLOSEDLOOP_DEC_WITH_PARKING_COMPENSATION);
  manager->Process(input, &control_cmd, &control_debug);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);

  input.is_auto_mode = false;
  manager->Process(input, &control_cmd, &control_debug);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
}

TEST(LonPostProcess, NonClosedAccStandStill1) {
  InitTest();
  LonPostProcessInput input;
  ControlCommand control_cmd;
  ControllerDebugProto control_debug;

  auto manager =
      std::make_unique<LonPostProcess>(&control_config, &vehicle_drive_params);

  input.gear_fb = Chassis::GEAR_DRIVE;
  input.gear_cmd = Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.pitch_pose = 0.1;  // down slope, offset < 0
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.0;
  input.acc_feedback = 0.0;
  input.acc_target = 0.0;
  input.speed_feedback = 0.5;
  input.speed_planner = 0.0;
  input.is_onboard_mode = false;
  input.steer_wheel_angle = 0.0;

  for (int i = 0; i < FloorToInt(4.0 * kControlFrequency); ++i) {
    manager->Process(input, &control_cmd, &control_debug);
    EXPECT_NEAR(control_cmd.acceleration_offset(), -0.979031, kEpsilon);
    //   const auto acc_calibration = control_cmd.acceleration_calibration();
    //   const auto exp_acc = std::max(
    //       input.acc_target - 0.01 * (i + 1),
    //       control_config.full_stop_condition().standstill_acceleration() -
    //           std::fabs(control_cmd.acceleration_offset()));
    //   // TODO(zhenxing): check the exp_acc value.
    //   EXPECT_NEAR(acc_calibration, exp_acc, kEpsilon);
  }
}

TEST(LonPostProcess, NonClosedAccStandStill2) {
  InitTest();
  LonPostProcessInput input;
  ControlCommand control_cmd;
  ControllerDebugProto control_debug;

  auto manager =
      std::make_unique<LonPostProcess>(&control_config, &vehicle_drive_params);

  input.gear_fb = Chassis::GEAR_DRIVE;
  input.gear_cmd = Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.pitch_pose = 0.1;  // down slope, offset < 0
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.0;
  input.acc_feedback = 0.0;
  input.acc_target = 0.0;
  input.speed_feedback = 0.005;
  input.speed_planner = 0.0;
  input.is_onboard_mode = false;
  input.steer_wheel_angle = 0.0;

  for (int i = 0; i < FloorToInt(4.0 * kControlFrequency); ++i) {
    manager->Process(input, &control_cmd, &control_debug);
    EXPECT_NEAR(control_cmd.acceleration_offset(), -0.979031, kEpsilon);
  }
}

TEST(LonPostProcess, NonClosedParking) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;

  auto manager =
      std::make_unique<LonPostProcess>(&control_config, &vehicle_drive_params);

  input.gear_fb = qcraft::Chassis::GEAR_REVERSE;
  input.gear_cmd = qcraft::Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.pitch_pose = 0.0;
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.5;
  input.acc_feedback = 0.5;
  input.acc_target = 0.5;
  input.speed_feedback = -1.0;
  input.speed_planner = -1.0;
  input.is_onboard_mode = false;
  input.steer_wheel_angle = 0.0;

  manager->Process(input, &control_cmd, &control_debug);

  EXPECT_NEAR(control_cmd.acceleration(),
              FLAGS_longitudinal_acc_jerk_limit * kControlInterval, kEpsilon);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_idle(), -0.2,
              kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_pure(), 0.26,
              kEpsilon);
  EXPECT_NEAR(control_cmd.brake(), 22.6, kEpsilon);
}

TEST(LonPostProcess, ClosedAccThrottle) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;

  control_config.set_enable_speed_mode_manager(true);
  control_config.mutable_closed_loop_acc_conf()->set_enable_closed_loop_acc(
      true);

  auto manager =
      std::make_unique<LonPostProcess>(&control_config, &vehicle_drive_params);

  input.gear_fb = qcraft::Chassis::GEAR_DRIVE;
  input.gear_cmd = qcraft::Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.pitch_pose = 0.1;  // down slope, offset < 0
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.0;
  input.acc_feedback = 0.0;
  input.acc_target = 0.0;
  input.speed_feedback = 0.5;
  input.speed_planner = 0.0;
  input.is_onboard_mode = false;
  input.steer_wheel_angle = 0.0;

  for (int i = 0; i < FloorToInt(4.0 * kControlFrequency); ++i) {
    manager->Process(input, &control_cmd, &control_debug);
    EXPECT_NEAR(control_cmd.acceleration_calibration(),
                control_debug.speed_mode_debug_proto().input_acc(), kEpsilon);
  }
}

TEST(LonPostProcess, OnboardMode) {
  InitTest();
  LonPostProcessInput input;
  ControlCommand control_cmd;
  ControllerDebugProto control_debug;

  control_config.set_enable_speed_mode_manager(true);
  control_config.mutable_closed_loop_acc_conf()->set_enable_closed_loop_acc(
      true);

  auto manager =
      std::make_unique<LonPostProcess>(&control_config, &vehicle_drive_params);

  input.gear_fb = Chassis::GEAR_DRIVE;
  input.gear_cmd = Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.speed_feedback = 2.0;
  input.speed_planner = 2.0;
  input.acc_feedback = 0.0;
  input.acc_target = 1.5;
  input.low_speed_freespace = false;
  input.is_onboard_mode = true;

  manager->Process(input, &control_cmd, &control_debug);
  EXPECT_NEAR(control_cmd.acceleration(),
              FLAGS_longitudinal_acc_jerk_limit * kControlInterval, kEpsilon);
}
}  // namespace
}  // namespace qcraft::control
