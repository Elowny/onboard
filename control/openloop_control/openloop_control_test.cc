#include "onboard/control/openloop_control/openloop_control.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <memory>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/math/util.h"

namespace qcraft::control {
namespace {
constexpr double kEpsilon = 1e-2;

std::unique_ptr<OpenloopControl> BuildOpenloopControl() {
  const auto wheel_base = 3.0;
  const auto steer_ratio = 20.0;
  const auto max_steer_angle = 9.0;
  return std::make_unique<OpenloopControl>(wheel_base, steer_ratio,
                                           max_steer_angle);
}

TEST(OpenloopControl, throttle_brake) {
  auto openloop_control = BuildOpenloopControl();
  openloop_control->Reset();

  ControlCommand control_cmd;
  openloop_control->CreateCommand(&control_cmd);
  EXPECT_NEAR(control_cmd.throttle(), 0.0, kEpsilon);
  EXPECT_NEAR(control_cmd.brake(), 0.0, kEpsilon);
}

TEST(OpenloopControl, acceleration) {
  auto openloop_control = BuildOpenloopControl();

  openloop_control->Reset();

  ControlCommand control_cmd;
  for (int i = 0; i < FloorToInt(50.0 * kControlFrequency); ++i) {
    openloop_control->CreateCommand(&control_cmd);
    EXPECT_NEAR(control_cmd.acceleration(), -2.0, kEpsilon);
  }
  for (int i = 0; i < FloorToInt(5.0 * kControlFrequency); ++i) {
    openloop_control->CreateCommand(&control_cmd);
    EXPECT_NEAR(control_cmd.acceleration(), -0.5, kEpsilon);
  }
}

TEST(OpenloopControl, steer_reset) {
  auto openloop_control = BuildOpenloopControl();

  openloop_control->Reset();

  ControlCommand control_cmd;
  for (int i = 0; i < FloorToInt(2.0 * kControlFrequency); ++i) {
    openloop_control->CreateCommand(&control_cmd);
    EXPECT_NEAR(control_cmd.steering_target(), 0.0, kEpsilon);
  }
}

TEST(OpenloopControl, steer_zero) {
  auto openloop_control = BuildOpenloopControl();

  OpenloopCmd openloop_cmd;
  openloop_cmd.set_testtype(OpenloopCmd::STEERZERO);
  auto throttle_brake_cmd = openloop_cmd.add_throttle_brake_cmd();
  throttle_brake_cmd->set_aperture(10.0);
  throttle_brake_cmd->set_duration_time(10.0);
  auto acceleration_cmd = openloop_cmd.add_acceleration_cmd();
  acceleration_cmd->set_acceleration(0.0);
  acceleration_cmd->set_duration_time(0.0);
  openloop_control->SetOpenloopCmd(openloop_cmd);

  ControlCommand control_cmd;
  for (int i = 0; i < FloorToInt(10.0 * kControlFrequency); ++i) {
    openloop_control->CreateCommand(&control_cmd);
    EXPECT_NEAR(control_cmd.steering_target(), 0.0, kEpsilon);
  }
}

TEST(OpenloopControl, steer_follow) {
  auto openloop_control = BuildOpenloopControl();

  const std::string file_path =
      "onboard/control/openloop_control/testdata/steer_angle.txt";
  const std::vector<std::vector<double>> datas = utils::ReadFromTxt(file_path);

  OpenloopCmd openloop_cmd;
  openloop_cmd.set_testtype(OpenloopCmd::STEERFOLLOW);
  openloop_cmd.mutable_steer_follow()->set_start_time(0.0);
  openloop_cmd.mutable_steer_follow()->set_data_name(file_path);
  auto throttle_brake_cmd = openloop_cmd.add_throttle_brake_cmd();
  throttle_brake_cmd->set_aperture(0.0);
  throttle_brake_cmd->set_duration_time(10.0);
  auto acceleration_cmd = openloop_cmd.add_acceleration_cmd();
  acceleration_cmd->set_acceleration(0.0);
  acceleration_cmd->set_duration_time(10.0);
  openloop_control->SetOpenloopCmd(openloop_cmd);

  ControlCommand control_cmd;
  const auto max_steer_angle = 9.0;
  for (int i = 1; i < FloorToInt(5.0 * kControlFrequency); ++i) {
    openloop_control->CreateCommand(&control_cmd);
    const double steer_angle = datas[i][1] / max_steer_angle * 100.0;
    EXPECT_NEAR(control_cmd.steering_target(), steer_angle, kEpsilon);
  }
}
}  // namespace
}  // namespace qcraft::control
