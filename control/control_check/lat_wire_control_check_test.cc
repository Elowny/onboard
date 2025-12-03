#include "onboard/control/control_check/lat_wire_control_check.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/math/util.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {
namespace {

TEST(LonWireControlCheck, NotAutoMode) {
  const auto steer_delay = 0.5;
  const auto wheel_base = 3.0;
  const auto steer_ratio = 20.0;
  const auto max_steer_angle = 9.0;
  auto check_ptr = std::make_unique<LatWireControlChecker>(
      steer_delay, wheel_base, steer_ratio, max_steer_angle);
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(2.1);
  vehicle_state.set_is_auto_steer(false);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  const bool is_onboard_mode = true;
  WireControlCheckDebugProto debug;
  const bool status = check_ptr->IsAbnormal(vehicle_state, control_cmd,
                                            is_onboard_mode, &debug);
  EXPECT_EQ(status, false);
  EXPECT_EQ(debug.lat_abnormal(), false);
}

TEST(LonWireControlCheck, ShiftGear) {
  const auto steer_delay = 0.5;
  const auto wheel_base = 3.0;
  const auto steer_ratio = 20.0;
  const auto max_steer_angle = 9.0;
  auto check_ptr = std::make_unique<LatWireControlChecker>(
      steer_delay, wheel_base, steer_ratio, max_steer_angle);
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(2.1);
  vehicle_state.set_is_auto_steer(true);
  vehicle_state.set_gear(Chassis::GEAR_NONE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  const bool is_onboard_mode = true;
  WireControlCheckDebugProto debug;
  const bool status = check_ptr->IsAbnormal(vehicle_state, control_cmd,
                                            is_onboard_mode, &debug);
  EXPECT_EQ(status, false);
  EXPECT_EQ(debug.lat_abnormal(), false);
}

TEST(WireControlCheck, SteerTest1) {
  const auto steer_delay = 0.1;
  const auto wheel_base = 3.0;
  const auto steer_ratio = 20.0;
  const auto max_steer_angle = 9.0;
  auto check_ptr = std::make_unique<LatWireControlChecker>(
      steer_delay, wheel_base, steer_ratio, max_steer_angle);
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(20.0);
  vehicle_state.set_is_auto_steer(true);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  vehicle_state.set_chassis_steering_percentage(0.0);
  control_cmd.set_steering_target(30.0);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  control_cmd.mutable_custom_command()->set_low_speed_freespace(false);
  bool is_onboard_mode = true;
  WireControlCheckDebugProto debug;
  bool status = false;
  for (int i = 0; i < FloorToInt(0.2 * kControlFrequency); ++i) {
    status = check_ptr->IsAbnormal(vehicle_state, control_cmd, is_onboard_mode,
                                   &debug);
    EXPECT_EQ(status, true);
    EXPECT_EQ(debug.lat_abnormal(), true);
  }
}

TEST(WireControlCheck, SteerTest2) {
  const auto steer_delay = 0.1;
  const auto wheel_base = 3.0;
  const auto steer_ratio = 20.0;
  const auto max_steer_angle = 9.0;
  auto check_ptr = std::make_unique<LatWireControlChecker>(
      steer_delay, wheel_base, steer_ratio, max_steer_angle);
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(0.5);
  vehicle_state.set_is_auto_steer(true);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  vehicle_state.set_chassis_steering_percentage(0.0);
  control_cmd.set_steering_target(30.0);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  control_cmd.mutable_custom_command()->set_low_speed_freespace(false);
  bool is_onboard_mode = false;
  WireControlCheckDebugProto debug;
  bool status = false;
  for (int i = 0; i < FloorToInt(0.2 * kControlFrequency); ++i) {
    status = check_ptr->IsAbnormal(vehicle_state, control_cmd, is_onboard_mode,
                                   &debug);
    EXPECT_EQ(status, false);
    EXPECT_EQ(debug.lat_abnormal(), false);
  }
}

}  // namespace
}  // namespace control
}  // namespace qcraft
