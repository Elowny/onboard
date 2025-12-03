#include "onboard/control/control_check/lon_wire_control_check.h"

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
  auto check_ptr = std::make_unique<LonWireControlChecker>();
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(2.1);
  vehicle_state.set_is_auto_speed(false);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  WireControlCheckDebugProto debug;
  const bool status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
  EXPECT_EQ(status, false);
  EXPECT_EQ(debug.lon_abnormal(), false);
}

TEST(LonWireControlCheck, ShiftGear) {
  auto check_ptr = std::make_unique<LonWireControlChecker>();
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(2.1);
  vehicle_state.set_is_auto_speed(true);
  vehicle_state.set_gear(Chassis::GEAR_NONE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  WireControlCheckDebugProto debug;
  const bool status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
  EXPECT_EQ(status, false);
  EXPECT_EQ(debug.lon_abnormal(), false);
}

TEST(LonWireControlCheck, Freespace) {
  auto check_ptr = std::make_unique<LonWireControlChecker>();
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(3.5);
  vehicle_state.set_is_auto_speed(true);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  control_cmd.mutable_custom_command()->set_low_speed_freespace(true);
  WireControlCheckDebugProto debug;
  bool status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
  EXPECT_EQ(status, true);
  EXPECT_EQ(debug.lon_abnormal(), true);

  for (int i = 0; i < FloorToInt(6.0 * kControlFrequency); ++i) {
    vehicle_state.set_linear_velocity(1.5);
    vehicle_state.set_linear_acceleration(2.0);
    status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
    if (i > FloorToInt(1.2 * kControlFrequency)) {
      EXPECT_EQ(status, true);
      EXPECT_EQ(debug.lon_abnormal(), true);
    }
  }
}

TEST(LonWireControlCheck, Throttle) {
  auto check_ptr = std::make_unique<LonWireControlChecker>();
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(2.1);
  vehicle_state.set_is_auto_speed(true);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  control_cmd.mutable_custom_command()->set_low_speed_freespace(false);
  WireControlCheckDebugProto debug;
  bool status = false;
  for (int i = 0; i < FloorToInt(6.0 * kControlFrequency); ++i) {
    const int size = FloorToInt(1.0 * kControlFrequency);
    if (i < size) {
      vehicle_state.set_linear_acceleration(0.0);
      status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
      EXPECT_EQ(status, false);
    } else if (i < FloorToInt(2.0 * kControlFrequency)) {
      vehicle_state.set_linear_acceleration(0.03 * (i - size));  // 1.5m/s2
      status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
      EXPECT_EQ(status, false);
    } else if (i < FloorToInt(3.5 * kControlFrequency)) {
      vehicle_state.set_linear_acceleration(0.03 * (i - size));
      status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
      EXPECT_EQ(status, false);
    } else {
      vehicle_state.set_linear_acceleration(0.03 * (i - size));
      status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
      EXPECT_EQ(status, true);
    }
  }
}

TEST(LonWireControlCheck, Brake) {
  constexpr double kEpsilon = 1e-5;
  auto check_ptr = std::make_unique<LonWireControlChecker>();

  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  vehicle_state.set_linear_velocity(10.0);
  vehicle_state.set_is_auto_speed(true);
  vehicle_state.set_gear(Chassis::GEAR_DRIVE);
  control_cmd.set_gear_location(Chassis::GEAR_DRIVE);
  control_cmd.mutable_custom_command()->set_low_speed_freespace(false);
  WireControlCheckDebugProto debug;

  bool status = false;
  for (int i = 0; i < FloorToInt(4.0 * kControlFrequency); ++i) {
    control_cmd.set_acceleration(-2.0);
    vehicle_state.set_linear_acceleration(-1.0);
    status = check_ptr->IsAbnormal(vehicle_state, control_cmd, &debug);
    EXPECT_EQ(status, false);
    if (i > FloorToInt(3.0 * kControlFrequency))
      EXPECT_NEAR(debug.brake_response_rate(), 0.5, kEpsilon);
  }
}

}  // namespace
}  // namespace control
}  // namespace qcraft
