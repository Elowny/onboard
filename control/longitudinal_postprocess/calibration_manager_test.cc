#include "onboard/control/longitudinal_postprocess/calibration_manager.h"

#include "gtest/gtest.h"

#include "onboard/math/proto/piecewise_linear_function.pb.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-5;

TEST(CalibrationManager, Version_1_Init_Failed) {
  VehicleDriveParamsProto vehicle_drive_param;
  vehicle_drive_param.set_enable_calibration_v2(false);
  const auto lon_calibration = std::make_shared<CalibrationManager>();
  EXPECT_EQ(lon_calibration->Init(vehicle_drive_param),
            absl::InvalidArgumentError("Not have version 1.0 parameters!"));
}

TEST(CalibrationManager, Version_2_Init_Failed) {
  VehicleDriveParamsProto vehicle_drive_param;
  vehicle_drive_param.set_enable_calibration_v2(true);
  const auto lon_calibration = std::make_shared<CalibrationManager>();
  EXPECT_EQ(lon_calibration->Init(vehicle_drive_param),
            absl::InvalidArgumentError("Version 2.0 not parameters!"));
}

TEST(CalibrationManager, Version_1) {
  VehicleDriveParamsProto vehicle_drive_param;
  vehicle_drive_param.set_enable_calibration_v2(false);
  vehicle_drive_param.set_throttle_deadzone(16.0);
  vehicle_drive_param.set_brake_deadzone(20.0);
  const auto calibration_table =
      vehicle_drive_param.mutable_calibration_table();

  auto calibration = calibration_table->add_calibration();
  calibration->set_speed(0.0);
  calibration->set_acceleration(0.0);
  calibration->set_command(14.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(0.0);
  calibration->set_acceleration(1.0);
  calibration->set_command(20.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(10.0);
  calibration->set_acceleration(0.0);
  calibration->set_command(20.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(10.0);
  calibration->set_acceleration(1.0);
  calibration->set_command(40.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(0.0);
  calibration->set_acceleration(-1.0);
  calibration->set_command(-16.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(10.0);
  calibration->set_acceleration(-1.0);
  calibration->set_command(-30.0);

  const auto lon_calibration = std::make_shared<CalibrationManager>();
  EXPECT_EQ(lon_calibration->Init(vehicle_drive_param), absl::OkStatus());

  double calibration_value =
      lon_calibration->UpdateCalibrationValue(0.0, 0.0, Chassis::GEAR_DRIVE);
  EXPECT_NEAR(calibration_value, 14.0, kEpsilon);

  CalibrationCmd calibration_cmd =
      lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  calibration_value =
      lon_calibration->UpdateCalibrationValue(5.0, 0.5, Chassis::GEAR_DRIVE);
  EXPECT_NEAR(calibration_value, 23.5, kEpsilon);

  calibration_cmd = lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 23.5, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  calibration_value =
      lon_calibration->UpdateCalibrationValue(5.0, -0.5, Chassis::GEAR_DRIVE);
  EXPECT_NEAR(calibration_value, -3.0, kEpsilon);

  calibration_cmd = lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);
}

TEST(CalibrationManager, Version_2) {
  VehicleDriveParamsProto vehicle_drive_param;
  vehicle_drive_param.set_enable_calibration_v2(true);
  vehicle_drive_param.set_throttle_deadzone(16.0);
  vehicle_drive_param.set_brake_deadzone(20.0);
  const auto calibration_table_v2 =
      vehicle_drive_param.mutable_calibration_table_v2();

  calibration_table_v2->mutable_idle_v_a_plf()->add_x(-3.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.5);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(0.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.4);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(1.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.2);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(1.8);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(5.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(-0.2);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(10.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(-0.4);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(15.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(-0.7);

  calibration_table_v2->mutable_a_throttle_plf()->add_x(-3.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(50.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(0.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(16.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(1.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(20.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(2.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(40.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(3.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(50.0);

  calibration_table_v2->mutable_a_brake_plf()->add_x(-4.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(40.0);
  calibration_table_v2->mutable_a_brake_plf()->add_x(-1.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(30.0);
  calibration_table_v2->mutable_a_brake_plf()->add_x(0.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(20.0);
  calibration_table_v2->mutable_a_brake_plf()->add_x(4.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(60.0);

  calibration_table_v2->mutable_a_gain_wrt_speed_plf()->add_x(0.0);
  calibration_table_v2->mutable_a_gain_wrt_speed_plf()->add_y(1.0);
  calibration_table_v2->mutable_a_gain_wrt_speed_plf()->add_x(30.0);
  calibration_table_v2->mutable_a_gain_wrt_speed_plf()->add_y(1.0);

  const auto lon_calibration = std::make_shared<CalibrationManager>();
  EXPECT_EQ(lon_calibration->Init(vehicle_drive_param), absl::OkStatus());

  double calibration_value =
      lon_calibration->UpdateCalibrationValue(0.0, 0.0, Chassis::GEAR_DRIVE);
  EXPECT_NEAR(calibration_value, -23.98, kEpsilon);

  CalibrationCmd calibration_cmd =
      lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 23.98, kEpsilon);

  calibration_value =
      lon_calibration->UpdateCalibrationValue(5.0, 0.5, Chassis::GEAR_DRIVE);
  EXPECT_NEAR(calibration_value, 18.8, kEpsilon);

  calibration_cmd = lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 18.8, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  calibration_value =
      lon_calibration->UpdateCalibrationValue(5.0, -1.2, Chassis::GEAR_DRIVE);
  EXPECT_NEAR(calibration_value, -30.0, kEpsilon);

  calibration_cmd = lon_calibration->ComputeLongitudinalCmd(calibration_value);
  calibration_value = lon_calibration->UpdateCalibrationValue(
      -3.0, -1.0, Chassis::GEAR_REVERSE);
  EXPECT_NEAR(calibration_value, 33.0, kEpsilon);

  calibration_cmd = lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 33.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  calibration_value =
      lon_calibration->UpdateCalibrationValue(-3.0, 1.0, Chassis::GEAR_REVERSE);
  EXPECT_NEAR(calibration_value, -25.0, kEpsilon);

  calibration_cmd = lon_calibration->ComputeLongitudinalCmd(calibration_value);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 25.0, kEpsilon);
}

}  // namespace
}  // namespace qcraft::control
