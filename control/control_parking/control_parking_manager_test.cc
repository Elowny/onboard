#include "onboard/control/control_parking/control_parking_manager.h"

#include <memory>

#include "absl/status/status.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::control {
namespace {

TEST(GetParkingState, NotParking) {
  const VehicleModel vehicle_model = VEHICLE_MARVELR;

  ControlCommand cmd;
  ControllerDebugProto debug;

  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(false);
  trajectory.set_enable_stationary_steering(false);
  auto point = trajectory.mutable_trajectory_point()->Add();
  point->mutable_path_point()->set_kappa(0.1);
  trajectory.set_gear(Chassis::GEAR_REVERSE);
  TrajectoryInterface trajectory_interface(vehicle_model);
  const auto status = trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, trajectory, &debug);
  EXPECT_TRUE(status.ok());

  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);
  vehicle_state.set_linear_velocity(0.01);

  SteeringProtectionResult steering_protection_result;
  steering_protection_result.set_kappa_limit_wrt_geometry(0.15);
  steering_protection_result.set_kappa_limit_wrt_lat_a(0.2);

  const double wheel_base = 3.0;
  const double steer_ratio = 20.0;
  const double max_steer_angle = 8.0;
  auto steering_converter = std::make_unique<const SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);

  const ParkingManagerInput input = {
      .trajectory_interface = &trajectory_interface,
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter.get(),
      .is_onboard = true};
  ParkingManager manager(vehicle_model);
  const ParkingState state = manager.ParkingProcess(input, &cmd, &debug);

  EXPECT_EQ(state.reset_lon_controller, false);
  EXPECT_EQ(state.reset_lat_controller, false);
  EXPECT_EQ(cmd.has_speed(), false);
  EXPECT_EQ(cmd.has_parking_distance(), false);
}

TEST(GetParkingState, NotStationarySteer) {
  const VehicleModel vehicle_model = VEHICLE_MARVELR;

  ControlCommand cmd;
  ControllerDebugProto debug;

  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(true);
  trajectory.set_enable_stationary_steering(false);
  auto point = trajectory.mutable_trajectory_point()->Add();
  point->mutable_path_point()->set_kappa(0.1);
  trajectory.set_gear(Chassis::GEAR_REVERSE);
  TrajectoryInterface trajectory_interface(vehicle_model);
  const auto status = trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, trajectory, &debug);
  EXPECT_TRUE(status.ok());

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(0.01);
  vehicle_state.set_is_auto_mode(true);

  SteeringProtectionResult steering_protection_result;
  steering_protection_result.set_kappa_limit_wrt_geometry(0.15);
  steering_protection_result.set_kappa_limit_wrt_lat_a(0.2);

  const double wheel_base = 3.0;
  const double steer_ratio = 20.0;
  const double max_steer_angle = 8.0;
  auto steering_converter = std::make_unique<const SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);

  const ParkingManagerInput input = {
      .trajectory_interface = &trajectory_interface,
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter.get(),
      .is_onboard = true};

  ParkingManager manager(vehicle_model);
  const ParkingState state = manager.ParkingProcess(input, &cmd, &debug);

  EXPECT_EQ(state.reset_lon_controller, false);
  EXPECT_EQ(state.reset_lat_controller, false);
}

TEST(GetParkingState, StationarySteer) {
  const VehicleModel vehicle_model = VEHICLE_MARVELR;

  ControlCommand cmd;
  ControllerDebugProto debug;

  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(true);
  trajectory.set_enable_stationary_steering(true);
  trajectory.set_gear(Chassis::GEAR_REVERSE);
  trajectory.set_stop_s(0.05);
  auto point = trajectory.mutable_trajectory_point()->Add();
  point->mutable_path_point()->set_kappa(0.1);
  TrajectoryInterface trajectory_interface(vehicle_model);
  const auto status = trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, trajectory, &debug);
  EXPECT_TRUE(status.ok());

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(0.01);
  vehicle_state.set_is_auto_mode(true);

  SteeringProtectionResult steering_protection_result;
  steering_protection_result.set_kappa_limit_wrt_geometry(0.15);
  steering_protection_result.set_kappa_limit_wrt_lat_a(0.2);

  const double wheel_base = 3.0;
  const double steer_ratio = 20.0;
  const double max_steer_angle = 8.0;
  auto steering_converter = std::make_unique<const SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);

  const ParkingManagerInput input = {
      .trajectory_interface = &trajectory_interface,
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter.get(),
      .is_onboard = true};

  ParkingManager manager(vehicle_model);
  const ParkingState state = manager.ParkingProcess(input, &cmd, &debug);

  EXPECT_EQ(state.reset_lon_controller, false);
  EXPECT_EQ(state.reset_lat_controller, true);
  EXPECT_EQ(debug.parking_debug().is_stationary_steering(), true);
}

TEST(GetParkingState, StationarySteerM5) {
  const VehicleModel vehicle_model = VEHICLE_QCRAFTVEHICLE_SUV;

  ControlCommand cmd;
  ControllerDebugProto debug;

  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(true);
  trajectory.set_enable_stationary_steering(true);
  auto point = trajectory.mutable_directional_path()->add_path();
  point->set_kappa(0.1);
  trajectory.set_gear(Chassis::GEAR_REVERSE);
  TrajectoryInterface trajectory_interface(vehicle_model);
  const auto status = trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, trajectory, &debug);
  EXPECT_TRUE(status.ok());

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(0.01);
  vehicle_state.set_is_auto_mode(true);

  SteeringProtectionResult steering_protection_result;
  steering_protection_result.set_kappa_limit_wrt_geometry(0.15);
  steering_protection_result.set_kappa_limit_wrt_lat_a(0.2);

  const double wheel_base = 3.0;
  const double steer_ratio = 20.0;
  const double max_steer_angle = 8.0;
  auto steering_converter = std::make_unique<const SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);
  const ParkingManagerInput input = {
      .trajectory_interface = &trajectory_interface,
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter.get(),
      .is_onboard = true};

  ParkingManager manager(vehicle_model);
  const ParkingState state = manager.ParkingProcess(input, &cmd, &debug);

  EXPECT_EQ(state.reset_lon_controller, true);
  EXPECT_EQ(state.reset_lat_controller, true);
}

}  // namespace
}  // namespace qcraft::control
