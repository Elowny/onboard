#include "onboard/control/control_parking/stationary_steer_control.h"

#include <memory>

#include "gtest/gtest.h"

namespace qcraft {
namespace control {
namespace {
TEST(StationarySteerControl, NoEntry) {
  const double wheel_base = 3.0;
  const double steer_ratio = 20.0;
  const double max_steer_angle = 8.0;
  auto steering_converter = std::make_unique<const SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);
  VehicleStateProto vehicle_state;
  SteeringProtectionResult steering_protection_result;
  StationarySteerControlInput input = {
      .is_stationary_steer = false,
      .kappa_cmd = 0.1,
      .kappa_trajectory = 0.04,
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter.get(),
  };
  StationarySteerControl stationary_steer_control;
  const double kappa_cmd =
      stationary_steer_control.ComputestationarySteerCmd(input);
  EXPECT_NEAR(kappa_cmd, input.kappa_cmd, 1e-3);
}  // namespace

TEST(StationarySteerControl, Entry) {
  const double wheel_base = 3.0;
  const double steer_ratio = 20.0;
  const double max_steer_angle = 8.0;
  auto steering_converter = std::make_unique<const SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);
  vehicle_state.set_front_wheel_steering_angle(0.15);

  SteeringProtectionResult steering_protection_result;
  steering_protection_result.set_kappa_limit_wrt_geometry(0.15);
  steering_protection_result.set_kappa_limit_wrt_lat_a(0.2);
  StationarySteerControlInput input = {
      .is_stationary_steer = true,
      .kappa_cmd = 0.1,
      .kappa_trajectory = 0.04,
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter.get(),
  };
  StationarySteerControl stationary_steer_control;
  const double kappa_cmd =
      stationary_steer_control.ComputestationarySteerCmd(input);
  EXPECT_NEAR(kappa_cmd, input.kappa_trajectory, 1e-3);
}  // namespace
}  // namespace

}  // namespace control
}  // namespace qcraft
