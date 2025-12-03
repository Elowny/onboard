#include "onboard/control/parameter_identification/parameter_identification.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move

#include "gtest/gtest.h"

#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-5;
ParameterIdentificator::SteerBiasIdentificationInputData MakeInputData(
    double front_wheel_angle, double kappa) {
  return {.front_wheel_angle = front_wheel_angle, .kappa = kappa};
}

TEST(ParameterIdentificatorTest, CalculateSteerBias) {
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  ControllerConf control_conf;
  vehicle_drive_params.set_steer_ratio(10.0);
  vehicle_drive_params.set_max_steer_angle(10.0);
  vehicle_geometry_params.set_wheel_base(4.5);
  SteeringConverter steering_converter(vehicle_geometry_params,
                                       vehicle_drive_params);
  const double init_steer_bias = 0.1;

  ParameterIdentificator parameter_identificator(
      control_conf, &steering_converter, init_steer_bias);

  EXPECT_NEAR(parameter_identificator.SteerBiasOutput(), init_steer_bias,
              kEpsilon);
  EXPECT_NEAR(
      parameter_identificator.CalculateSteerBias(MakeInputData(0.1, 0.01)),
      0.099783304438, kEpsilon);

  EXPECT_NEAR(
      parameter_identificator.CalculateSteerBias(MakeInputData(0.2, 0.03)),
      0.0997474641171, kEpsilon);
}

TEST(ParameterIdentificatorTest, EstimateLatBias) {
  ControllerConf control_conf;
  BiasEstimationDebug bias_estimation_debug;
  qcraft::control::ParameterIdentificationInput input;
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  vehicle_drive_params.set_steer_ratio(16.0);
  vehicle_drive_params.set_max_steer_angle(10.0);
  vehicle_geometry_params.set_wheel_base(2.9);
  SteeringConverter steering_converter(vehicle_geometry_params,
                                       vehicle_drive_params);

  control_conf.mutable_bias_estimation_conf()
      ->set_use_low_pass_filter_estimation(true);
  control_conf.mutable_bias_estimation_conf()->set_enable_compensate_yaw_bias(
      true);
  control_conf.mutable_bias_estimation_conf()->set_weight_on_steering(1);
  control_conf.mutable_bias_estimation_conf()->set_weight_on_heading(100);

  const double init_steer_bias = 0.0;
  ParameterIdentificator parameter_identificator(
      control_conf, &steering_converter, init_steer_bias);

  ControlCacheManager control_cache_manager;
  input.control_cache_mgr = &control_cache_manager;
  input.is_auto = true;

  input.speed_measurement = control_conf.bias_estimation_conf().vel_lb() * 2.0;
  parameter_identificator.EstimateLatBias(input, &bias_estimation_debug);
  EXPECT_NEAR(bias_estimation_debug.heading_bias(), 0.0, kEpsilon);

  // Updated Yaw bias when all requirements satisfied.
  input.heading_err =
      control_conf.bias_estimation_conf().heading_err_ub() / 2.0;
  parameter_identificator.EstimateLatBias(input, &bias_estimation_debug);

  // Stop Update bias when the error is too large.
  input.heading_err =
      control_conf.bias_estimation_conf().heading_err_ub() * 2.0;
  parameter_identificator.EstimateLatBias(input, &bias_estimation_debug);
  EXPECT_NEAR(bias_estimation_debug.is_bias_updated(), false, kEpsilon);
}

TEST(ParameterIdentificatorTest, CalculateSteerDelayTest) {
  ControllerConf control_conf;
  // Create a mock SteeringConverter.
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  vehicle_drive_params.set_steer_ratio(16.0);
  vehicle_drive_params.set_max_steer_angle(10.0);
  vehicle_geometry_params.set_wheel_base(2.9);
  SteeringConverter mock_steering_converter(vehicle_geometry_params,
                                            vehicle_drive_params);

  // Create a mock VehicleStateProto and set up expectations for is_auto_steer
  // and linear_velocity.
  VehicleStateProto mock_vehicle_state;
  mock_vehicle_state.set_is_auto_steer(false);
  mock_vehicle_state.set_linear_velocity(10.0);
  // CalculateSteerDelay.
  const double init_steer_bias = 0.0;
  ParameterIdentificator parameter_identificator(
      control_conf, &mock_steering_converter, init_steer_bias);
  const double kappa_cmd = 0.1;

  // Verify that the expected and actual delays are within some tolerance.
  double expected_delay = 0.1;
  double actual_delay = parameter_identificator.CalculateSteerDelay(
      mock_steering_converter, mock_vehicle_state, kappa_cmd);
  EXPECT_NEAR(expected_delay, actual_delay, kEpsilon);

  mock_vehicle_state.set_is_auto_steer(true);
  expected_delay = 0;
  actual_delay = parameter_identificator.CalculateSteerDelay(
      mock_steering_converter, mock_vehicle_state, kappa_cmd);
  EXPECT_NEAR(expected_delay, actual_delay, kEpsilon);
}

TEST(ParameterIdentificatorTest, AssembleInputData) {
  ControllerConf control_conf;
  // Create a mock SteeringConverter.
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  vehicle_drive_params.set_steer_ratio(16.0);
  vehicle_drive_params.set_max_steer_angle(6);
  vehicle_geometry_params.set_wheel_base(2.9);
  SteeringConverter mock_steering_converter(vehicle_geometry_params,
                                            vehicle_drive_params);
  const double init_steer_bias = 0.0;

  ParameterIdentificator parameter_identificator(
      control_conf, &mock_steering_converter, init_steer_bias);
  // Create a mock VehicleStateProto
  VehicleStateProto mock_vehicle_state;
  double expected_kappa = 0.1;
  mock_vehicle_state.set_kappa(expected_kappa);
  for (auto i = 0; i < 20; i++) {
    parameter_identificator.pose_curvature_cache_.push_back(expected_kappa);
  }
  auto input_data =
      parameter_identificator.AssembleInputData(mock_vehicle_state);

  EXPECT_NEAR(expected_kappa, input_data->kappa, kEpsilon);
}

TEST(ParameterIdentificatorTest, Process) {
  ControllerConf control_conf;
  // Create a mock SteeringConverter.
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  vehicle_drive_params.set_steer_ratio(16.0);
  vehicle_drive_params.set_max_steer_angle(6);
  vehicle_geometry_params.set_wheel_base(2.9);
  SteeringConverter mock_steering_converter(vehicle_geometry_params,
                                            vehicle_drive_params);
  const double init_steer_bias = 0.0;

  ParameterIdentificator parameter_identificator(
      control_conf, &mock_steering_converter, init_steer_bias);
  // Create a mock VehicleStateProto
  VehicleStateProto mock_vehicle_state;
  ControlCommand control_command;
  parameter_identificator.Process(mock_vehicle_state, &control_command, 0.0);
  // Return when not auto
  EXPECT_NEAR(0.0, parameter_identificator.steer_bias_output_, kEpsilon);

  // Return when pose_curvature_cache_ is empty
  mock_vehicle_state.set_linear_velocity(10.0);
  mock_vehicle_state.set_is_auto_mode(true);
  EXPECT_NEAR(0.0, parameter_identificator.steer_bias_output_, kEpsilon);

  // Update bias value.
  for (auto i = 0; i < 20; i++) {
    parameter_identificator.pose_curvature_cache_.push_back(0.01);
  }
  parameter_identificator.Process(mock_vehicle_state, &control_command, 0.0);
  EXPECT_LT(0.0, parameter_identificator.steer_bias_output_);
}

}  // namespace
}  // namespace qcraft::control
