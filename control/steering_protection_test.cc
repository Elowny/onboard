#include "onboard/control/steering_protection.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 0.0001;

TEST(SteeringProtection, CalcKappaAndKappaRateLimitWhenAvControllableTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto& control_conf = run_params.vehicle_params().controller_conf();
  SteeringConverter steering_converter(vehicle_geo_params,
                                       vehicle_drive_params);

  constexpr int kMpcHorizon = 10;
  const double previous_kappa_cmd = 0.1;
  const double av_speed_list[] = {-5.0, -1.0, 0.0, 0.05, 1.0, 5.0};
  const double steer_pct_lit[] = {-50.0, -1.0, 0.0, 1.0, 50.0};

  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);

  for (const auto& av_speed : av_speed_list) {
    for (const auto& steer_pct : steer_pct_lit) {
      SteeringProtection steering_protection(
          vehicle_drive_params, &steering_converter, &control_conf);

      ControlCommand control_cmd;
      // Kappa rate ground truth.
      constexpr double kSpeedLowerLimit = 0.1;  // m/s.
      const double v = std::max(std::fabs(av_speed), kSpeedLowerLimit);
      const double steer_angle =
          steering_converter.SteerPctToSteerAngle(steer_pct);
      const double kappa_rate_limit_static =
          steering_converter.SteerRateToKappaRate(kSteeringSpeedLimitRad,
                                                  steer_angle);

      const double kappa_rate_limit_wrt_lat_jerk = kLateralkJerkLimit / Sqr(v);
      const double kappa_rate_limit_final =
          std::min(kappa_rate_limit_static, kappa_rate_limit_wrt_lat_jerk);

      // Kappa limit ground truth.
      const double kappa_limit_wrt_geometry =
          steering_converter.SteerAngleToKappa(
              vehicle_drive_params.max_steer_angle());

      const double kappa_limit_wrt_lat_a =
          control_conf.max_lateral_acceleration() / Sqr(v);

      const double kappa_limit_wrt_geometry_and_speed =
          std::min(kappa_limit_wrt_geometry, kappa_limit_wrt_lat_a);

      vehicle_state.set_linear_velocity(av_speed);
      vehicle_state.set_chassis_steering_percentage(steer_pct);
      SteeringProtectionResult result =
          steering_protection.CalcKappaAndKappaRateLimit(previous_kappa_cmd,
                                                         vehicle_state);

      EXPECT_NEAR(result.kappa_rate_upper(), kappa_rate_limit_final, kEpsilon)
          << "kappa rate limit final: " << kappa_rate_limit_final;
      EXPECT_NEAR(result.kappa_rate_lower(), -kappa_rate_limit_final, kEpsilon);

      EXPECT_EQ(result.kappa_output_upper_size(), kMpcHorizon)
          << "kappa output upper size: " << result.kappa_output_upper_size()
          << " | expect result: " << kMpcHorizon;
      EXPECT_EQ(result.kappa_output_lower_size(), kMpcHorizon);

      double kappa_output_uppper =
          previous_kappa_cmd + kappa_rate_limit_final * kControlInterval;
      double kappa_output_lower =
          previous_kappa_cmd - kappa_rate_limit_final * kControlInterval;
      for (int i = 0; i < kMpcHorizon; ++i) {
        EXPECT_NEAR(result.kappa_output_upper(i), kappa_output_uppper, kEpsilon)
            << "index: " << i;
        EXPECT_NEAR(result.kappa_output_lower(i), kappa_output_lower, kEpsilon);
        kappa_output_uppper += kappa_rate_limit_final *
                               control_conf.ts_pkmpc_controller_conf().ts();
        kappa_output_uppper =
            std::min(kappa_output_uppper, kappa_limit_wrt_geometry_and_speed);
        kappa_output_lower -= kappa_rate_limit_final *
                              control_conf.ts_pkmpc_controller_conf().ts();
        kappa_output_lower =
            std::max(kappa_output_lower, -kappa_limit_wrt_geometry_and_speed);
      }
    }
  }
}

TEST(SteeringProtection,
     CalcKappaAndKappaRateLimitTestWhenAvUncontrollableTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto& control_conf = run_params.vehicle_params().controller_conf();
  SteeringConverter steering_converter(vehicle_geo_params,
                                       vehicle_drive_params);

  constexpr int kMpcHorizon = 10;
  double previous_kappa_cmd = 0.1;
  //   double front_wheel_angle_list[] = {-0.1, -0.01, 0.0, 0.01, 0.1};
  const double steer_pct_lit[] = {-10.0, -1.0, 0.0, 1.0, 10.0};

  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(false);
  vehicle_state.set_linear_velocity(10.0);

  for (const auto& steer_pct : steer_pct_lit) {
    SteeringProtection steering_protection(vehicle_drive_params,
                                           &steering_converter, &control_conf);

    ControlCommand control_cmd;

    // Kappa rate ground truth.
    const double steer_angle =
        steering_converter.SteerPctToSteerAngle(steer_pct);
    const double kappa_rate_limit_static =
        steering_converter.SteerRateToKappaRate(kSteeringSpeedLimitRad,
                                                steer_angle);

    // Kappa limit ground truth.
    const double kappa_limit_wrt_geometry =
        steering_converter.SteerAngleToKappa(
            vehicle_drive_params.max_steer_angle());

    vehicle_state.set_chassis_steering_percentage(steer_pct);

    SteeringProtectionResult result =
        steering_protection.CalcKappaAndKappaRateLimit(previous_kappa_cmd,
                                                       vehicle_state);

    EXPECT_NEAR(result.kappa_rate_upper(), kappa_rate_limit_static, kEpsilon);
    EXPECT_NEAR(result.kappa_rate_lower(), -kappa_rate_limit_static, kEpsilon);

    EXPECT_EQ(result.kappa_output_upper_size(), kMpcHorizon);
    EXPECT_EQ(result.kappa_output_lower_size(), kMpcHorizon);

    for (int i = 0; i < kMpcHorizon; ++i) {
      EXPECT_NEAR(result.kappa_output_upper(i), kappa_limit_wrt_geometry,
                  kEpsilon);
      EXPECT_NEAR(result.kappa_output_lower(i), -kappa_limit_wrt_geometry,
                  kEpsilon);
    }
  }
}

// Test whether unreasonable previous_kappa_cmd will lead to crash.
TEST(SteeringProtection, CalcKappaAndKappaRateLimitCrashTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto& control_conf = run_params.vehicle_params().controller_conf();
  SteeringConverter steering_converter(vehicle_geo_params,
                                       vehicle_drive_params);

  ControlCommand control_cmd;
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);
  vehicle_state.set_linear_velocity(5.0);  // m/s.
  vehicle_state.set_chassis_steering_percentage(-30.0);
  const double previous_kappa_cmd = -0.5;

  SteeringProtection steering_protection(vehicle_drive_params,
                                         &steering_converter, &control_conf);
  SteeringProtectionResult result =
      steering_protection.CalcKappaAndKappaRateLimit(previous_kappa_cmd,
                                                     vehicle_state);
}

TEST(SteeringProtection, SteerResultStatusTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  auto control_conf = run_params.vehicle_params().controller_conf();
  control_conf.clear_active_controllers();
  control_conf.add_active_controllers(ControllerConf::TOB_TSPKMPC_CONTROLLER);
  SteeringConverter steering_converter(vehicle_geo_params,
                                       vehicle_drive_params);
  SteeringProtection steering_protection(vehicle_drive_params,
                                         &steering_converter, &control_conf);

  ControlCommand control_cmd;
  ControllerDebugProto controller_debug_proto;

  SteeringProtectionResult result;
  constexpr double kKappaRateUpper = 0.1;  // rad/s.
  result.set_kappa_rate_upper(kKappaRateUpper);

  ControlCacheManager control_cache_mgr;

  VehicleStateProto vehicle_state;
  vehicle_state.set_linear_velocity(5.0);
  vehicle_state.set_is_auto_mode(true);

  // Case 1: Cache data is not full.
  for (int i = 0; i < /*engage step*/ 10; ++i) {
    control_cmd.set_curvature(i * kKappaRateUpper);
    control_cache_mgr.UpdateCacheData(control_cmd, vehicle_state, {});
  }
  EXPECT_TRUE(steering_protection
                  .SteerResultStatus(vehicle_state, control_cache_mgr,
                                     controller_debug_proto, result)
                  .ok());

  // Case 2: the current autonomy state is not under control.
  for (int i = 0; i < kCacheSize + 1; ++i) {
    control_cmd.set_curvature(i * kKappaRateUpper);
    control_cache_mgr.UpdateCacheData(control_cmd, vehicle_state, {});
  }
  vehicle_state.set_is_auto_mode(false);
  control_cache_mgr.UpdateCacheData(control_cmd, vehicle_state, {});

  EXPECT_TRUE(steering_protection
                  .SteerResultStatus(vehicle_state, control_cache_mgr,
                                     controller_debug_proto, result)
                  .ok());

  // Case 3: Consider steering speed only in past time without mpc predict
  // horizon.
  vehicle_state.set_is_auto_mode(true);
  const double relax_factor_list[] = {-1.1,
                                      -kRelaxFactorThreshold - 0.01,
                                      -kRelaxFactorThreshold + 0.01,
                                      -0.5,
                                      0.0,
                                      0.5,
                                      kRelaxFactorThreshold - 0.01,
                                      kRelaxFactorThreshold + 0.01,
                                      1.1};
  const double av_speed_list[] = {-2.0, -1.0, 0.0, 1.0, 2.0};

  for (const auto& av_speed : av_speed_list) {
    for (const auto& relax_factor : relax_factor_list) {
      for (int i = 0; i < kCacheSize + 1; ++i) {
        control_cmd.set_curvature(i * relax_factor * kKappaRateUpper *
                                  kControlInterval);
        control_cache_mgr.UpdateCacheData(control_cmd, vehicle_state, {});
      }
      EXPECT_EQ(steering_protection
                    .SteerResultStatus(vehicle_state, control_cache_mgr,
                                       controller_debug_proto, result)
                    .ok(),
                std::fabs(relax_factor) < kRelaxFactorThreshold)
          << "relax_factor: " << relax_factor << " | av_speed: " << av_speed
          << " | kappa rate upper: " << result.kappa_rate_upper();
    }
  }

  // Case 4: Consider the steering speed in past time and tob ts pk mpc predict
  // horizon.
  const double relax_factor_list_1[] = {
      -kRelaxFactorThreshold - 0.01, -kRelaxFactorThreshold + 0.01,
      kRelaxFactorThreshold - 0.01, kRelaxFactorThreshold + 0.01};

  SteeringProtection steering_protection_1(vehicle_drive_params,
                                           &steering_converter, &control_conf);
  for (const auto& actual_relax_factor : relax_factor_list_1) {
    for (const auto& predict_relax_factor : relax_factor_list_1) {
      for (int i = 0; i < kCacheSize + 1; ++i) {
        control_cmd.set_curvature(i * actual_relax_factor * kKappaRateUpper *
                                  kControlInterval);
        control_cache_mgr.UpdateCacheData(control_cmd, vehicle_state, {});
      }
      constexpr int kMpcHorizon = 10;
      auto* mpc_debug = controller_debug_proto.mutable_mpc_debug_proto();
      mpc_debug->Clear();
      for (int i = 0; i < kMpcHorizon; ++i) {
        mpc_debug->add_s_control_mpc_result(predict_relax_factor *
                                            kKappaRateUpper);
      }

      const bool is_protective_kickout_gt =
          std::fabs(actual_relax_factor) >= kRelaxFactorThreshold &&
          std::fabs(predict_relax_factor) >= kRelaxFactorThreshold &&
          actual_relax_factor * predict_relax_factor > 0.0;
      EXPECT_EQ(steering_protection_1
                    .SteerResultStatus(vehicle_state, control_cache_mgr,
                                       controller_debug_proto, result)
                    .ok(),
                !is_protective_kickout_gt)
          << "actual_relax_factor: " << actual_relax_factor
          << " | predict_relax_factor: " << predict_relax_factor;
    }
  }
}

}  // namespace
}  // namespace qcraft::control
