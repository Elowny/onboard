#include "onboard/control/steering_protection.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/charts/chart_util.h"

namespace qcraft::control {

namespace {

constexpr double kPredictTime = 1.4;  // s.

std::string DebugCurrControlStateToString(
    double cache_time_range, double kappa_rate_actual_mean,
    const VehicleStateProto& vehicle_state,
    const SteeringProtectionResult& steering_protection_result,
    const SteeringConverter& steering_converter) {
  const double av_kappa = steering_converter.FrontWheelAngleToKappa(
      vehicle_state.front_wheel_steering_angle());
  const double steering_rate_limitation =
      steering_converter.KappaRateToSteerRate(
          steering_protection_result.kappa_rate_upper(), av_kappa);
  const double steering_rate_actual =
      steering_converter.KappaRateToSteerRate(kappa_rate_actual_mean, av_kappa);
  return absl::StrCat(
      "Steering rate limitation is ", r2d(steering_rate_limitation),
      " deg/s at the speed of ", Mps2Kph(vehicle_state.linear_velocity()),
      " km/h. The average steering rate in the last ", cache_time_range, " s, ",
      r2d(steering_rate_actual), " deg/s, is over ", kRelaxFactorThreshold,
      " of the limitation.");
}

std::string DebugPredControlStateToString(
    double kappa_rate_predict_mean, const VehicleStateProto& vehicle_state,
    const SteeringConverter& steering_converter) {
  const double av_kappa = steering_converter.FrontWheelAngleToKappa(
      vehicle_state.front_wheel_steering_angle());
  const double steering_rate_predict = steering_converter.KappaRateToSteerRate(
      kappa_rate_predict_mean, av_kappa);
  return absl::StrCat("The predicted steering rate, ",
                      r2d(steering_rate_predict), " deg/s, in the coming ",
                      kPredictTime, "s is also beyond the limit.");
}

std::string DebugPlannerStateToString(
    double kappa_rate_planner_mean, const VehicleStateProto& vehicle_state,
    const SteeringConverter& steering_converter) {
  const double av_kappa = steering_converter.FrontWheelAngleToKappa(
      vehicle_state.front_wheel_steering_angle());
  const double steering_rate_planner = steering_converter.KappaRateToSteerRate(
      kappa_rate_planner_mean, av_kappa);
  return absl::StrCat("The current planner steering rate is ",
                      r2d(steering_rate_planner), " deg/s.");
}

}  // namespace

void WrapSteerConstraintChartData(
    double prev_kappa_cmd, const ControllerConf& controller_conf,
    const ControlCommand& control_command,
    const ControllerDebugProto& controller_debug_proto,
    vis::vantage::ChartsDataProto* chart_data) {
  SCOPED_QTRACE("WrapSteerConstraintChartData");
  if (controller_debug_proto.mpc_debug_proto().s_control_mpc_result_size() <
      kSControlHorizon) {
    return;
  }

  const double mpc_time_step = controller_conf.ts_pkmpc_controller_conf().ts();
  constexpr int kChartSize = 4;
  std::vector<double> x_t;
  std::vector<std::string> names = {"", "", "mpc_kappa", "planner_kappa"};
  std::vector<std::vector<double>> values(kChartSize);
  const std::vector<vis::Color> colors = {
      vis::Color::kTomato, vis::Color::kTomato, vis::Color::kDarkGray,
      vis::Color::kOrange};
  const std::vector<vis::vantage::ChartSeriesDataProto::PenStyle> pen_styles = {
      vis::vantage::ChartSeriesDataProto::DASHLINE,
      vis::vantage::ChartSeriesDataProto::DASHLINE,
      vis::vantage::ChartSeriesDataProto::SOLIDLINE,
      vis::vantage::ChartSeriesDataProto::DOTLINE};

  const auto& steering_protection =
      control_command.debug().simple_mpc_debug().steering_protection_result();

  if (steering_protection.kappa_rate_limit_static() <
      steering_protection.kappa_rate_limit_wrt_lat_jerk()) {
    names[0] = "psi_static_ul";
    names[1] = "psi_static_ll";
  } else {
    names[0] = "psi_jerk_ul";
    names[1] = "psi_jerk_ll";
  }

  // Set base point at the relative time -kControlInterval;
  x_t.push_back(-kControlInterval);
  values[0].push_back(prev_kappa_cmd);
  values[1].push_back(prev_kappa_cmd);
  values[2].push_back(prev_kappa_cmd);
  values[3].push_back(
      controller_debug_proto.mpc_debug_proto().s_control_kappa_ref(0));

  // Calculate limitation and MPC output.
  double kappa_cmd = prev_kappa_cmd;
  for (int i = 0; i < kSControlHorizon; ++i) {
    const double t = i * mpc_time_step;
    const double time_step = i == 0 ? kControlInterval : mpc_time_step;
    x_t.push_back(t);

    // psi limit;
    values[0].push_back(steering_protection.kappa_output_upper(i));
    values[1].push_back(steering_protection.kappa_output_lower(i));

    // MPC kappa results.
    const double mpc_cmd =
        controller_debug_proto.mpc_debug_proto().s_control_mpc_result(i);
    kappa_cmd += mpc_cmd * time_step;
    values[2].push_back(kappa_cmd);

    // Planner kappa.
    values[3].push_back(
        controller_debug_proto.mpc_debug_proto().s_control_kappa_ref(i + 1));
  }

  vis::vantage::GenerateChartFromData("control/steer_constraint", x_t, values,
                                      "t", names, colors, pen_styles,
                                      chart_data->add_charts());
}

SteeringProtectionResult SteeringProtection::CalcKappaAndKappaRateLimit(
    double previous_kappa_cmd, const VehicleStateProto& vehicle_state) const {
  SCOPED_QTRACE("SteeringProtection::CalcKappaAndKappaRateLimit");
  SteeringProtectionResult steering_protection_result;
  /*
   * Step1: Calculate kappa limit.
   */
  const double kappa_limit_wrt_geometry =
      steering_converter_->SteerAngleToKappa(max_steer_angle_);

  constexpr double kSpeedLowerLimit = 0.1;  // m/s.
  const double v =
      std::max(std::fabs(vehicle_state.linear_velocity()), kSpeedLowerLimit);
  const double kappa_limit_wrt_lat_a =
      control_conf_->max_lateral_acceleration() / Sqr(v);

  const double kappa_limit_wrt_geometry_and_speed =
      std::min(kappa_limit_wrt_geometry, kappa_limit_wrt_lat_a);

  /*
   * Step2: Calculate kappa rate limit.
   */
  const double steer_angle = steering_converter_->SteerPctToSteerAngle(
      vehicle_state.chassis_steering_percentage());
  const double kappa_rate_limit_static =
      steering_converter_->SteerRateToKappaRate(kSteeringSpeedLimitRad,
                                                steer_angle);

  // Calculate kappa_rate_limit based on lateral jerk limitation.
  // Lateral jerk ~= v^2 * psi;
  const double kappa_rate_limit_wrt_lat_jerk = kLateralkJerkLimit / Sqr(v);
  const double kappa_rate_limit_final =
      std::min(kappa_rate_limit_static, kappa_rate_limit_wrt_lat_jerk);
  QCHECK_GT(kappa_rate_limit_final, 0.0)
      << "kappa_rate_limit_final = " << kappa_rate_limit_final;

  /*
   * Step3: Record result.
   */
  steering_protection_result.set_kappa_limit_wrt_geometry(
      kappa_limit_wrt_geometry);
  steering_protection_result.set_kappa_limit_wrt_lat_a(kappa_limit_wrt_lat_a);
  steering_protection_result.set_kappa_rate_limit_static(
      kappa_rate_limit_static);
  steering_protection_result.set_kappa_rate_limit_wrt_lat_jerk(
      kappa_rate_limit_wrt_lat_jerk);

  /*
   * Step4: Calculate kappa output upper and lower sequence in lateral mpc
   * control,horizon, meanwhile suppose kappa rate upper and lower are const in
   * mpc.
   */

  // When av is not AUTO_MODE, use static limit without regard to speed and
  // lateral jerk limit.
  if (!vehicle_state.is_auto_mode()) {
    steering_protection_result.set_kappa_rate_upper(kappa_rate_limit_static);
    steering_protection_result.set_kappa_rate_lower(-kappa_rate_limit_static);
    for (int i = 0; i < kSControlHorizon; ++i) {
      steering_protection_result.add_kappa_output_upper(
          kappa_limit_wrt_geometry);
      steering_protection_result.add_kappa_output_lower(
          -kappa_limit_wrt_geometry);
    }

    return steering_protection_result;
  }

  // Av is AUTO_MODE.
  double kappa_output_upper = previous_kappa_cmd;
  double kappa_output_lower = previous_kappa_cmd;
  for (int i = 0; i < kSControlHorizon + 1; ++i) {
    if (i == 0) {
      kappa_output_upper += kappa_rate_limit_final * kControlInterval;
      kappa_output_lower -= kappa_rate_limit_final * kControlInterval;

      steering_protection_result.set_kappa_rate_upper(kappa_rate_limit_final);
      steering_protection_result.set_kappa_rate_lower(-kappa_rate_limit_final);
      continue;
    }

    const double published_kappa_output_upper =
        std::clamp(kappa_output_upper, -kappa_limit_wrt_geometry_and_speed,
                   kappa_limit_wrt_geometry_and_speed);
    const double published_kappa_output_lower =
        std::clamp(kappa_output_lower, -kappa_limit_wrt_geometry_and_speed,
                   kappa_limit_wrt_geometry_and_speed);

    QCHECK_GE(published_kappa_output_upper, published_kappa_output_lower)
        << " i = " << i
        << ", published_kappa_output_upper = " << published_kappa_output_upper
        << ", published_kappa_output_lower = " << published_kappa_output_lower;

    steering_protection_result.add_kappa_output_upper(
        published_kappa_output_upper);
    steering_protection_result.add_kappa_output_lower(
        published_kappa_output_lower);

    kappa_output_upper +=
        kappa_rate_limit_final * control_conf_->ts_pkmpc_controller_conf().ts();
    kappa_output_lower -=
        kappa_rate_limit_final * control_conf_->ts_pkmpc_controller_conf().ts();
  }

  return steering_protection_result;
}

absl::Status SteeringProtection::SteerResultStatus(
    const VehicleStateProto& vehicle_state,
    const ControlCacheManager& control_cache_manager,
    const ControllerDebugProto& controller_debug_proto,
    const SteeringProtectionResult& steering_protection_result) {
  const double av_speed = vehicle_state.linear_velocity();
  PiecewiseLinearFunction<double, double> vt(/* speed(kph) = */
                                             {3.0, 10.0, 20.0},
                                             /* time(s) =  */ {1.2, 1.0, 0.4});
  double steering_protection_kickout_time =
      vt.Evaluate(Mps2Kph(std::fabs(av_speed)));
  steering_protection_kickout_time =
      std::min(steering_protection_kickout_time, kCacheSize * kControlInterval);
  const int index_step = std::clamp(
      FloorToInt(steering_protection_kickout_time * kControlFrequency), 1,
      kCacheSize);

  if (!control_cache_manager.IsInAlwaysAutoMode(index_step)) {
    return absl::OkStatus();
  }

  // Calculate kappa rate actual mean.
  const double kappa_cmd_newest =
      control_cache_manager.QueryKappaCmd(/*step*/ 0);
  const double kappa_cmd_a_certain_time_ago =
      control_cache_manager.QueryKappaCmd(index_step);
  const double kappa_rate_actual_mean =
      (kappa_cmd_newest - kappa_cmd_a_certain_time_ago) /
      (steering_protection_kickout_time);

  // Calculate kappa rate planner mean.
  const double planner_newest =
      control_cache_manager.QueryPlannerKappa(/*step*/ 0);
  const double planner_kappa_a_certain_time_ago =
      control_cache_manager.QueryKappaCmd(index_step);
  const double kappa_rate_planner_mean =
      (planner_newest - planner_kappa_a_certain_time_ago) /
      (steering_protection_kickout_time);

  const bool is_kappa_rate_actual_over_limit =
      std::fabs(kappa_rate_actual_mean) >
      kRelaxFactorThreshold * steering_protection_result.kappa_rate_upper();

  const bool is_steering_to_one_side =
      kappa_cmd_newest * kappa_cmd_a_certain_time_ago > 0.0 &&
      std::fabs(kappa_cmd_newest) > std::fabs(kappa_cmd_a_certain_time_ago);

  // If there is no prediction, only consider actual kappa rate in past
  // time.
  if (!controller_debug_proto.has_mpc_debug_proto() ||
      controller_debug_proto.mpc_debug_proto().s_control_mpc_result_size() !=
          kSControlHorizon) {
    if (is_kappa_rate_actual_over_limit && is_steering_to_one_side) {
      const std::string err_msg =
          DebugCurrControlStateToString(
              steering_protection_kickout_time, kappa_rate_actual_mean,
              vehicle_state, steering_protection_result, *steering_converter_) +
          " " +
          DebugPlannerStateToString(kappa_rate_planner_mean, vehicle_state,
                                    *steering_converter_);

      return absl::OutOfRangeError(err_msg);
    }
    return absl::OkStatus();
  }

  // Consider future kappa rate prediction.
  const auto& s_control_mpc_result =
      controller_debug_proto.mpc_debug_proto().s_control_mpc_result();

  const int predict_steps =
      FloorToInt(kPredictTime / control_conf_->ts_pkmpc_controller_conf().ts());

  // Calculate kappa rate predict mean.
  double kappa_rate_predict_sum = 0.0;
  for (int i = 0; i < predict_steps; ++i) {
    kappa_rate_predict_sum += s_control_mpc_result[i];
  }
  const double kappa_rate_predict_mean = kappa_rate_predict_sum / predict_steps;

  const bool is_kappa_rate_predict_over_limit =
      std::fabs(kappa_rate_predict_mean) >
      kRelaxFactorThreshold * steering_protection_result.kappa_rate_upper();

  const bool is_kappa_rate_actual_and_future_same_direction =
      kappa_rate_actual_mean * kappa_rate_predict_mean > 0.0;

  if (is_kappa_rate_actual_over_limit && is_kappa_rate_predict_over_limit &&
      is_kappa_rate_actual_and_future_same_direction &&
      is_steering_to_one_side && std::fabs(av_speed) > Kph2Mps(3.0)) {
    const std::string err_msg =
        DebugCurrControlStateToString(
            steering_protection_kickout_time, kappa_rate_actual_mean,
            vehicle_state, steering_protection_result, *steering_converter_) +
        " " +
        DebugPredControlStateToString(kappa_rate_predict_mean, vehicle_state,
                                      *steering_converter_) +
        " " +
        DebugPlannerStateToString(kappa_rate_planner_mean, vehicle_state,
                                  *steering_converter_);

    return absl::OutOfRangeError(err_msg);
  }

  return absl::OkStatus();
}

}  // namespace qcraft::control
