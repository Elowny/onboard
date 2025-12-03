#include "onboard/control/parameter_identification/parameter_identification.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move
// IWYU pragma: no_include <ostream>
#include <algorithm>
#include <cmath>

#include "onboard/control/control_defs.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/clock.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/utils/time_util.h"

namespace qcraft::control {

namespace {

constexpr double kLowSpeedLimit = 3.0;       // m/s
constexpr double kMaxCurvatureLimit = 6e-3;  // About 1 degree for front wheel.

}  // namespace

ParameterIdentificator::ParameterIdentificator(
    const ControllerConf& control_conf,
    const SteeringConverter* steering_converter, double init_steer_bias)
    : steering_converter_(QCHECK_NOTNULL(steering_converter)),
      steer_bias_output_(init_steer_bias) {
  bias_estimation_conf_ = control_conf.bias_estimation_conf();
  weight_update_heading_ =
      bias_estimation_conf_.weight_on_heading() * kControlInterval;
  weight_update_steer_ =
      bias_estimation_conf_.weight_on_steering() * kControlInterval;
  steer_delay_.InitConf(control_conf);
  steering_bias_smoother_ =
      std::make_optional<ExponentialSmoothing>(weight_update_steer_);
  calib_steering_ =
      steering_converter->SteerAngleToFrontWheelAngle(init_steer_bias);
}

void ParameterIdentificator::Process(const VehicleStateProto& vehicle_state,
                                     ControlCommand* control_command,
                                     double last_kappa_cmd) {
  SCOPED_QTRACE("ParameterIdentificator::Process");
  // Update current steering delay.
  const double steer_delay_online =
      CalculateSteerDelay(*steering_converter_, vehicle_state, last_kappa_cmd);
  const double control_time_now_adding_steering_delay =
      ToUnixDoubleSeconds(Clock::Now()) + steer_delay_online;
  steer_delay_.steer_delay = steer_delay_online;
  control_command->mutable_debug()
      ->mutable_bias_estimation_debug()
      ->set_steer_delay_online(steer_delay_online);
  control_command->mutable_debug()->set_timestamp_adding_steering_delay(
      control_time_now_adding_steering_delay);

  // Ignore these data: manual data, very low speed data and big steering data.
  const bool is_manual_mode = !vehicle_state.is_auto_mode();
  const bool is_speed_too_low =
      vehicle_state.linear_velocity() < kLowSpeedLimit;
  const bool is_steering_too_big =
      std::abs(vehicle_state.kappa()) > kMaxCurvatureLimit;
  if (is_manual_mode || is_speed_too_low || is_steering_too_big) {
    pose_curvature_cache_.clear();
    return;
  }

  const auto steering_bias_identification_input_data =
      AssembleInputData(vehicle_state);
  if (steering_bias_identification_input_data.has_value()) {
    steer_bias_output_ =
        CalculateSteerBias(steering_bias_identification_input_data.value());
  }
  constexpr int kJudgingNum = 30000;  // about 5min valid data.
  constexpr double kSteerAngleBiasAlertThreshold = 10.0;  // unit: degree.
  if (prev_valid_result_num_ == kJudgingNum &&
      r2d(std::fabs(steer_bias_output_)) > kSteerAngleBiasAlertThreshold) {
    QEVENT("shijun", "control_alert_steer_bias_too_big", [&](QEvent* qevent) {
      qevent->AddField("steer angle bias", r2d(steer_bias_output_));
    });
  }
}

double ParameterIdentificator::CalculateSteerDelay(
    const SteeringConverter& steering_converter,
    const VehicleStateProto& vehicle_state, double last_kappa_cmd) {
  const double chassis_front_wheel_angle =
      steering_converter.KappaToFrontWheelAngle(last_kappa_cmd);

  const double delay_wrt_steer = vehicle_state.is_auto_steer()
                                     ? steer_delay_.steer_delay_plf.Evaluate(
                                           std::fabs(chassis_front_wheel_angle))
                                     : steer_delay_.canbus_steer_delay;
  if (steer_delay_.has_kappa_delay_gain_wrt_speed) {
    return (delay_wrt_steer * steer_delay_.kappa_delay_gain_wrt_speed_plf(
                                  std::fabs(vehicle_state.linear_velocity())));
  } else {
    return delay_wrt_steer;
  }
}

std::optional<ParameterIdentificator::SteerBiasIdentificationInputData>
ParameterIdentificator::AssembleInputData(
    const VehicleStateProto& vehicle_state) {
  if (pose_curvature_cache_.size() < kSteeringBiasDelayCacheSize) {
    pose_curvature_cache_.push_back(vehicle_state.kappa());
    return std::nullopt;
  }
  const double pose_curvature_wrt_steering_delay =
      pose_curvature_cache_.front();
  pose_curvature_cache_.push_back(vehicle_state.kappa());
  const double front_wheel_angle =
      steering_converter_->SteerPctToFrontWheelAngle(
          vehicle_state.chassis_steering_percentage());

  return std::make_optional<
      ParameterIdentificator::SteerBiasIdentificationInputData>(
      {.front_wheel_angle = front_wheel_angle,
       .kappa = pose_curvature_wrt_steering_delay});
}

// Detailed design doc https://qcraft.feishu.cn/docs/doccntqo8TTuM25n9S7K4DxORFv
double ParameterIdentificator::CalculateSteerBias(
    const SteerBiasIdentificationInputData& input) {
  // Using gradient to solve the result: steering_bias which minimum the
  // quadratic cost. J = a * x^2 + b * x + c, a > 0, x* = -b / 2a.
  // Iterative computation steering bias.
  constexpr int kInitValueWeightNum = 3000;
  const double current_steering_bias =
      ((prev_valid_result_num_ + kInitValueWeightNum) *
           steering_converter_->SteerAngleToFrontWheelAngle(
               steer_bias_output_) -
       (input.front_wheel_angle -
        steering_converter_->KappaToFrontWheelAngle(input.kappa))) /
      (prev_valid_result_num_ + kInitValueWeightNum + 1);

  ++prev_valid_result_num_;
  // Pay attention to the conversion from front wheel angle bias to steering
  // angle bias.
  return steering_converter_->FrontWheelAngleToSteerAngle(
      current_steering_bias);
}

// TODO(yangyu): return the result instead of writing it into debug proto if the
// result will be called somewhere.
void ParameterIdentificator::EstimateLatBias(
    const ParameterIdentificationInput& input,
    BiasEstimationDebug* bias_estimation_debug) {
  const bool is_tracking_offset_threshold_satisfied =
      (std::fabs(input.heading_err) < bias_estimation_conf_.heading_err_ub() &&
       std::fabs(input.lat_error) < bias_estimation_conf_.lateral_err_ub());
  const bool is_vel_threshold_satisfied =
      (std::fabs(input.speed_measurement) > bias_estimation_conf_.vel_lb());

  // Query last 1.0 second steer command.
  const double max_abs_kappa = std::max(
      std::fabs(input.control_cache_mgr->QueryMaxKappaCmd(kControlFrequency)),
      std::fabs(input.control_cache_mgr->QueryMaxKappaCmd(kControlFrequency)));

  const double steer_cmd_max_past_abs =
      steering_converter_->KappaToFrontWheelAngle(max_abs_kappa);
  const bool is_steering_satisfied =
      std::max(
          std::max(std::fabs(input.steer_cmd), std::fabs(input.steer_feedback)),
          steer_cmd_max_past_abs) < bias_estimation_conf_.steering_ub();

  const bool is_lat_acc_threshold_satisfied =
      (std::fabs(input.lat_acc) < bias_estimation_conf_.lat_acc_threshold());

  const int steer_delay_cycle =
      FloorToInt(steer_delay_.steer_delay * kControlFrequency);

  const double kappa_target_wrt_delay =
      input.control_cache_mgr->QueryKappaCmd(steer_delay_cycle);

  const double front_wheel_target =
      steering_converter_->KappaToFrontWheelAngle(kappa_target_wrt_delay);

  const double steering0_ub = bias_estimation_conf_.calib_steering0_ub();
  const double steer_bias_curr = std::clamp(
      input.steer_pose - front_wheel_target, -steering0_ub, steering0_ub);

  bool is_bias_updated = false;
  if (is_tracking_offset_threshold_satisfied && is_vel_threshold_satisfied &&
      is_steering_satisfied && is_lat_acc_threshold_satisfied &&
      input.is_auto) {
    is_bias_updated = true;
    if (bias_estimation_conf_.use_low_pass_filter_estimation()) {
      calib_steering_ += weight_update_steer_ * steer_bias_curr;
    } else {
      calib_steering_ = steering_bias_smoother_->Evaluate(steer_bias_curr);
    }
    if (bias_estimation_conf_.enable_compensate_yaw_bias()) {
      heading_bias_ += weight_update_heading_ * input.heading_err;
    } else {
      heading_bias_ +=
          weight_update_heading_ * (input.heading_err - heading_bias_);
    }
  }

  constexpr double kSteeringBiasThreshold =
      0.005;  // Front steering wheel, rad.
  const PiecewiseLinearFunction<double> heading_error_threshold_plf = {
      {10, 30}, {0.005, 0.002}};  // rad
  double heading_bias_threshold =
      heading_error_threshold_plf.Evaluate(std::fabs(input.speed_measurement));
  const bool is_steer_calib_overbound =
      fabs(calib_steering_) > kSteeringBiasThreshold;
  const bool is_heading_calib_overbound =
      fabs(heading_bias_) > heading_bias_threshold;

  heading_bias_ =
      std::clamp(heading_bias_, -bias_estimation_conf_.calib_heading0_ub(),
                 bias_estimation_conf_.calib_heading0_ub());

  calib_steering_ = std::clamp(calib_steering_, -steering0_ub, steering0_ub);
  steer_bias_output_ =
      steering_converter_->FrontWheelAngleToSteerAngle(calib_steering_);
  bias_estimation_debug->set_heading_bias(heading_bias_);
  bias_estimation_debug->set_steer_bias(steer_bias_output_);
  bias_estimation_debug->set_is_bias_updated(is_bias_updated);
  bias_estimation_debug->set_is_steer_calib_overbound(is_steer_calib_overbound);
  bias_estimation_debug->set_is_heading_calib_overbound(
      is_heading_calib_overbound);
  bias_estimation_debug->set_steer_delay_online(steer_delay_.steer_delay);
  bias_estimation_debug->set_heading_bias_curr(input.heading_err);
  bias_estimation_debug->set_steer_bias_curr(
      steering_converter_->FrontWheelAngleToSteerAngle(steer_bias_curr));
}

}  // namespace qcraft::control
