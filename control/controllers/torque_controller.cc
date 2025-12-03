#include "onboard/control/controllers/torque_controller.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <tuple>
#include <utility>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/filters/digital_filter_coefficients.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::control {

TorqueController::TorqueController(
    const ControllerSteerTorqueProto& torque_proto) {
  // Load proto config;
  steer_torque_proto_ = torque_proto;
  InitConfig();
  QLOG(INFO) << "load proto from vehicle param bin file!";
}

TorqueController::TorqueController(const std::string& proto_path) {
  QCHECK(file_util::TextFileToProto(proto_path, &steer_torque_proto_))
      << "failed to read proto file!";
  InitConfig();
  QLOG(INFO) << "load proto from gflags file : " << proto_path;
}

double GetUpAndDownErrorFactor(bool is_auto_steer, double vertical_acc,
                               RoadCondition* condition) {
  constexpr double kVerticalAccThreshold = 3.5;

  const int no_response_threshold = RoundToInt(0.3 * kControlFrequency);
  const int transition_threshold =
      RoundToInt(0.8 * kControlFrequency);  // 0.8 second

  if (!is_auto_steer) return 1.0;

  if (std::fabs(vertical_acc) > kVerticalAccThreshold) {
    condition->up_and_down_counter = 0;
    condition->is_up_and_down = true;
  }

  if (!condition->is_up_and_down) return 1.0;

  // is_up_and_down
  ++condition->up_and_down_counter;
  if (condition->up_and_down_counter < no_response_threshold) {
    return 0.0;
  } else if (condition->up_and_down_counter < no_response_threshold) {
    return static_cast<double>(condition->up_and_down_counter -
                               no_response_threshold) /
           static_cast<double>(transition_threshold - no_response_threshold);
  } else {
    condition->up_and_down_counter = 0;
    condition->is_up_and_down = false;
    return 1.0;
  }
}

double GetTransitionConditionTorqueErrorFactor(
    bool is_lka, bool is_auto_steer,
    TransitionCondition* transition_condition) {
  // eps is not for first 0.2s.
  const int active_threshold = RoundToInt(0.2 * kControlFrequency);
  // error_factor from 0 to 1 in 0.2~0.6s.
  const int transition_counter_threshold = RoundToInt(0.6 * kControlFrequency);

  if (!is_auto_steer) {
    transition_condition->is_transition = true;
    transition_condition->transition_counter = 0;
    return 0.0;
  }
  if (is_lka) return 1.0;

  if (transition_condition->transition_counter <= active_threshold) {
    ++transition_condition->transition_counter;
    return 0.0;
  }
  if (transition_condition->transition_counter <=
      transition_counter_threshold) {
    ++transition_condition->transition_counter;
    return static_cast<double>(transition_condition->transition_counter -
                               active_threshold) /
           static_cast<double>(transition_counter_threshold - active_threshold);
  }
  return 1.0;
}

double TorqueController::ComputeSteerTorqueTarget(
    const TorqueControllerInput& input, ControllerDebugProto* control_debug) {
  if (!input.is_auto_steer) {
    Reset();
    return 0.0;
  }
  QCHECK(input.control_cmd->has_steer_speed_target())
      << "Torque controller requests steer speed target.";
  const double v = input.vehicle_state->linear_velocity();

  const double steering_canbus = input.steering_converter->SteerPctToSteerAngle(
      input.vehicle_state->chassis_steering_percentage());

  double steering_control = 0.0;
  if (FLAGS_torque_use_past_steer_cmd) {
    steering_control = input.steering_converter->SteerPctToSteerAngle(
                           input.steer_angle_target_past) -
                       input.control_cmd->steer_angle_bias();
  } else {
    steering_control = input.steering_converter->SteerPctToSteerAngle(
                           input.control_cmd->steering_target()) -
                       input.control_cmd->steer_angle_bias();
  }

  // Add filter for steer speed.
  const double steer_speed_control = input.control_cmd->steer_speed_target();
  const double steer_speed_canbus =
      input.vehicle_state->chassis_steering_speed();

  // Steering angle error, rad;
  const double steer_angle_error = steering_control - steering_canbus;
  // Steering speed error， rad/s;
  const double steer_speed_error = steer_speed_error_filter_.Filter(
      steer_speed_control - steer_speed_canbus);

  // Add steer angle error limit as pid input.
  const double vertical_acc = input.vehicle_state->pose().accel_body().z();
  const double up_and_down_factor = GetUpAndDownErrorFactor(
      input.is_auto_steer, vertical_acc, &road_condition_);

  const double transition_torque_error_factor =
      GetTransitionConditionTorqueErrorFactor(input.is_lka, input.is_auto_steer,
                                              &transition_condition_);

  const double max_steer_angle_error =
      transition_torque_error_factor * up_and_down_factor *
      max_steer_angle_error_plf_->Evaluate(std::fabs(v));
  const double steer_angle_error_limited = std::clamp(
      steer_angle_error, -max_steer_angle_error, max_steer_angle_error);

  const double max_steer_speed_error =
      up_and_down_factor * transition_torque_error_factor *
      max_steer_speed_error_plf_->Evaluate(std::fabs(v));

  const double steer_speed_error_limit = std::clamp(
      steer_speed_error, -max_steer_speed_error, max_steer_speed_error);

  // Add steer torque limit as final output
  const double max_torque = max_torque_plf_->Evaluate(std::fabs(v));

  // Compute steer torque forward
  double torque_forward =
      ComputeSteerTorqueForward(v,
                                input.steering_converter->SteerPctToSteerAngle(
                                    input.control_cmd->steering_target()) -
                                    input.control_cmd->steer_angle_bias(),
                                steer_speed_control);
  torque_forward = std::clamp(torque_forward, -max_torque, max_torque);

  // Compute steer torque pid feedback
  const PIDConfig angle_pid_config = ComputeAnglePIDConfig(v);
  const PIDConfig speed_pid_config = ComputeAngleSpeedPIDConfig(v);

  pid_angle_controller_.SetConfig(angle_pid_config);
  pid_angle_controller_.Compute(steer_angle_error_limited);

  pid_angle_spd_controller_.SetConfig(speed_pid_config);
  pid_angle_spd_controller_.Compute(steer_speed_error_limit);

  // Reset pid integral
  ResetStaticTorqueIntegral(v);

  // Torque pid feedback.
  double torque_fb_cmd = pid_angle_controller_.GetPIDOut().result +
                         pid_angle_spd_controller_.GetPIDOut().result;
  torque_fb_cmd = std::clamp(torque_fb_cmd, -max_torque, max_torque);

  // Torque requested = forward + pid_feedback
  double torque_requested = torque_forward + torque_fb_cmd;

  // Add setp constraint limit for steer torque requested
  const double max_torque_increase_step =
      max_torque_speed_plf_->Evaluate(std::fabs(v)) * kControlInterval;
  torque_requested = std::clamp(
      torque_requested, torque_requested_last_ - max_torque_increase_step,
      torque_requested_last_ + max_torque_increase_step);

  // Add steer torque limit as final output
  torque_requested = std::clamp(torque_requested, -max_torque, max_torque);
  torque_requested_last_ = torque_requested;

  // Record torque controller status to controller debug.
  ControllerDebugProto_TorqueControllerDebugProto debug;
  debug.set_torque_feedback(input.vehicle_state->chassis_steering_torque());
  debug.add_feedback_gain(angle_pid_config.Kp);
  debug.add_feedback_gain(angle_pid_config.Ki);
  debug.add_feedback_gain(angle_pid_config.Kd);
  debug.add_feedback_gain(speed_pid_config.Kp);
  debug.add_feedback_gain(speed_pid_config.Ki);
  debug.add_feedback_gain(speed_pid_config.Kd);
  debug.set_steer_angle_error(steer_angle_error);
  debug.set_steer_angle_error_limited(steer_angle_error_limited);
  debug.set_steer_speed_error(steer_speed_error_limit);
  debug.set_torque_angle_p(pid_angle_controller_.GetPIDOut().offset_P);
  debug.set_torque_angle_i(pid_angle_controller_.GetPIDOut().offset_I);
  debug.set_angle_pid_out(pid_angle_controller_.GetPIDOut().result);
  debug.set_torque_speed_p(pid_angle_spd_controller_.GetPIDOut().offset_P);
  debug.set_torque_speed_i(pid_angle_spd_controller_.GetPIDOut().offset_I);
  debug.set_torque_speed_pid(pid_angle_spd_controller_.GetPIDOut().result);
  debug.set_torque_sat(torque_fb_cmd);
  debug.set_torque_forward(torque_forward);
  debug.set_torque_requested(torque_requested);
  debug.set_up_and_down_factor(up_and_down_factor);
  debug.set_transition_torque_error_factor(transition_torque_error_factor);
  control_debug->mutable_torque_controller_debug_proto()->CopyFrom(debug);

  return torque_requested;
}

void TorqueController::InitConfig() {
  QCHECK(steer_torque_proto_.steer_torque_pid_size() > 0 &&
         steer_torque_proto_.steer_torque_max_size() > 0 &&
         steer_torque_proto_.steer_torque_speed_max_size() > 0)
      << "failed to parse proto!";

  LoadSteerAngleConfigPLF();     // Angle feedback
  LoadSteerSpeedConfigPLF();     // Speed feedback
  LoadSteerTorqueMaxPLF();       // Torque max limit
  LoadSteerTorqueSpeedMaxPLF();  // Torque speed max limit
  LoadSteerSpeedErrorLimitPLF();
  LoadSteerAngleErrorLimitPLF();
  LoadSteerTorqueInterpolation2D();  // Torque feedforward

  // Default: 5Hz , large cutoff frequency.
  constexpr double kSteerSpeedCutoff = 5.0;
  std::vector<double> den_vector, num_vector;
  LpfCoefficients(kControlInterval, kSteerSpeedCutoff, &den_vector,
                  &num_vector);
  steer_speed_error_filter_.set_coefficients(den_vector, num_vector);
}

void TorqueController::LoadSteerAngleConfigPLF() {
  const auto size = steer_torque_proto_.steer_torque_pid_size();
  std::vector<double> speed_vector, p_vector, i_vector, d_vector, min_vector,
      max_vector, max_integral_vector;
  speed_vector.reserve(size);
  p_vector.reserve(size);
  i_vector.reserve(size);
  d_vector.reserve(size);
  min_vector.reserve(size);
  max_vector.reserve(size);
  max_integral_vector.reserve(size);
  for (const auto& conf : steer_torque_proto_.steer_torque_pid()) {
    speed_vector.push_back(conf.speed());
    p_vector.push_back(conf.angle_pid().kp());
    i_vector.push_back(conf.angle_pid().ki());
    d_vector.push_back(conf.angle_pid().kd());
    min_vector.push_back(conf.angle_pid().min());
    max_vector.push_back(conf.angle_pid().max());
    max_integral_vector.push_back(conf.angle_pid().max_integral());
  }
  angle_p_plf_ =
      std::make_unique<PiecewiseLinearFunction<double>>(speed_vector, p_vector);
  angle_i_plf_ =
      std::make_unique<PiecewiseLinearFunction<double>>(speed_vector, i_vector);
  angle_d_plf_ =
      std::make_unique<PiecewiseLinearFunction<double>>(speed_vector, d_vector);
  angle_min_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, min_vector);
  angle_max_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, max_vector);
  angle_max_integral_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, max_integral_vector);
}

void TorqueController::LoadSteerSpeedConfigPLF() {
  const auto size = steer_torque_proto_.steer_torque_pid_size();
  std::vector<double> speed_vector, p_vector, i_vector, d_vector, min_vector,
      max_vector, max_integral_vector;
  speed_vector.reserve(size);
  p_vector.reserve(size);
  i_vector.reserve(size);
  d_vector.reserve(size);
  min_vector.reserve(size);
  max_vector.reserve(size);
  max_integral_vector.reserve(size);
  for (const auto& conf : steer_torque_proto_.steer_torque_pid()) {
    speed_vector.push_back(conf.speed());
    p_vector.push_back(conf.speed_pid().kp());
    i_vector.push_back(conf.speed_pid().ki());
    d_vector.push_back(conf.speed_pid().kd());
    min_vector.push_back(conf.speed_pid().min());
    max_vector.push_back(conf.speed_pid().max());
    max_integral_vector.push_back(conf.speed_pid().max_integral());
  }
  speed_p_plf_ =
      std::make_unique<PiecewiseLinearFunction<double>>(speed_vector, p_vector);
  speed_i_plf_ =
      std::make_unique<PiecewiseLinearFunction<double>>(speed_vector, i_vector);
  speed_d_plf_ =
      std::make_unique<PiecewiseLinearFunction<double>>(speed_vector, d_vector);
  speed_min_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, min_vector);
  speed_max_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, max_vector);
  speed_max_integral_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, max_integral_vector);
}

void TorqueController::LoadSteerTorqueMaxPLF() {
  const auto size = steer_torque_proto_.steer_torque_max_size();
  std::vector<double> speed_vector, max_torque_vector;
  speed_vector.reserve(size);
  max_torque_vector.reserve(size);
  for (const auto& value : steer_torque_proto_.steer_torque_max()) {
    speed_vector.push_back(value.speed());
    max_torque_vector.push_back(value.max_torque());
  }
  max_torque_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, max_torque_vector);
}

void TorqueController::LoadSteerTorqueSpeedMaxPLF() {
  const auto size = steer_torque_proto_.steer_torque_speed_max_size();
  std::vector<double> speed_vector, max_torque_speed_vector;
  speed_vector.reserve(size);
  max_torque_speed_vector.reserve(size);
  for (const auto& value : steer_torque_proto_.steer_torque_speed_max()) {
    speed_vector.push_back(value.speed());
    max_torque_speed_vector.push_back(value.max_torque_speed());
  }
  max_torque_speed_plf_ = std::make_unique<PiecewiseLinearFunction<double>>(
      speed_vector, max_torque_speed_vector);
}

void TorqueController::LoadSteerSpeedErrorLimitPLF() {
  if (steer_torque_proto_.steer_speed_error_max().empty()) {
    const std::vector<double> speed_vector = {0.0, 5.0, 10.0, 20.0,
                                              40.0};  // m/s
    const std::vector<double> steer_speed_error_vector = {5.0, 2.0, 1.0, 0.1,
                                                          0.1};
    max_steer_speed_error_plf_ =
        std::make_unique<PiecewiseLinearFunction<double>>(
            speed_vector, steer_speed_error_vector);
  } else {
    const auto size = steer_torque_proto_.steer_speed_error_max_size();
    std::vector<double> speed_vector, max_steer_speed_error_vector;
    speed_vector.reserve(size);
    max_steer_speed_error_vector.reserve(size);
    for (const auto& value : steer_torque_proto_.steer_speed_error_max()) {
      speed_vector.push_back(value.speed());
      max_steer_speed_error_vector.push_back(value.max_steer_speed_error());
    }
    max_steer_speed_error_plf_ =
        std::make_unique<PiecewiseLinearFunction<double>>(
            speed_vector, max_steer_speed_error_vector);
  }
}

void TorqueController::LoadSteerAngleErrorLimitPLF() {
  if (steer_torque_proto_.steer_angle_error_max().empty()) {
    const std::vector<double> speed_vector = {0.0, 5.0, 10.0, 20.0, 40};  // m/s
    const std::vector<double> steer_angle_error_vector = {0.5, 0.5, 0.25, 0.08,
                                                          0.06};
    max_steer_angle_error_plf_ =
        std::make_unique<PiecewiseLinearFunction<double>>(
            speed_vector, steer_angle_error_vector);
  } else {
    const auto size = steer_torque_proto_.steer_angle_error_max_size();
    std::vector<double> speed_vector, max_steer_angle_error_vector;
    speed_vector.reserve(size);
    max_steer_angle_error_vector.reserve(size);
    for (const auto& value : steer_torque_proto_.steer_angle_error_max()) {
      speed_vector.push_back(value.speed());
      max_steer_angle_error_vector.push_back(value.max_steer_angle_error());
    }
    max_steer_angle_error_plf_ =
        std::make_unique<PiecewiseLinearFunction<double>>(
            speed_vector, max_steer_angle_error_vector);
  }
}

void TorqueController::LoadSteerTorqueInterpolation2D() {
  if (steer_torque_proto_.steer_torque_table().empty()) return;
  Interpolation2D::DataType angle_xyz, speed_xyz;
  for (const auto& table : steer_torque_proto_.steer_torque_table()) {
    if (!table.steer_stiffness_offset().empty()) {
      for (const auto& angle_offset : table.steer_stiffness_offset()) {
        angle_xyz.push_back(std::make_tuple(
            table.speed(), angle_offset.steer_angle(), angle_offset.torque()));
      }
    }
    if (!table.steer_damping_offset().empty()) {
      for (const auto& speed_offset : table.steer_damping_offset()) {
        speed_xyz.push_back(std::make_tuple(
            table.speed(), speed_offset.steer_speed(), speed_offset.torque()));
      }
    }
  }
  if (!angle_xyz.empty()) {
    Interpolation2D angle_interpolation2d;
    angle_interpolation2d.Init(angle_xyz);
    angle_interpolation2d_opt_ = std::move(angle_interpolation2d);
  }
  if (!speed_xyz.empty()) {
    Interpolation2D speed_interpolation2d;
    speed_interpolation2d.Init(speed_xyz);
    speed_interpolation2d_opt_ = std::move(speed_interpolation2d);
  }
}

PIDConfig TorqueController::ComputeAnglePIDConfig(double speed) {
  PIDConfig pid_config;
  pid_config.Kp = angle_p_plf_->Evaluate(std::fabs(speed));
  pid_config.Ki = angle_i_plf_->Evaluate(std::fabs(speed));
  pid_config.Kd = angle_d_plf_->Evaluate(std::fabs(speed));
  pid_config.min_value = angle_min_plf_->Evaluate(std::fabs(speed));
  pid_config.max_value = angle_max_plf_->Evaluate(std::fabs(speed));
  pid_config.period = kControlInterval;
  pid_config.integral_limit =
      angle_max_integral_plf_->Evaluate(std::fabs(speed));
  return pid_config;
}

PIDConfig TorqueController::ComputeAngleSpeedPIDConfig(double speed) {
  PIDConfig pid_config;
  pid_config.Kp = speed_p_plf_->Evaluate(std::fabs(speed));
  pid_config.Ki = speed_i_plf_->Evaluate(std::fabs(speed));
  pid_config.Kd = speed_d_plf_->Evaluate(std::fabs(speed));
  pid_config.min_value = speed_min_plf_->Evaluate(std::fabs(speed));
  pid_config.max_value = speed_max_plf_->Evaluate(std::fabs(speed));
  pid_config.period = kControlInterval;
  pid_config.integral_limit =
      speed_max_integral_plf_->Evaluate(std::fabs(speed));
  return pid_config;
}

double TorqueController::ComputeSteerTorqueForward(double speed,
                                                   double steer_angle,
                                                   double steer_speed) {
  if (steer_torque_proto_.steer_torque_table().empty()) return 0.0;
  const auto table = steer_torque_proto_.steer_torque_table();
  double torque_angle_offset = 0.0;
  if (angle_interpolation2d_opt_.has_value()) {
    torque_angle_offset = angle_interpolation2d_opt_->Interpolate(
        std::make_pair(std::fabs(speed), std::fabs(steer_angle)));
  }
  double torque_speed_offset = 0.0;
  if (speed_interpolation2d_opt_.has_value()) {
    torque_speed_offset = speed_interpolation2d_opt_->Interpolate(
        std::make_pair(std::fabs(speed), std::fabs(steer_speed)));
  }
  return torque_angle_offset * std::copysign(1.0, steer_angle) +
         torque_speed_offset * std::copysign(1.0, steer_speed);
}

// Reset integral when vehicle is from stationary to running.
void TorqueController::ResetStaticTorqueIntegral(double speed) {
  constexpr double kStaticSpeedLimit = 0.1;  // m/s
  const bool is_vehicle_stop = std::fabs(speed) < kStaticSpeedLimit;
  if (is_vehicle_stop_last_ && !is_vehicle_stop) {
    pid_angle_controller_.ResetIntegral();
    pid_angle_spd_controller_.ResetIntegral();
  }
  is_vehicle_stop_last_ = is_vehicle_stop;
}

void TorqueController::Reset() {
  pid_angle_controller_.ResetIntegral();
  pid_angle_spd_controller_.ResetIntegral();
  is_vehicle_stop_last_ = false;
  torque_requested_last_ = 0.0;
}

}  // namespace qcraft::control
