#include "onboard/control/closed_loop_acc/speed_mode_manager.h"

#include <algorithm>
#include <cmath>

#include "onboard/control/control_defs.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {

double ClosedLoopAcc::UpdateAccCommand(
    bool is_auto_mode, bool is_full_stop, double acc_calib, double acc_cmd,
    double acc_offset, double acc_idle,
    ControllerDebugProto* controller_debug_proto) {
  QCHECK_NOTNULL(controller_debug_proto);

  acc_calib_ = acc_calib;
  acc_offset_ = acc_offset;
  acc_cmd_ = acc_cmd;
  auto* speed_mode_debug_proto =
      controller_debug_proto->mutable_speed_mode_debug_proto();
  speed_mode_debug_proto->set_input_acc(acc_calib_);
  if (!is_auto_mode) {
    ResetSpeedMode();
    return acc_calib_;
  }

  CalculteSpeedModeAccBound(acc_idle);

  if (is_full_stop) {
    speed_mode_ = SpeedMode::DEC_MODE;
    acc_counter_ = 0;
    dec_counter_ = 0;
    is_first_frame_ = true;
    speed_mode_debug_proto->set_curr_speed_mode(speed_mode_);
    return acc_calib_;
  }

  if (is_first_frame_) {
    if (acc_calib_ > 0.0) {
      speed_mode_ = SpeedMode::ACC_MODE;
    } else {
      speed_mode_ = SpeedMode::DEC_MODE;
    }
    is_first_frame_ = false;
  }

  if (acc_calib_ > dec_upperbound_) {
    speed_mode_ = SpeedMode::ACC_MODE;
    acc_counter_ = 0;
    dec_counter_ = 0;
  } else if (acc_calib_ < acc_lowerbound_) {
    speed_mode_ = SpeedMode::DEC_MODE;
    acc_counter_ = 0;
    dec_counter_ = 0;
  } else {
    UpdateSpeedMode();
    LimitAccBySpeedMode();
  }
  speed_mode_debug_proto->set_curr_speed_mode(speed_mode_);
  speed_mode_debug_proto->set_acc2dec_counter(dec_counter_);
  speed_mode_debug_proto->set_dec2acc_counter(acc_counter_);
  speed_mode_debug_proto->set_output_acc(acc_calib_);
  speed_mode_debug_proto->set_acc_bound(acc_bound_);
  speed_mode_debug_proto->set_hysteresis_upper_limit(dec_upperbound_);
  speed_mode_debug_proto->set_hysteresis_lower_limit(acc_lowerbound_);

  return acc_calib_;
}

void ClosedLoopAcc::UpdateSpeedMode() {
  QCHECK_LE(acc_counter_, acc_counter_threshold_);
  QCHECK_LE(dec_counter_, dec_counter_threshold_);

  if ((acc_calib_ > acc_bound_) && (speed_mode_ == SpeedMode::DEC_MODE)) {
    if (acc_counter_ == acc_counter_threshold_) {
      speed_mode_ = SpeedMode::ACC_MODE;
      acc_counter_ = 0;
      dec_counter_ = 0;
    } else {
      acc_counter_++;
    }
  } else if ((acc_calib_ < acc_bound_) &&
             (speed_mode_ == SpeedMode::ACC_MODE)) {
    if (dec_counter_ == dec_counter_threshold_) {
      speed_mode_ = SpeedMode::DEC_MODE;
      acc_counter_ = 0;
      dec_counter_ = 0;
    } else {
      dec_counter_++;
    }
  } else {
    acc_counter_ = std::max(acc_counter_ - 1, 0);
    dec_counter_ = std::max(dec_counter_ - 1, 0);
  }
}

void ClosedLoopAcc::LimitAccBySpeedMode() {
  switch (speed_mode_) {
    case SpeedMode::ACC_MODE:
      acc_calib_ = std::max(acc_calib_, acc_lowerbound_);
      break;
    case SpeedMode::DEC_MODE:
      acc_calib_ = std::min(acc_calib_, dec_upperbound_);
      break;
    default:
      break;
  }
}

SpeedMode ClosedLoopAcc::GetCurrSpeedMode() { return speed_mode_; }

ClosedLoopAcc::ClosedLoopAcc(const ControllerConf& control_conf,
                             bool is_available_idle, double throttle_lowerbound,
                             double brake_lowerbound) {
  const auto& closed_loop_acc_conf = control_conf.closed_loop_acc_conf();
  is_available_idle_ = is_available_idle;
  speed_mode_ = SpeedMode::ACC_MODE;
  acc_counter_ = 0;
  dec_counter_ = 0;
  acc_counter_threshold_ =
      FloorToInt(control_conf.acc2dec_timer() * kControlFrequency);
  dec_counter_threshold_ =
      FloorToInt(control_conf.dec2acc_timer() * kControlFrequency);
  is_first_frame_ = true;
  hysteresis_zone_ = control_conf.hysteresis_zone();

  acc_deque_size_ =
      FloorToInt(closed_loop_acc_conf.acc_deque_time() * kControlFrequency);
  throttle_delay_cycle_ = FloorToInt(
      closed_loop_acc_conf.throttle_delay_time() * kControlFrequency);
  brake_delay_cycle_ =
      FloorToInt(closed_loop_acc_conf.brake_delay_time() * kControlFrequency);
  acc_cmd_past_.resize(acc_deque_size_, 0.0);
  throttle_deadzone_ = throttle_lowerbound;
  throttle_windup_ = closed_loop_acc_conf.throttle_windup();
  kp_acc_ = closed_loop_acc_conf.kp_acc();
  ki_acc_ = closed_loop_acc_conf.ki_acc();
  kd_acc_ = closed_loop_acc_conf.kd_acc();
  max_throttle_ = closed_loop_acc_conf.max_throttle();
  min_delta_throttle_ = closed_loop_acc_conf.min_delta_throttle();
  max_delta_throttle_ = closed_loop_acc_conf.max_delta_throttle();
  steer2throttle_ratio_ = closed_loop_acc_conf.steer2throttle_ratio();
  brake_deadzone_ = brake_lowerbound;
  brake_windup_ = closed_loop_acc_conf.brake_windup();
  kp_dec_ = closed_loop_acc_conf.kp_dec();
  ki_dec_ = closed_loop_acc_conf.ki_dec();
  kd_dec_ = closed_loop_acc_conf.kd_dec();
  max_brake_ = closed_loop_acc_conf.max_brake();
  min_delta_brake_ = closed_loop_acc_conf.min_delta_brake();
  max_delta_brake_ = closed_loop_acc_conf.max_delta_brake();
  keep_brake_decrease_threshold_ =
      closed_loop_acc_conf.keep_brake_decrease_threshold();
}

void ClosedLoopAcc::ResetSpeedMode() {
  speed_mode_ = SpeedMode::ACC_MODE;
  acc_counter_ = 0;
  dec_counter_ = 0;
  is_first_frame_ = true;
}

void ClosedLoopAcc::CalculteSpeedModeAccBound(double acc_idle) {
  if (is_available_idle_) {
    acc_bound_ = acc_idle;
  }
  constexpr double kMaxPolynomialAccBound = 1.0;
  constexpr double kMinPolynomialAccBound = -1.5;
  acc_bound_ = std::max(kMinPolynomialAccBound, acc_bound_);
  acc_bound_ = std::min(kMaxPolynomialAccBound,
                        acc_bound_);  // To do : Remove after zhenxing's commit
  acc_lowerbound_ = acc_bound_ - hysteresis_zone_;
  dec_upperbound_ = acc_bound_ + hysteresis_zone_;
}

double ClosedLoopAcc::UpdateCalibrationCmd(
    bool is_auto_mode, bool is_full_stop, double calib_value,
    double acc_feedback, double steer_rad_abs,
    ControllerDebugProto* controller_debug_proto) {
  steer_rad_abs_ = steer_rad_abs;
  calib_value_ = calib_value;
  acc_feedback_ = acc_feedback;

  if (is_auto_mode) {
    acc_cmd_past_.push_back(acc_cmd_);
    acc_cmd_past_.pop_front();
  } else {
    acc_cmd_past_.push_back(0.0);
    acc_cmd_past_.pop_front();
    ResetControlStatus();
    return calib_value_;
  }
  if (speed_mode_ != speed_mode_last_) {
    acc_cmd_ = acc_feedback_;
    ResetControlStatus();
  }
  double throttle_last_degbug = 0.0;
  double brake_last_degbug = 0.0;
  auto* speed_mode_debug_proto =
      controller_debug_proto->mutable_speed_mode_debug_proto();
  if (speed_mode_ == SpeedMode::ACC_MODE) {
    UpdateThrottle();
    throttle_last_degbug = throttle_last_;
    brake_last_degbug = -brake_deadzone_;
    throttle_last_ = value_;
    brake_last_ = -brake_deadzone_;
  } else if (is_full_stop) {
    throttle_last_ = throttle_deadzone_;
    brake_last_ = calib_value_;
    delta_value_ = 0.0;
    delta_value_p_ = 0.0;
    delta_value_i_ = 0.0;
    delta_value_d_ = 0.0;
    brake_integral_ = 0.0;
    value_ = calib_value_;
  } else {
    UpdateBrake();
    throttle_last_degbug = throttle_deadzone_;
    brake_last_degbug = brake_last_;
    brake_last_ = value_;
    throttle_last_ = throttle_deadzone_;
  }
  speed_mode_debug_proto->set_input_calib_value(calib_value);
  speed_mode_debug_proto->set_is_full_stop(is_full_stop);
  speed_mode_debug_proto->set_output_value(value_);
  speed_mode_debug_proto->set_diff_acc(diff_acc_);
  speed_mode_debug_proto->set_diff_acc_last(diff_acc_last_);
  speed_mode_debug_proto->set_throttle_last(throttle_last_degbug);
  speed_mode_debug_proto->set_throttle_integral(throttle_integral_);
  speed_mode_debug_proto->set_diff_dec(diff_dec_);
  speed_mode_debug_proto->set_diff_dec_last(diff_dec_last_);
  speed_mode_debug_proto->set_brake_last(brake_last_degbug);
  speed_mode_debug_proto->set_brake_integral(brake_integral_);
  speed_mode_debug_proto->set_delta_value(delta_value_);
  speed_mode_debug_proto->set_delta_value_p(delta_value_p_);
  speed_mode_debug_proto->set_delta_value_i(delta_value_i_);
  speed_mode_debug_proto->set_delta_value_d(delta_value_d_);
  speed_mode_debug_proto->set_calib_value_raw(calib_value_raw_);
  speed_mode_debug_proto->set_acc_feedback(acc_feedback_);
  speed_mode_debug_proto->set_acc_cmd_past(expt_acc_);

  diff_acc_last_ = diff_acc_;
  diff_dec_last_ = diff_dec_;
  speed_mode_last_ = speed_mode_;
  return value_;
}

void ClosedLoopAcc::ResetControlStatus() {
  throttle_last_ = throttle_deadzone_;
  throttle_integral_ = 0.0;
  diff_acc_last_ = 0.0;

  brake_last_ = -brake_deadzone_;
  brake_integral_ = 0.0;
  diff_dec_last_ = 0.0;
}

void ClosedLoopAcc::UpdateThrottle() {
  QCHECK_LT(throttle_delay_cycle_, acc_cmd_past_.size());
  QCHECK_GE(throttle_delay_cycle_, 0);
  expt_acc_ =
      std::max(acc_lowerbound_ - acc_offset_,
               acc_cmd_past_[acc_cmd_past_.size() - throttle_delay_cycle_ - 1]);
  diff_acc_ = expt_acc_ - acc_feedback_;
  throttle_integral_ = throttle_integral_ + diff_acc_;
  throttle_integral_ =
      std::clamp(throttle_integral_, -throttle_windup_, throttle_windup_);
  calib_value_ =
      std::max(calib_value_, throttle_deadzone_) +
      (std::max(calib_value_, throttle_deadzone_) - throttle_deadzone_) *
          steer_rad_abs_ *
          steer2throttle_ratio_;  // increase throttle when steering
  delta_value_p_ = kp_acc_ * diff_acc_;
  delta_value_i_ = ki_acc_ * throttle_integral_ * kControlInterval;
  delta_value_d_ = kd_acc_ * (diff_acc_ - diff_acc_last_) * kControlFrequency;
  delta_value_ = kp_acc_ * diff_acc_ +
                 ki_acc_ * throttle_integral_ * kControlInterval +
                 kd_acc_ * (diff_acc_ - diff_acc_last_) * kControlFrequency;
  calib_value_raw_ = calib_value_ + delta_value_;
  throttle_last_ = std::max(throttle_last_, throttle_deadzone_);
  value_ = std::clamp(calib_value_ + delta_value_,
                      throttle_last_ + min_delta_throttle_,
                      throttle_last_ + max_delta_throttle_);
  value_ = std::clamp(value_, throttle_deadzone_, max_throttle_);
}

void ClosedLoopAcc::UpdateBrake() {
  QCHECK_LT(brake_delay_cycle_, acc_cmd_past_.size());
  QCHECK_GE(brake_delay_cycle_, 0);
  expt_acc_ =
      std::min(dec_upperbound_ - acc_offset_,
               acc_cmd_past_[acc_cmd_past_.size() - brake_delay_cycle_ - 1]);
  diff_dec_ = expt_acc_ - acc_feedback_;
  brake_integral_ = brake_integral_ + diff_dec_;

  brake_integral_ = std::clamp(brake_integral_, -brake_windup_, brake_windup_);
  calib_value_ = std::min(calib_value_, -brake_deadzone_);
  delta_value_p_ = kp_dec_ * diff_dec_;
  delta_value_i_ = ki_dec_ * brake_integral_ * kControlInterval;
  delta_value_d_ = kd_dec_ * (diff_dec_last_ - diff_dec_) * kControlFrequency;
  delta_value_ = kp_dec_ * diff_dec_ +
                 ki_dec_ * brake_integral_ * kControlInterval +
                 kd_dec_ * (diff_dec_last_ - diff_dec_) * kControlFrequency;
  calib_value_raw_ = calib_value_ + delta_value_;
  brake_last_ = std::min(brake_last_, -brake_deadzone_);
  if (acc_cmd_ > 0.0) {
    value_ = calib_value_raw_;  // Allowed release brake when acc.
  } else {
    value_ =
        std::clamp(calib_value_ + delta_value_, brake_last_ + min_delta_brake_,
                   brake_last_ + max_delta_brake_);
  }
  value_ = std::clamp(value_, max_brake_, -brake_deadzone_);
  if ((abs(value_ - brake_last_) < keep_brake_decrease_threshold_) &&
      (value_ - brake_last_) > 0.0) {
    value_ = brake_last_;  // keep brake value when value decrease little
  }
}

double ClosedLoopAcc::GetLongitudinalDelay(double brake_delay,
                                           double throttle_delay) {
  QCHECK_GE(brake_delay, 0.01);
  QCHECK_GE(throttle_delay, 0.01);
  if (speed_mode_ == SpeedMode::DEC_MODE) {
    return brake_delay;
  } else {
    // When speed mode is disable, use throttle delay.
    return throttle_delay;
  }
}

}  // namespace control
}  // namespace qcraft
