#include "onboard/control/longitudinal_postprocess/lon_postprocess.h"

#include <algorithm>
#include <cmath>
// IWYU pragma: no_include <ostream>

#include "glog/logging.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/controllers/controller_common.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/steering_protection.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"

namespace qcraft::control {

namespace {

double ComputeAccLimitWrtSteerRate(double steer_execute_ratio,
                                   double conf_max_acceleration) {
  if (std::isnan(steer_execute_ratio)) return conf_max_acceleration;
  QCHECK(InRange(steer_execute_ratio, 0.0, 2.0)) << steer_execute_ratio;

  return PiecewiseLinearFunction<double, double>(
             /*steer_execute_ratio*/ {0.1, 0.5, 0.95},
             /*acc_limitation*/ {conf_max_acceleration, 0.4, 0.1})
      .Evaluate(std::clamp(steer_execute_ratio, 0.0, 1.0));
}

}  // namespace

double LonPostProcess::ComputeLongitudinalJerk(bool is_full_stop,
                                               double acc_feedback,
                                               double acc_target,
                                               double delay) {
  constexpr double kMinLonDelay = 0.1;
  constexpr double kMinFullStopJerk = -0.1;
  if (delay < kMinLonDelay) return 0.0;
  const double jerk_raw = (acc_target - acc_feedback) / delay;

  if (control_config_->has_jerk_calculation_conf()) {
    const PiecewiseLinearFunction<double>& jerk_upper_limit_plf =
        PiecewiseLinearFunctionFromProto(
            control_config_->jerk_calculation_conf().jerk_upper_limit_plf());
    const PiecewiseLinearFunction<double>& jerk_lower_limit_plf =
        PiecewiseLinearFunctionFromProto(
            control_config_->jerk_calculation_conf().jerk_lower_limit_plf());

    double jerk_upper_limit = jerk_upper_limit_plf(acc_target);
    if (is_full_stop) {
      jerk_upper_limit = std::min(kMinFullStopJerk, jerk_upper_limit);
    }
    const double jerk_lower_limit = jerk_lower_limit_plf(acc_target);

    return std::clamp(jerk_raw, jerk_lower_limit, jerk_upper_limit);
  } else {
    if (is_full_stop) {
      return std::clamp(jerk_raw, kMinFullStopJerk,
                        FLAGS_longitudinal_acc_jerk_limit);
    } else {
      return std::clamp(jerk_raw, FLAGS_longitudinal_dec_jerk_limit,
                        FLAGS_longitudinal_acc_jerk_limit);
    }
  }
}

LonPostProcessInput::LonPostProcessInput(
    bool onboard_mode, bool low_speed_freespace, double kappa_rate,
    const ControlCommand& cmd, const VehicleStateProto& vehicle_state,
    const TrajectoryInterface& trajectory_interface)
    : is_onboard_mode(onboard_mode),
      is_auto_mode(vehicle_state.is_auto_speed()),
      gear_cmd(cmd.gear_location()),
      gear_fb(vehicle_state.gear()),
      low_speed_freespace(low_speed_freespace),
      acc_target(cmd.acceleration()),
      acc_feedback(vehicle_state.linear_acceleration()),
      acc_planner(cmd.debug().simple_mpc_debug().acceleration_reference()),
      speed_feedback(vehicle_state.linear_velocity()),
      speed_planner(cmd.debug().simple_mpc_debug().speed_reference()),
      pitch_pose(vehicle_state.pitch()),
      steer_wheel_angle(vehicle_state.front_wheel_steering_angle()),
      kappa_cmd(cmd.curvature()),
      kappa_rate_cmd(kappa_rate),
      steer_speed_target(cmd.steer_speed_target()),
      trajectory_accumulate_s(trajectory_interface.accumulate_s()) {}

LonPostProcess::LonPostProcess(
    const ControllerConf* control_config,
    const VehicleDriveParamsProto* vehicle_drive_params) {
  control_config_ = control_config;
  vehicle_params_config_ = vehicle_drive_params;

  // Mean filter for sin(slope).
  constexpr int kSinSlopeFilterWindowSize = 120;
  sin_slope_mean_filter_ = MeanFilter(kSinSlopeFilterWindowSize);

  if (control_config_->enable_speed_mode_manager()) {
    const bool is_available_idle =
        vehicle_params_config_->calibration_table_v2().has_idle_v_a_plf();
    speed_mode_acc_closed_loop_ = std::make_unique<ClosedLoopAcc>(
        *control_config_, is_available_idle,
        vehicle_params_config_->throttle_deadzone(),
        vehicle_params_config_->brake_deadzone());
  }

  calibration_manager_ = std::make_unique<CalibrationManager>();
  QCHECK_OK(calibration_manager_->Init(*vehicle_drive_params));
}

void LonPostProcess::Process(const LonPostProcessInput& input,
                             ControlCommand* control_cmd,
                             ControllerDebugProto* control_debug) {
  SCOPED_QTRACE("LonPostProcess");

  if (!input.is_auto_mode) {
    previous_acc_smoothed_ = 0.0;
    previous_acc_calibration_ = 0.0;
  }

  double acc_smoothed = input.acc_target;
  /*
   * 1. Add acc upper limitation wrt steering speed.
   */
  const double steer_execute_ratio =
      std::abs(input.steer_speed_target / kSteeringSpeedLimitRad);
  const double acc_limit_wrt_steer_execute_ratio = ComputeAccLimitWrtSteerRate(
      steer_execute_ratio, control_config_->max_acceleration_cmd());
  acc_smoothed = std::min(acc_smoothed, acc_limit_wrt_steer_execute_ratio);

  if (steer_execute_ratio > 0.7) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "steer_fast", /*seconds=*/3.0, [&](QEvent* qevent) {
          qevent->AddField("steer_execute_ratio", steer_execute_ratio)
              .AddField("acc_target", input.acc_target)
              .AddField("acc_limit_wrt_steer_execute_ratio",
                        acc_limit_wrt_steer_execute_ratio);
        });
  }

  /*
   * 2. Slope to acc compensation.
   */
  const double acc_offset = GetAccCompensationBySlope(input.pitch_pose);

  /*
   * 3. Smooth acc for hard brake and limit acc according to control config.
   */
  acc_smoothed = std::clamp(SmoothHardBrake(input.acc_target),
                            control_config_->max_deceleration_cmd(),
                            control_config_->max_acceleration_cmd());
  previous_acc_smoothed_ = acc_smoothed;

  /*
   * 4. Judge is full stop and generate acc for brakding down, it don't depend
   * on mpc output.
   */
  const bool is_full_stop =
      IsFullStop(input.trajectory_accumulate_s, input.speed_feedback,
                 input.low_speed_freespace);
  const auto acc_full_stop_opt = GenerateAccForFullStop(
      *control_config_, input.gear_fb, acc_offset, acc_smoothed, is_full_stop);
  /*
   * 5. Take full stop and road slope into overall consideration, calculate the
   * acc to lookup calibraiton table.
   */
  double acc_calibration = CalcAccForCalibration(
      acc_full_stop_opt, input.gear_fb, acc_smoothed, acc_offset);

  /*
   * 6. Override acc for calibation with speed mode.
   */
  const bool is_gear_shifting = input.gear_cmd != input.gear_fb;
  if (speed_mode_acc_closed_loop_ != nullptr && !is_gear_shifting) {
    acc_calibration = ComputeAccCalibrationBySpeedMode(
        input.is_auto_mode, is_full_stop, input.gear_fb, input.speed_feedback,
        acc_calibration, acc_offset, acc_smoothed);
  }
  previous_acc_calibration_ = acc_calibration;

  /*
   * 7. Convert acc to brake or throttle percentage, -100~100%.
   */
  const double executor_percentage = ConvertAcc2ExecutorPercentage(
      input.is_auto_mode, input.gear_fb, input.speed_feedback,
      input.acc_feedback, input.steer_wheel_angle, acc_calibration,
      is_full_stop, is_gear_shifting);
  const auto throttle_or_brake =
      calibration_manager_->ComputeLongitudinalCmd(executor_percentage);
  /*
   * 8. Update states and record relatated data in proto.
   */
  // TODO(yangyu): is_standstill should not be updated here or it shoud defined
  // in other message rather than speed mode debug message.
  const bool is_standstill = IsStandstill(input.speed_feedback);
  const double jerk = ComputeLongitudinalJerk(
      is_full_stop, input.acc_feedback, input.acc_target,
      control_config_->closed_loop_acc_conf().throttle_delay_time());

  // TODO(yangyu): move it to speed mode manager.
  if (speed_mode_acc_closed_loop_ != nullptr) {
    control_cmd->set_speed_mode(
        speed_mode_acc_closed_loop_->GetCurrSpeedMode());
  }
  // TODO(shijun): wrap it in a function, it need to be merged with speed mode
  // related acc modification.
  if (input.is_onboard_mode) {
    double acc_cmd_output = acc_smoothed;
    double acc_calibration_output = acc_calibration;
    constexpr double kEpsilonAcc = 0.01;

    // Use speed mode to limit acc.
    if (speed_mode_acc_closed_loop_ != nullptr) {
      const auto curr_speed_mode =
          speed_mode_acc_closed_loop_->GetCurrSpeedMode();

      const bool is_negative_acc_output =
          (input.gear_fb == Chassis::GEAR_REVERSE &&
           curr_speed_mode == qcraft::SpeedMode::ACC_MODE) ||
          (input.gear_fb == Chassis::GEAR_DRIVE &&
           curr_speed_mode == qcraft::SpeedMode::DEC_MODE);

      if (curr_speed_mode == qcraft::SpeedMode::ACC_MODE) {
        if (input.gear_fb == Chassis::GEAR_DRIVE) {
          acc_cmd_output = std::max(acc_cmd_output, kEpsilonAcc);
        } else if (input.gear_fb == Chassis::GEAR_REVERSE) {
          acc_cmd_output = std::min(acc_cmd_output, -kEpsilonAcc);
        }
      }
      acc_calibration_output =
          is_negative_acc_output
              ? std::min(acc_calibration_output, -kEpsilonAcc)
              : std::max(acc_calibration_output, kEpsilonAcc);
    }

    const auto throttle_interface = control_config_->throttle_interface();
    switch (throttle_interface) {
      case ThrottleInterface::CLOSEDLOOP_ACC:
        control_cmd->set_acceleration(acc_cmd_output);
        break;
      case ThrottleInterface::OPENEDLOOP_ACC:
        control_cmd->set_acceleration(acc_calibration_output);
        break;
      default:
        control_cmd->set_acceleration(acc_smoothed);
    }

    const auto brake_interface = control_config_->brake_interface();
    switch (brake_interface) {
      case BrakeInterface::CLOSEDLOOP_DEC:
        control_cmd->set_acceleration(acc_cmd_output);
        break;
      case BrakeInterface::OPENEDLOOP_DEC:
        control_cmd->set_acceleration(acc_calibration_output);
        break;
      case BrakeInterface::CLOSEDLOOP_DEC_WITH_PARKING_COMPENSATION:
        control_cmd->set_acceleration(
            acc_full_stop_opt.value_or(acc_cmd_output));
        break;
      default:
        control_cmd->set_acceleration(acc_smoothed);
    }
  } else {
    control_cmd->set_acceleration(acc_smoothed);
  }

  control_cmd->set_acceleration_offset(acc_offset);
  control_cmd->set_acceleration_calibration(acc_calibration);

  control_cmd->set_throttle(throttle_or_brake.throttle_cmd);
  control_cmd->set_brake(throttle_or_brake.brake_cmd);
  control_cmd->set_longitudinal_jerk(jerk);

  // Update control command.
  auto mpc_debug = control_cmd->mutable_debug()->mutable_simple_mpc_debug();
  mpc_debug->set_is_full_stop(is_full_stop);
  mpc_debug->set_acceleration_cmd(input.acc_target);
  mpc_debug->set_acceleration_cmd_closeloop(input.acc_target);
  mpc_debug->set_calibration_value(executor_percentage);

  // Update control debug.
  control_debug->mutable_speed_mode_debug_proto()->CopyFrom(
      control_debug_.speed_mode_debug_proto());
  auto* lon_post_process_debug =
      control_debug->mutable_lon_post_process_debug_proto();
  lon_post_process_debug->set_acceleration_offset(acc_offset);
  lon_post_process_debug->set_acc_limit_wrt_steer_execute_ratio(
      acc_limit_wrt_steer_execute_ratio);
  control_debug->mutable_speed_mode_debug_proto()->set_standstill(
      is_standstill);

  const double acceleration_idle = calibration_manager_->GetIdleAcceleration();
  const double acceleration_pure = calibration_manager_->GetPureAcceleration();
  control_debug->mutable_calibration_debug_proto()->set_acceleration_idle(
      acceleration_idle);
  control_debug->mutable_calibration_debug_proto()->set_acceleration_pure(
      acceleration_pure);
}

double LonPostProcess::SmoothHardBrake(double acc_target) {
  return std::clamp(acc_target,
                    previous_acc_smoothed_ +
                        FLAGS_longitudinal_dec_jerk_limit * kControlInterval,
                    previous_acc_smoothed_ +
                        FLAGS_longitudinal_acc_jerk_limit * kControlInterval);
}

double LonPostProcess::GetAccCompensationBySlope(double pitch) {
  // Estimate acceleration offset based on pose pitch.
  const double sin_slope = -std::sin(pitch);
  const double sin_slope_smooth = sin_slope_mean_filter_.Update(
      std::clamp(sin_slope, -kSinSlopeLimit, kSinSlopeLimit));
  return sin_slope_smooth * kGravitationalAcceleration;
}

std::optional<double> LonPostProcess::GenerateAccForFullStop(
    const ControllerConf& controller_conf, Chassis::GearPosition gear_fb,
    double acc_offset, double acc_smoothed, bool is_full_stop) const {
  if (!is_full_stop) {
    return std::nullopt;
  }

  constexpr double kJerkLimit = 0.4;  // m/s^3.
  const double acc_delta = kJerkLimit * kControlInterval;
  double acc_standstill =
      controller_conf.full_stop_condition().lockdown_acceleration();

  // Smooth brake and take gear and slope acceleration into consideration.
  double acc_full_stop = 0.0;
  // The scale factor which is more than 1.0 is to prevent slip when full stop
  // on slope.
  constexpr double kScaleFactor = 1.5;
  if (gear_fb == Chassis::GEAR_REVERSE) {
    acc_standstill = -acc_standstill - kScaleFactor * std::fabs(acc_offset);
    acc_full_stop = std::min(
        std::max(previous_acc_calibration_, 0.0) + acc_delta, acc_standstill);
    acc_full_stop = std::max(acc_full_stop, acc_smoothed);
  } else {
    acc_standstill -= kScaleFactor * std::fabs(acc_offset);
    acc_full_stop = std::max(
        std::min(previous_acc_calibration_, 0.0) - acc_delta, acc_standstill);
    acc_full_stop = std::min(acc_full_stop, acc_smoothed);
  }
  return acc_full_stop;
}

// Compute acceleration calibration.
double LonPostProcess::CalcAccForCalibration(
    std::optional<double> acc_full_stop_opt, Chassis::GearPosition gear_fb,
    double acc_smoothed, double acc_offset) const {
  double acc_calibration = 0.0;

  if (acc_full_stop_opt.has_value()) {
    if (gear_fb == Chassis::GEAR_REVERSE) {
      acc_calibration = std::max(acc_smoothed, *acc_full_stop_opt);
    } else {
      acc_calibration = std::min(acc_smoothed, *acc_full_stop_opt);
    }
  } else {
    acc_calibration = acc_smoothed + acc_offset;
  }

  CHECK(std::fabs(acc_calibration) < 10.0)
      << "Abs of acc_calibration value ," << acc_calibration << ", is too big.";

  return acc_calibration;
}

double LonPostProcess::ComputeAccCalibrationBySpeedMode(
    bool is_auto_mode, bool is_full_stop, Chassis::GearPosition gear_fb,
    double speed_feedback, double acc_calibration, double acc_offset,
    double acc_smoothed) {
  double acc_idle =
      calibration_manager_->ComputeIdleAcc(speed_feedback, gear_fb);

  // Use opposite input for speed mode when reversing.
  if (gear_fb == Chassis::GEAR_REVERSE) {
    acc_calibration = -acc_calibration;
    acc_smoothed = -acc_smoothed;
    speed_feedback = -speed_feedback;
    acc_idle = -acc_idle;
  }

  // Update acc by speed mode.
  const double acceleration_calibration_speedmode =
      speed_mode_acc_closed_loop_->UpdateAccCommand(
          is_auto_mode, is_full_stop, acc_calibration, acc_smoothed, acc_offset,
          acc_idle, &control_debug_);

  return gear_fb == Chassis::GEAR_REVERSE ? -acceleration_calibration_speedmode
                                          : acceleration_calibration_speedmode;
}

double LonPostProcess::ConvertAcc2ExecutorPercentage(
    bool is_auto_mode, Chassis::GearPosition gear_fb, double speed_feedback,
    double acc_feedback, double steer_wheel_angle, double acc_calibration,
    bool is_full_stop, bool is_gear_shifting) {
  double executor_percentage = calibration_manager_->UpdateCalibrationValue(
      speed_feedback, acc_calibration, gear_fb);

  if (control_config_->closed_loop_acc_conf().enable_closed_loop_acc() &&
      speed_mode_acc_closed_loop_ != nullptr && !is_gear_shifting) {
    const auto steer_rad_abs = std::abs(std::atan(steer_wheel_angle));
    if (gear_fb == Chassis::GEAR_REVERSE) {
      acc_feedback = -acc_feedback;
    }
    const double executor_percentage_closedloop =
        speed_mode_acc_closed_loop_->UpdateCalibrationCmd(
            is_auto_mode, is_full_stop, executor_percentage, acc_feedback,
            steer_rad_abs, &control_debug_);
    executor_percentage = executor_percentage_closedloop;
  }

  if (is_gear_shifting) {
    executor_percentage = std::min(executor_percentage, 0.0);
  }

  return executor_percentage;
}

}  // namespace qcraft::control
