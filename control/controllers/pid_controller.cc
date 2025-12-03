#include "onboard/control/controllers/pid_controller.h"

#include <algorithm>
#include <cmath>

namespace qcraft {
namespace control {

PIDControl::PIDControl() { ResetIntegral(); }

void PIDControl::SetConfig(const PIDConfig& config) { config_ = config; }

void PIDControl::Compute(double error) {
  // Anti integral windup
  if (std::fabs(pid_out_.offset_I) >= config_.integral_limit) {
    if (pid_out_.offset_I * error < 0) {
      pid_out_.offset_I += config_.Ki * error * config_.period;
    }
  } else {
    pid_out_.offset_I += config_.Ki * error * config_.period;
  }

  // Compute derivative
  const double error_derivative = (error - error_last_) / config_.period;

  // pid control computer
  pid_out_.offset_P = config_.Kp * error;
  pid_out_.offset_D = config_.Kd * error_derivative;
  error_last_ = error;
}

void PIDControl::ResetIntegral() { pid_out_.offset_I = 0.0; }

PIDOut PIDControl::GetPIDOut() {
  pid_out_.result = pid_out_.offset_P + pid_out_.offset_I + pid_out_.offset_D;
  pid_out_.result =
      std::clamp(pid_out_.result, config_.min_value, config_.max_value);
  return pid_out_;
}

}  // namespace control
}  // namespace qcraft
