#include "onboard/control/control_check/lon_wire_control_check.h"

#include <cmath>
#include <memory>

#include "onboard/control/control_defs.h"
#include "onboard/control/math/util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {

LonWireControlChecker::LonWireControlChecker() {
  // Config for check longitudinal wire control.
  constexpr double kAccCheckTime = 1.0;    // check time for acc cache.
  constexpr double kBrakeDelayTime = 0.4;  // brake delay time.

  num_acc_check_ = FloorToInt(kAccCheckTime * kControlFrequency) + 1;
  num_brake_delay_ = FloorToInt(kBrakeDelayTime * kControlFrequency) + 1;
  acc_fb_cache_.resize(num_acc_check_);
  acc_cmd_cache_.resize(num_acc_check_ + num_brake_delay_);
}

void LonWireControlChecker::Reset() {
  acc_fb_cache_.assign(num_acc_check_, 0.0);
  acc_cmd_cache_.assign(num_acc_check_ + num_brake_delay_, 0.0);
}

// Check throttle acc >= 3.0m/s2.
bool LonWireControlChecker::IsAccelerateTooHard(double acc_fb) {
  constexpr double kAccThreshold = 3.0;
  if (acc_fb >= kAccThreshold) return true;

  return false;
}

// Check AVP |speed| >= 3.0m/s, acc >= 1.5m/s2.
bool LonWireControlChecker::IsAVPSpeedTooHard(bool is_freespace, double acc_fb,
                                              double speed) {
  if (!is_freespace) return false;

  constexpr double kAVPAccThreshold = 1.5;
  constexpr double kAVPSpeedThreshold = 3.0;
  if (acc_fb >= kAVPAccThreshold) return true;
  if (std::fabs(speed) >= kAVPSpeedThreshold) return true;

  return false;
}

// Compute brake response rate.
double LonWireControlChecker::ComputeBrakeResponseRate(double acc_cmd,
                                                       double acc_fb,
                                                       double speed) {
  double brake_response_rate = 1.0;

  constexpr double kSpeedThreshold = 3.0;
  constexpr double kBrakeAccThreshold = -1;
  if (std::fabs(speed) > kSpeedThreshold && acc_cmd < kBrakeAccThreshold) {
    brake_response_rate = acc_fb / acc_cmd;
  }
  return brake_response_rate;
}

bool LonWireControlChecker::IsAbnormal(const VehicleStateProto& vehicle_state,
                                       const ControlCommand& control_cmd,
                                       WireControlCheckDebugProto* debug) {
  // Change gear, or not auto drive mode ,and reset cache data.
  if ((control_cmd.gear_location() != vehicle_state.gear()) ||
      !vehicle_state.is_auto_speed()) {
    Reset();
    debug->set_lon_abnormal(false);
    debug->set_brake_response_rate(0.0);
    return false;
  }
  const bool is_freespace = control_cmd.custom_command().low_speed_freespace();
  const double v = vehicle_state.linear_velocity();

  acc_cmd_cache_.push_back(control_cmd.acceleration());
  const boost::circular_buffer<double> acc_cmd_delay(
      acc_cmd_cache_.begin(), acc_cmd_cache_.begin() + num_acc_check_);
  const double acc_cmd_mean = ComputeMeanOfCircularBuffer(acc_cmd_delay);
  acc_fb_cache_.push_back(vehicle_state.linear_acceleration());
  const double acc_fb_mean = ComputeMeanOfCircularBuffer(acc_fb_cache_);

  const bool throttle_abnormal = IsAccelerateTooHard(acc_fb_mean);
  const bool avp_abnormal = IsAVPSpeedTooHard(is_freespace, acc_fb_mean, v);
  const double brake_response_rate =
      ComputeBrakeResponseRate(acc_cmd_mean, acc_fb_mean, v);

  debug->set_lon_abnormal(throttle_abnormal || avp_abnormal);
  debug->set_brake_response_rate(brake_response_rate);

  constexpr double kBrakeDecayRate = 0.7;  // 70%
  if (brake_response_rate < kBrakeDecayRate) {
    QEVENT_EVERY_N_SECONDS("zhenxing", "control_brake_decay_too_hard",
                           /*seconds=*/5.0, [&](QEvent* qevent) {
                             qevent->AddField("brake_response_rate",
                                              brake_response_rate);
                           });
  }
  return throttle_abnormal || avp_abnormal;
}

}  // namespace control
}  // namespace qcraft
