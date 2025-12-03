#include "onboard/control/control_check/lat_wire_control_check.h"

#include <cmath>

#include "boost/move/utility_core.hpp"

#include "onboard/control/control_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {

LatWireControlChecker::LatWireControlChecker(double steer_delay,
                                             double wheel_base,
                                             double steer_ratio,
                                             double max_steer_angle) {
  // Config for check lateral wire control.
  num_steer_delay_ = FloorToInt(steer_delay * kControlFrequency) + 1;
  steer_cmd_cache_.resize(num_steer_delay_);

  // Steer and kappa converter
  steer_converter_ = std::make_unique<SteeringConverter>(
      wheel_base, steer_ratio, max_steer_angle);
  Reset();
}

void LatWireControlChecker::Reset() { steer_cmd_cache_.clear(); }

// Check steer wheel angle, between control and chassis
bool LatWireControlChecker::IsSteerAngleErrorTooLarge(double steer_error,
                                                      double speed,
                                                      bool is_onboard_mode) {
  constexpr double kCheckSpeedLimit = 1.0;
  if (std::fabs(speed) < kCheckSpeedLimit) return false;

  const PiecewiseLinearFunction<double, double>
      v_steer_error_plf(/* v(m/s) = */
                        {0.0, 5.0, 10.0, 20.0, 30.0},
                        /* error(rad) =  */ {6.0, 4.0, 1.0, 0.3, 0.1});
  const double threshold = v_steer_error_plf.Evaluate(std::fabs(speed));
  if (is_onboard_mode && (std::fabs(steer_error) > threshold)) {
    return true;
  }
  return false;
}

bool LatWireControlChecker::IsAbnormal(const VehicleStateProto& vehicle_state,
                                       const ControlCommand& control_cmd,
                                       bool is_onboard_mode,
                                       WireControlCheckDebugProto* debug) {
  // Change gear, or not auto drive mode ,and reset cache data.
  if ((control_cmd.gear_location() != vehicle_state.gear()) ||
      !vehicle_state.is_auto_steer()) {
    Reset();
    debug->set_lat_abnormal(false);
    debug->set_steer_error(0.0);
    return false;
  }
  steer_cmd_cache_.push_back(control_cmd.steering_target());
  const double steer_cmd =
      steer_converter_->SteerPctToSteerAngle(steer_cmd_cache_.front());
  const double steer_fb = steer_converter_->SteerPctToSteerAngle(
      vehicle_state.chassis_steering_percentage());

  const double v = vehicle_state.linear_velocity();
  const double steer_error = steer_cmd - steer_fb;
  const bool steer_angle_abnormal =
      IsSteerAngleErrorTooLarge(steer_error, v, is_onboard_mode);

  debug->set_lat_abnormal(steer_angle_abnormal);
  debug->set_steer_error(steer_error);

  return steer_angle_abnormal;
}
}  // namespace control
}  // namespace qcraft
