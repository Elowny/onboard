#include "onboard/control/control_parking/stationary_steer_control.h"

#include <algorithm>
#include <cmath>
#include <ostream>

#include "glog/logging.h"

#include "onboard/control/control_defs.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {
// Return curvature_cmd(unit: rad).
double CalcStationarySteeringCmd(double previous_control_kappa, double av_kappa,
                                 double ref_kappa,
                                 const SteeringConverter& steering_converter) {
  const double kappa_error = ref_kappa - av_kappa;
  constexpr double kStationarySteeringPrecision = 0.02;  // rad.
  if (std::fabs(kappa_error) < kStationarySteeringPrecision) {
    return ref_kappa;
  }

  constexpr double kSteeringSpeed = d2r(300.0);  // rad/s.
  const double kappa_rate =
      steering_converter.SteerRateToKappaRate(kSteeringSpeed, av_kappa);
  const double control_curvature =
      previous_control_kappa +
      std::copysign(kappa_rate * kControlInterval, kappa_error);
  VLOG(1) << "**********************" << '\n'
          << "previous_control_kappa: " << previous_control_kappa << '\n'
          << "av_kappa: " << av_kappa << '\n'
          << "ref_kappa: " << ref_kappa << '\n'
          << "control curvature: " << control_curvature << '\n'
          << "kappa error: " << kappa_error << '\n'
          << "kappa_rate: " << kappa_rate << '\n'
          << "**********************";
  return control_curvature;
}

double StationarySteerControl::ComputestationarySteerCmd(
    StationarySteerControlInput input) {
  // When av is in auto_mode and stops standstill and receives
  // stationary trajectory with enable_stationary_steering, enter stationary
  // steering state and override curvature.
  if (input.is_stationary_steer) {
    VLOG(2) << "Enter stationary steering state.";

    const double av_kappa = input.steering_converter->FrontWheelAngleToKappa(
        input.vehicle_state->front_wheel_steering_angle());
    if (first_hit_stationary_steering_) {
      previous_stationary_steering_cmd_ = input.kappa_cmd;
      first_hit_stationary_steering_ = false;
    }
    const double control_curvature_unlimited = CalcStationarySteeringCmd(
        previous_stationary_steering_cmd_, av_kappa, input.kappa_trajectory,
        *input.steering_converter);
    const double kappa_limit =
        std::min(input.steering_protection_result->kappa_limit_wrt_geometry(),
                 input.steering_protection_result->kappa_limit_wrt_lat_a());
    const double control_curvature_limited =
        std::clamp(control_curvature_unlimited, -kappa_limit, kappa_limit);
    previous_stationary_steering_cmd_ = control_curvature_limited;
    return control_curvature_limited;
  } else {
    if (!first_hit_stationary_steering_) {
      first_hit_stationary_steering_ = true;
    }
    return input.kappa_cmd;
  }
}

}  // namespace control
}  // namespace qcraft
