#ifndef ONBOARD_CONTROL_STEERING_PROTECTION_H_
#define ONBOARD_CONTROL_STEERING_PROTECTION_H_

#include <cmath>

#include "absl/status/status.h"

#include "onboard/control/control_cache_manager.h"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/lite/check.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {

constexpr double kLateralkJerkLimit = 3.5;             // m/s^3
constexpr double kSteeringSpeedLimitRad = 2.0 * M_PI;  // rad/s.
constexpr double kRelaxFactorThreshold = 0.95;

void WrapSteerConstraintChartData(
    double prev_kappa_cmd, const ControllerConf& controller_conf,
    const ControlCommand& control_command,
    const ControllerDebugProto& controller_debug_proto,
    vis::vantage::ChartsDataProto* chart_data);

// Limit maximum steering kappa rate and steering kappa for the safety.
class SteeringProtection {
 public:
  SteeringProtection(const VehicleDriveParamsProto& vehicle_drive_params,
                     const SteeringConverter* steering_converter,
                     const ControllerConf* control_conf)
      : control_conf_(QCHECK_NOTNULL(control_conf)),
        steering_converter_(QCHECK_NOTNULL(steering_converter)),
        max_steer_angle_(vehicle_drive_params.max_steer_angle()) {}

  SteeringProtectionResult CalcKappaAndKappaRateLimit(
      double previous_kappa_cmd, const VehicleStateProto& vehicle_state) const;

  absl::Status SteerResultStatus(
      const VehicleStateProto& vehicle_state,
      const ControlCacheManager& control_cache_manager,
      const ControllerDebugProto& controller_debug_proto,
      const SteeringProtectionResult& steering_protection_result);

 private:
  const ControllerConf* control_conf_;
  const SteeringConverter* steering_converter_;
  double max_steer_angle_ = 0.0;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_STEERING_PROTECTION_H_
