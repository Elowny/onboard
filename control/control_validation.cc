#include "onboard/control/control_validation.h"

#include <algorithm>
#include <string>

#include "absl/strings/str_cat.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/steering_converter.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/autonomy_state.pb.h"

namespace qcraft {
namespace control {

namespace {

constexpr double kMaxAccelerationCmd = 2.0;                     // m/s^2.
constexpr double kMaxDecelerationCmd = -6.0;                    // m/s^2.
constexpr double kMPCPredictedDeviationKickoutThreshold = 7.0;  // m.
constexpr double kMPCPredictedDeviationQEventThreshold = 1.0;   // m.
constexpr double kEpsilon = 1e-3;

bool ValidateLongitudinalControlOutput(
    const ControllerConf& controller_conf, const ControlCommand& control_cmd,
    ControllerDebugProto* controller_debug_proto) {
  auto* debug = controller_debug_proto->mutable_validation_result_proto();

  const double accel_mpc = control_cmd.acceleration();
  const double accel_offset = control_cmd.acceleration_offset();
  const double accel_calibration = control_cmd.acceleration_calibration();
  const double throttle = control_cmd.throttle();
  const double brake = control_cmd.brake();

  const double vehicle_max_accel = controller_conf.has_max_acceleration_cmd()
                                       ? controller_conf.max_acceleration_cmd()
                                       : kMaxAccelerationCmd;
  const double vehicle_max_decel = controller_conf.has_max_deceleration_cmd()
                                       ? controller_conf.max_deceleration_cmd()
                                       : kMaxDecelerationCmd;
  const double accel_offset_limit = kSinSlopeLimit * kGravitationalAcceleration;

  const bool accel_mpc_valid = InRange(accel_mpc, vehicle_max_decel - kEpsilon,
                                       vehicle_max_accel + kEpsilon);
  const bool accel_offset_valid =
      InRange(accel_offset, -accel_offset_limit - kEpsilon,
              accel_offset_limit + kEpsilon);
  const bool accel_calibration_valid = InRange(
      accel_calibration, vehicle_max_decel - accel_offset_limit - kEpsilon,
      vehicle_max_accel + accel_offset_limit + kEpsilon);

  if (!accel_mpc_valid || !accel_offset_valid || !accel_calibration_valid) {
    debug->set_error_code(
        ControlValidationResultProto::ACCELERATION_OVER_LIMIT);
    const std::string error_message = absl::StrCat(
        "accel_mpc, ", accel_mpc, ", accel_offset, ", accel_offset,
        ", or accel_calibration, ", accel_calibration, ", is out of range.");
    debug->set_error_message(error_message);
    return false;
  }

  constexpr double kCmdUpperLimit = 100.0;
  constexpr double kCmdLowerLimit = 0.0;

  const bool throttle_brake_cmd_valid =
      std::min(throttle, brake) == 0.0 &&
      InRange(std::max(throttle, brake), kCmdLowerLimit - kEpsilon,
              kCmdUpperLimit + kEpsilon);
  if (!throttle_brake_cmd_valid) {
    debug->set_error_code(
        ControlValidationResultProto::CALIBRATION_RESULT_OVER_LIMIT);
    const std::string error_message = absl::StrCat(
        "throttle, ", throttle, " or brake, ", brake, ", is out of range.");
    debug->set_error_message(error_message);
    return false;
  }

  return true;
}

bool ValidateLateralControlOutput(
    const SteeringConverter& steering_converter,
    const ControlCommand& control_cmd,
    ControllerDebugProto* controller_debug_proto) {
  auto* debug = controller_debug_proto->mutable_validation_result_proto();

  const double curvature_cmd = control_cmd.curvature();
  const double steering_cmd = control_cmd.steering_target();

  const double front_wheel_angle =
      steering_converter.SteerPctToFrontWheelAngle(steering_cmd);
  const double curvature_from_steering_cmd =
      steering_converter.FrontWheelAngleToKappa(front_wheel_angle);

  const auto& steering_protection_result =
      control_cmd.debug().simple_mpc_debug().steering_protection_result();
  const double curvature_limit =
      std::min(steering_protection_result.kappa_limit_wrt_geometry(),
               steering_protection_result.kappa_limit_wrt_lat_a());

  const bool curvature_cmd_valid = InRange(
      curvature_cmd, -curvature_limit - kEpsilon, curvature_limit + kEpsilon);
  const bool steering_cmd_valid =
      InRange(curvature_from_steering_cmd, -curvature_limit - kEpsilon,
              curvature_limit + kEpsilon);

  if (!curvature_cmd_valid) {
    debug->set_error_code(ControlValidationResultProto::CURVATURE_OVER_LIMIT);
    const std::string error_message =
        absl::StrCat("curvature_cmd, ", curvature_cmd, ", is out of range, ",
                     curvature_limit, ".");
    debug->set_error_message(error_message);
    return false;
  }

  if (!steering_cmd_valid) {
    debug->set_error_code(
        ControlValidationResultProto::STEERING_COMMAND_OVER_LIMIT);
    const std::string error_message =
        absl::StrCat("steering_cmd, ", steering_cmd,
                     ", is out of range, while the curvature_limit is ",
                     curvature_limit, ".");
    debug->set_error_message(error_message);
    return false;
  }
  return true;
}

bool ValidateMPCPredictionDeviation(
    const VehicleStateProto& vehicle_state,
    ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("ValidateMPCPredictionDeviation");
  // TODO(yangyu): consider pole_placement controller, now it has no predicted
  // points, skip it.
  if (!controller_debug_proto->has_mpc_debug_proto() ||
      !controller_debug_proto->mutable_mpc_debug_proto()
           ->mpc_predicted_traj_point_size()) {
    return true;
  }

  const auto& mpc_debug = controller_debug_proto->mpc_debug_proto();
  QCHECK_EQ(mpc_debug.mpc_reference_traj_point_size(),
            mpc_debug.mpc_predicted_traj_point_size());
  for (int i = 0; i < mpc_debug.mpc_reference_traj_point_size(); ++i) {
    const Vec2d reference_point(mpc_debug.mpc_reference_traj_point(i));
    const Vec2d predicted_point(mpc_debug.mpc_predicted_traj_point(i));
    const double distance = reference_point.DistanceTo(predicted_point);

    if (distance > kMPCPredictedDeviationQEventThreshold) {
      QEVENT_EVERY_N_SECONDS("zhichao", "mpc_predicted_deviation",
                             /*seconds=*/5.0, [&](QEvent* qevent) {
                               qevent->AddField("distance", distance)
                                   .AddField("speed",
                                             vehicle_state.linear_velocity());
                             });
      QLOG_EVERY_N_SEC(ERROR, 1.0) << "MPC prediction deviation.";
    }
    auto* debug = controller_debug_proto->mutable_validation_result_proto();
    if (distance > kMPCPredictedDeviationKickoutThreshold) {
      debug->set_error_code(
          ControlValidationResultProto::MPC_PREDICTION_ABNORMAL);
      const std::string error_message =
          absl::StrCat("MPC prediction deviation at i: ", i,
                       ", deviation distance: ", distance);
      debug->set_error_message(error_message);
      return false;
    }
  }

  return true;
}

}  // namespace

bool ValidateControlOutput(const VehicleStateProto& vehicle_state,
                           const SteeringConverter& steering_converter,
                           const ControllerConf& controller_conf,
                           const ControlCommand& control_cmd,
                           ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("ValidateControlOutput");
  switch (vehicle_state.autonomy_state()) {
    case AutonomyStateProto::READY_TO_AUTO_DRIVE:
    case AutonomyStateProto::AUTO_DRIVE:
    case AutonomyStateProto::EMERGENCY_TO_STOP:
      if (!ValidateLongitudinalControlOutput(controller_conf, control_cmd,
                                             controller_debug_proto) ||
          !ValidateLateralControlOutput(steering_converter, control_cmd,
                                        controller_debug_proto) ||
          !ValidateMPCPredictionDeviation(vehicle_state,
                                          controller_debug_proto)) {
        return false;
      }
      return true;
    case AutonomyStateProto::AUTO_STEER_ONLY:
      if (!ValidateLateralControlOutput(steering_converter, control_cmd,
                                        controller_debug_proto) ||
          !ValidateMPCPredictionDeviation(vehicle_state,
                                          controller_debug_proto)) {
        return false;
      }
      return true;
    case AutonomyStateProto::AUTO_SPEED_ONLY:
      if (!ValidateLongitudinalControlOutput(controller_conf, control_cmd,
                                             controller_debug_proto)) {
        return false;
      }
      return true;
    default:
      return true;
  }
}

}  // namespace control
}  // namespace qcraft
