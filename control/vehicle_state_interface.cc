#include "onboard/control/vehicle_state_interface.h"

#include <algorithm>
#include <cmath>

#include "absl/status/status.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/localization/visual/proto/data_type.pb.h"
#include "onboard/localization/visual/visual_localization_constants.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::control {
namespace {

constexpr int kChassisLossThres = 5;  // times

// Localize drift related limitation.
constexpr double kLocalizeDriftRateLimit = 0.2;      // m/s.
constexpr double kYawDriftLimit = d2r(/*deg*/ 2.0);  // rad.

double CalculateYawBiasWrtSlip(double v_x, double v_y) {
  if (std::abs(v_x) < /*speed threshold*/ 1.0) return 0.0;

  constexpr double kLateralSpeedLimitation = 0.5;  // m/s;
  return std::atan2(
      std::clamp(v_y, -kLateralSpeedLimitation, kLateralSpeedLimitation), v_x);
}

double CalculateLocalizeYDriftRate(
    const LocalizationViewerDebugProto& localize_debug,
    MeanFilter* localize_y_offset_rate_estimator,
    ControllerDebugProto* controller_debug_proto) {
  if (!localize_debug.has_v2_message()) return 0.0;
  if (!localize_debug.v2_message().has_debug_plot_data()) return 0.0;
  if (!localize_debug.v2_message().debug_plot_data().has_offset_xy_vcs()) {
    return 0.0;
  }

  const double localize_y_drift_rate =
      (localize_debug.v2_message().debug_plot_data().offset_xy_vcs().y()) *
      (1.0e3 / static_cast<double>(
                   /*int, ms*/ localization::visual::kMaxBevIntervalTime));
  const double localize_y_drift_rate_filtered =
      localize_y_offset_rate_estimator->Update(localize_y_drift_rate);

  const double localize_y_drift_rate_filteded_capped =
      std::clamp(localize_y_drift_rate_filtered, -kLocalizeDriftRateLimit,
                 kLocalizeDriftRateLimit);

  auto* drift_debug = controller_debug_proto->mutable_vehicle_state_debug()
                          ->mutable_localize_drift_debug();
  drift_debug->set_localize_y_drift_rate(localize_y_drift_rate);
  drift_debug->set_localize_y_drift_rate_filtered(
      localize_y_drift_rate_filtered);
  drift_debug->set_localize_y_drift_rate_filteded_capped(
      localize_y_drift_rate_filteded_capped);

  return localize_y_drift_rate_filteded_capped;
}

double CalcYawRefDriftWrtLocalize(double v_x, double localize_y_drift_rate) {
  if (v_x < /*speed threshold, m/s*/ 1.0) return 0.0;

  return std::clamp(std::atan(localize_y_drift_rate / v_x), -kYawDriftLimit,
                    kYawDriftLimit);
}

absl::Status ConstructVehicleStateFromChassis(
    const Chassis& chassis, VehicleStateProto* vehicle_state) {
  QCHECK_NOTNULL(vehicle_state);

  if (chassis.has_gear_location()) {
    vehicle_state->set_gear(chassis.gear_location());
  } else {
    return absl::FailedPreconditionError("Chassis has no gear location.");
  }

  if (chassis.has_driving_mode()) {
    vehicle_state->set_driving_mode(chassis.driving_mode());
  } else {
    return absl::FailedPreconditionError("Chassis has no driving mode.");
  }

  // TODO(zhichao): add a signal check.
  vehicle_state->set_chassis_steering_speed(d2r(chassis.steering_speed()));
  vehicle_state->set_chassis_steering_torque(chassis.steering_torque_nm());

  return absl::OkStatus();
}

void WriteChassisSteering(double steering_pct,
                          const SteeringConverter& steering_converter,
                          VehicleStateProto* vehicle_state) {
  vehicle_state->set_chassis_steering_percentage(steering_pct);
  const double front_wheel_angle =
      steering_converter.SteerPctToFrontWheelAngle(steering_pct);
  vehicle_state->set_front_wheel_steering_angle(front_wheel_angle);
}

absl::Status ConstructVehicleStateFromChassisSteering(
    const Chassis& chassis, const SteeringConverter& steering_converter,
    VehicleStateProto* vehicle_state) {
  if (!chassis.has_steering_percentage() ||
      std::isnan(chassis.steering_percentage())) {
    return absl::FailedPreconditionError("Chassis has no steering percentage.");
  }

  WriteChassisSteering(chassis.steering_percentage(), steering_converter,
                       vehicle_state);
  return absl::OkStatus();
}

absl::Status ConstructVehicleStateFromPose(
    double lateral_speed_filted, double yaw_bias, const PoseProto& pose,
    VehicleStateProto* vehicle_state,
    ControllerDebugProto* controller_debug_proto) {
  QCHECK_NOTNULL(vehicle_state);

  vehicle_state->mutable_pose()->CopyFrom(pose);

  if (pose.has_pos_smooth()) {
    vehicle_state->set_x(pose.pos_smooth().x());
    vehicle_state->set_y(pose.pos_smooth().y());
    vehicle_state->set_z(pose.pos_smooth().z());
  } else {
    return absl::FailedPreconditionError("Pose has no pos smooth.");
  }

  if (pose.has_ar_body()) {
    vehicle_state->set_angular_velocity(pose.ar_body().z());
  } else {
    return absl::FailedPreconditionError("Pose has no angular velocity.");
  }

  if (pose.has_accel_body()) {
    vehicle_state->set_linear_acceleration(pose.accel_body().x());
  } else {
    return absl::FailedPreconditionError("Pose has no linear acceleration.");
  }

  if (pose.has_vel_body()) {
    const double lon_speed = pose.vel_body().x();
    const double lat_speed = pose.vel_body().y();
    vehicle_state->set_linear_velocity(lon_speed);
    vehicle_state->set_lateral_velocity(lon_speed > 2.0 ? lat_speed : 0.0);
  } else {
    return absl::FailedPreconditionError("Pose has no linear velocity.");
  }

  const double yaw_adjustment =
      CalculateYawBiasWrtSlip(pose.vel_body().x(), lateral_speed_filted);
  const double yaw_after_bias = NormalizeAngle(pose.yaw() - yaw_bias);

  vehicle_state->set_yaw(yaw_after_bias);
  vehicle_state->set_pitch(pose.pitch());
  vehicle_state->set_roll(pose.roll());
  vehicle_state->set_moving_direction(
      NormalizeAngle(yaw_after_bias + yaw_adjustment));

  constexpr double kEpsilon = 1e-6;
  if (std::abs(vehicle_state->linear_velocity()) < kEpsilon) {
    vehicle_state->set_kappa(0.0);
  } else {
    vehicle_state->set_kappa(vehicle_state->angular_velocity() /
                             vehicle_state->linear_velocity());
  }

  vehicle_state->set_timestamp(pose.timestamp());

  auto* slip_debug = controller_debug_proto->mutable_vehicle_state_debug()
                         ->mutable_slip_debug();
  slip_debug->set_lateral_speed_raw(pose.vel_body().y());
  slip_debug->set_lateral_speed_filted(lateral_speed_filted);

  return absl::OkStatus();
}

void ConstructVehicleStateFromAutonomyState(
    const AutonomyStateProto_State& autonomy_state,
    VehicleStateProto* vehicle_state) {
  QCHECK_NOTNULL(vehicle_state);

  vehicle_state->set_is_auto_mode(
      autonomy_state == AutonomyStateProto::AUTO_DRIVE ||
      autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP ||
      autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE);
  vehicle_state->set_is_auto_steer(
      autonomy_state == AutonomyStateProto::AUTO_DRIVE ||
      autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP ||
      autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE ||
      autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY);

  vehicle_state->set_is_auto_speed(
      autonomy_state == AutonomyStateProto::AUTO_DRIVE ||
      autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP ||
      autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE ||
      autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY);

  vehicle_state->set_autonomy_state(autonomy_state);
}

}  // namespace

VehicleStateInterface::VehicleStateInterface() {
  lateral_velocity_estimator_ =
      std::make_optional<ExponentialSmoothing>(/*smoothing factor*/ 0.5);
  localize_y_offset_rate_estimator_ =
      MeanFilter(/*window_size*/ 25);  // 0.5 seconds window.
}

absl::Status VehicleStateInterface::Update(
    double yaw_bias, const AutonomyStateProto_State& autonomy_state,
    const PoseProto& pose, const Chassis& chassis,
    const std::optional<LocalizationViewerDebugProto>& localize_debug,
    const SteeringConverter& steering_converter,
    ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("VehicleStateInterface::Update");
  vehicle_state_proto_.Clear();
  RETURN_IF_ERROR(
      ConstructVehicleStateFromChassis(chassis, &vehicle_state_proto_));

  auto steering_cmd_status = ConstructVehicleStateFromChassisSteering(
      chassis, steering_converter, &vehicle_state_proto_);
  if (!steering_cmd_status.ok()) {
    if (std::isnan(last_valid_chassis_steering_pct_) ||
        chassis_msg_loss_count_++ >= kChassisLossThres) {
      return steering_cmd_status;
    }
    WriteChassisSteering(last_valid_chassis_steering_pct_, steering_converter,
                         &vehicle_state_proto_);
  } else {
    chassis_msg_loss_count_ = std::max(--chassis_msg_loss_count_, 0);
    last_valid_chassis_steering_pct_ = chassis.steering_percentage();
  }

  RETURN_IF_ERROR(ConstructVehicleStateFromPose(
      lateral_velocity_estimator_->Evaluate(pose.vel_body().y()), yaw_bias,
      pose, &vehicle_state_proto_, controller_debug_proto));

  if (localize_debug.has_value()) {
    const double localize_y_drift_rate = CalculateLocalizeYDriftRate(
        *localize_debug, &localize_y_offset_rate_estimator_,
        controller_debug_proto);
    vehicle_state_proto_.set_yaw_diff_wrt_localize_drift(
        CalcYawRefDriftWrtLocalize(vehicle_state_proto_.linear_velocity(),
                                   localize_y_drift_rate));
  }

  ConstructVehicleStateFromAutonomyState(autonomy_state, &vehicle_state_proto_);

  return absl::OkStatus();
}

VehicleStateProto VehicleStateInterface::Result() const {
  return vehicle_state_proto_;
}

}  // namespace qcraft::control
