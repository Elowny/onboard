#include "onboard/control/steering_converter.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft::control {

SteeringConverter::SteeringConverter(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    VehicleInfoProto::VehicleInterface vehicle_type) {
  wheel_base_ = vehicle_geometry_params.wheel_base();
  max_steer_angle_ = vehicle_drive_params.max_steer_angle();
  variable_steer_ratio_ = std::make_unique<VariableSteerRatio>(
      vehicle_drive_params,
      std::make_optional<VehicleInfoProto::VehicleInterface>(vehicle_type));
}

SteeringConverter::SteeringConverter(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  wheel_base_ = vehicle_geometry_params.wheel_base();
  max_steer_angle_ = vehicle_drive_params.max_steer_angle();
  variable_steer_ratio_ =
      std::make_unique<VariableSteerRatio>(vehicle_drive_params, std::nullopt);
}

SteeringConverter::SteeringConverter(double wheel_base, double steer_ratio,
                                     double max_steer_angle) {
  wheel_base_ = wheel_base;
  max_steer_angle_ = max_steer_angle;
  VehicleDriveParamsProto vehicle_drive_params;
  vehicle_drive_params.set_max_steer_angle(max_steer_angle);
  vehicle_drive_params.set_steer_ratio(steer_ratio);
  variable_steer_ratio_ =
      std::make_unique<VariableSteerRatio>(vehicle_drive_params, std::nullopt);
}

double SteeringConverter::SteerAngleToFrontWheelAngle(
    double steer_angle) const {
  return variable_steer_ratio_->SteerAngleToFrontWheelAngle(steer_angle);
}

double SteeringConverter::FrontWheelAngleToSteerAngle(
    double front_wheel_angle) const {
  return variable_steer_ratio_->FrontWheelAngleToSteerAngle(front_wheel_angle);
}

double SteeringConverter::KappaToFrontWheelAngle(double kappa) const {
  QCHECK_GT(wheel_base_, 0.0);
  return std::atan(kappa * wheel_base_);
}

double SteeringConverter::FrontWheelAngleToKappa(
    double front_wheel_angle) const {
  QCHECK_GT(wheel_base_, 0.0);
  return std::copysign(std::tan(front_wheel_angle) / wheel_base_,
                       front_wheel_angle);
}

double SteeringConverter::KappaToSteerAngle(double kappa) const {
  const double front_wheel_angle = KappaToFrontWheelAngle(kappa);
  return FrontWheelAngleToSteerAngle(front_wheel_angle);
}

double SteeringConverter::KappaToSteerPct(double kappa) const {
  QCHECK_GT(max_steer_angle_, 0.0);
  const double steer_angle = KappaToSteerAngle(kappa);
  return steer_angle / max_steer_angle_ * 100.0;
}

double SteeringConverter::SteerPctToSteerAngle(double steer_pct) const {
  QCHECK_GT(max_steer_angle_, 0.0);
  return steer_pct * 0.01 * max_steer_angle_;
}

double SteeringConverter::SteerPctToFrontWheelAngle(double steer_pct) const {
  const double steer_angle = SteerPctToSteerAngle(steer_pct);
  return SteerAngleToFrontWheelAngle(steer_angle);
}

double SteeringConverter::SteerPctToKappa(double steer_pct) const {
  const double front_wheel_angle = SteerPctToFrontWheelAngle(steer_pct);
  return FrontWheelAngleToKappa(front_wheel_angle);
}

double SteeringConverter::SteerAngleToKappa(double steer_angle) const {
  const double front_wheel_angle = SteerAngleToFrontWheelAngle(steer_angle);
  return FrontWheelAngleToKappa(front_wheel_angle);
}

double SteeringConverter::SteerAngleToSteerPct(double steer_angle) const {
  return steer_angle / max_steer_angle_ * 100.0;
}

double SteeringConverter::KappaRateToSteerRate(double kappa_rate,
                                               double kappa) const {
  // tan(delta) = l * kappa;
  // d(delta)/dt = l * cos^2(delta) d(kappa) /dt;
  QCHECK_GT(wheel_base_, 0.0);
  const double front_wheel_angle = KappaToFrontWheelAngle(kappa);
  const double steer_angle = FrontWheelAngleToSteerAngle(front_wheel_angle);
  const double steer_ratio = variable_steer_ratio_->steer_ratio(steer_angle);
  const double front_wheel_omega =
      wheel_base_ * Sqr(std::cos(front_wheel_angle)) * kappa_rate;
  return front_wheel_omega * steer_ratio;
}

double SteeringConverter::KappaRateToFrontWheelOmega(double kappa_rate,
                                                     double kappa) const {
  const double steer_angle = KappaToSteerAngle(kappa);
  const double steer_ratio = variable_steer_ratio_->steer_ratio(steer_angle);
  return KappaRateToSteerRate(kappa_rate, kappa) / steer_ratio;
}

double SteeringConverter::SteerRateToFrontWheelOmega(double steer_rate,
                                                     double steer_angle) const {
  const double steer_ratio = variable_steer_ratio_->steer_ratio(steer_angle);
  return steer_rate / steer_ratio;
}

double SteeringConverter::FrontWheelOmegaToKappaRate(
    double front_wheel_omega, double front_wheel_angle) const {
  QCHECK_GT(wheel_base_, 0.0);
  // d(kappa) /dt = d(delta)/dt / l / cos^2(delta);
  return front_wheel_omega / wheel_base_ / Sqr(std::cos(front_wheel_angle));
}

double SteeringConverter::FrontWheelOmegaToSteerRate(
    double front_wheel_omega) const {
  // Use approximate steer ratio.
  const double steer_ratio =
      variable_steer_ratio_->steer_ratio(/*steer_angle*/ 0.0);
  return front_wheel_omega * steer_ratio;
}

double SteeringConverter::SteerRateToKappaRate(double steer_rate,
                                               double steer_angle) const {
  const double kappa = SteerAngleToKappa(steer_angle);
  const double front_wheel_angle = KappaToFrontWheelAngle(kappa);
  const double steer_ratio = variable_steer_ratio_->steer_ratio(steer_angle);
  const double front_wheel_omega = steer_rate / steer_ratio;
  return FrontWheelOmegaToKappaRate(front_wheel_omega, front_wheel_angle);
}

double SteeringConverter::ClampKappaByMaxSteerAngle(double kappa) const {
  if (max_steer_angle_ > 0.0) {
    const double max_kappa = SteerAngleToKappa(max_steer_angle_);
    return std::clamp(kappa, -max_kappa, max_kappa);
  }
  return kappa;
}

double SteeringConverter::ClampSteerAngleByMaxSteerAngle(
    double steer_angle) const {
  return max_steer_angle_ > 0.0
             ? std::clamp(steer_angle, -max_steer_angle_, max_steer_angle_)
             : steer_angle;
}

}  // namespace qcraft::control
