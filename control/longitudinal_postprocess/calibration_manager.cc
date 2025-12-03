#include "onboard/control/longitudinal_postprocess/calibration_manager.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>
#include <cmath>
#include <ostream>
#include <tuple>
#include <utility>

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"

namespace qcraft {
namespace control {

absl::Status CalibrationManager::Init(
    const VehicleDriveParamsProto& vehicle_drive_params) {
  vehicle_drive_params_ = vehicle_drive_params;
  if (vehicle_drive_params_.has_enable_calibration_v2()) {
    enable_calibration_v2_ = vehicle_drive_params_.enable_calibration_v2();
  }
  throttle_lowerbound_ = vehicle_drive_params_.throttle_deadzone();
  brake_lowerbound_ = vehicle_drive_params_.brake_deadzone();

  if (!enable_calibration_v2_) {
    if (!vehicle_drive_params_.has_calibration_table()) {
      const auto error_msg = "Not have version 1.0 parameters!";
      QLOG_EVERY_N_SEC(ERROR, 1.0) << error_msg;
      return absl::InvalidArgumentError(error_msg);
    }
    control_interpolation_2d_ = std::make_unique<Interpolation2D>();
    QLOG_EVERY_N_SEC(INFO, 1.0) << "Import Calibration Version 1.0 ...";
  } else {
    if (!vehicle_drive_params_.has_calibration_table_v2() ||
        !vehicle_drive_params_.calibration_table_v2().has_idle_v_a_plf()) {
      const auto error_msg = "Version 2.0 not parameters!";
      QLOG_EVERY_N_SEC(ERROR, 1.0) << error_msg;
      return absl::InvalidArgumentError(error_msg);
    }
  }
  LoadControlCalibration();
  return absl::OkStatus();
}

double CalibrationManager::UpdateCalibrationValue(
    const double speed, const double acceleration_target,
    const Chassis::GearPosition gear_position) {
  acceleration_idle_ = ComputeIdleAcc(speed, gear_position);
  if (!enable_calibration_v2_) {
    return control_interpolation_2d_->Interpolate(
        std::make_pair(speed, acceleration_target));
  } else {
    acceleration_pure_ = acceleration_target - acceleration_idle_;
    return UpdateCalibrationValueV2(acceleration_pure_, speed, gear_position);
  }
}

CalibrationCmd CalibrationManager::ComputeLongitudinalCmd(
    const double calibration_value) {
  CalibrationCmd calibration_cmd;
  if (calibration_value >= 0.0) {
    calibration_cmd.throttle_cmd =
        calibration_value > throttle_lowerbound_ ? calibration_value : 0.0;
    calibration_cmd.brake_cmd = 0.0;
  } else {
    calibration_cmd.throttle_cmd = 0.0;
    calibration_cmd.brake_cmd =
        -calibration_value > brake_lowerbound_ ? -calibration_value : 0.0;
  }
  return calibration_cmd;
}

double CalibrationManager::GetIdleAcceleration() { return acceleration_idle_; }

double CalibrationManager::GetPureAcceleration() { return acceleration_pure_; }

void CalibrationManager::LoadControlCalibration() {
  if (vehicle_drive_params_.calibration_table_v2().has_idle_v_a_plf()) {
    idle_v_a_plf_ = PiecewiseLinearFunctionFromProto(
        vehicle_drive_params_.calibration_table_v2().idle_v_a_plf());
  }

  if (!enable_calibration_v2_) {
    QCHECK(SetInterpolation2D(vehicle_drive_params_.calibration_table(),
                              control_interpolation_2d_))
        << "Faid to load control calibraiton table";
  } else {
    if (vehicle_drive_params_.calibration_table_v2().has_a_throttle_plf()) {
      a_throttle_plf_ = PiecewiseLinearFunctionFromProto(
          vehicle_drive_params_.calibration_table_v2().a_throttle_plf());
    }
    if (vehicle_drive_params_.calibration_table_v2().has_a_brake_plf()) {
      a_brake_plf_ = PiecewiseLinearFunctionFromProto(
          vehicle_drive_params_.calibration_table_v2().a_brake_plf());
    }
  }
}

double CalibrationManager::ComputeIdleAcc(double speed,
                                          Chassis::GearPosition gear_position) {
  if (!vehicle_drive_params_.calibration_table_v2().has_idle_v_a_plf()) {
    constexpr double kDefaultIdleAcc = -0.3;  // m/s^2.
    return kDefaultIdleAcc;
  }

  constexpr double kEpsilonSpeed = 0.01;
  return gear_position == Chassis::GEAR_REVERSE
             ? idle_v_a_plf_(std::min(speed, -kEpsilonSpeed))
             : idle_v_a_plf_(std::max(speed, kEpsilonSpeed));
}

double CalibrationManager::UpdateCalibrationValueV2(
    const double acceleration_pure, const double speed,
    Chassis::GearPosition gear_position) {
  constexpr double kEpsilon = 0.1;
  double a_gain_wrt_speed = 1.0;
  if (vehicle_drive_params_.calibration_table_v2().has_a_gain_wrt_speed_plf()) {
    const auto a_gain_wrt_speed_plf = PiecewiseLinearFunctionFromProto(
        vehicle_drive_params_.calibration_table_v2().a_gain_wrt_speed_plf());
    a_gain_wrt_speed = a_gain_wrt_speed_plf.Evaluate(std::fabs(speed));
  }

  double throttle_percetage = 0.0;
  if (vehicle_drive_params_.calibration_table_v2().has_a_throttle_plf()) {
    throttle_percetage = a_throttle_plf_(acceleration_pure * a_gain_wrt_speed);
  }
  double brake_percetage = 0.0;
  if (vehicle_drive_params_.calibration_table_v2().has_a_brake_plf()) {
    brake_percetage = -a_brake_plf_(acceleration_pure);
  }

  if (acceleration_pure >= 0.0) {
    if (gear_position == Chassis::GEAR_REVERSE && speed <= kEpsilon) {
      return brake_percetage;
    }
    return throttle_percetage;
  } else {
    if (gear_position == Chassis::GEAR_REVERSE && speed <= kEpsilon) {
      return throttle_percetage;
    }
    return brake_percetage;
  }
}

bool CalibrationManager::SetInterpolation2D(
    const ControlCalibrationTable& calibration_table,
    const std::unique_ptr<Interpolation2D>& interpolation_2d_ptr) {
  Interpolation2D::DataType xyz;
  for (const auto& calibration : calibration_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  return interpolation_2d_ptr->Init(xyz);
}

}  // namespace control
}  // namespace qcraft
