#ifndef ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_CALIBRATION_MANAGE_H_
#define ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_CALIBRATION_MANAGE_H_

#include <memory>

#include "absl/status/status.h"

#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/math/interpolation_2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

struct CalibrationCmd {
  double throttle_cmd = 0.0;  // throttle control command,0~100%
  double brake_cmd = 0.0;     // brake control command,0~100%
};

// Manage version 1.0 and 2.0 of longitudinal control calibration table.
class CalibrationManager {
 public:
  absl::Status Init(const VehicleDriveParamsProto& vehicle_drive_params);

  // Compute calibrationvalue (+/-) by calibration table.
  double UpdateCalibrationValue(double speed, double acceleration_target,
                                Chassis::GearPosition gear_position);

  // Compute throttle and brake command according to update calibration_value
  // from speed model function.
  CalibrationCmd ComputeLongitudinalCmd(double calibration_value);

  // Compute idle acceleration by idling and sliding curve, for speed mode
  // manager.
  double ComputeIdleAcc(double speed, Chassis::GearPosition gear_position);

  double GetIdleAcceleration();

  double GetPureAcceleration();

 private:
  // Load control calibration table version 1.0 and 2.0
  void LoadControlCalibration();

  // Update control calibration table value of version 2.0
  double UpdateCalibrationValueV2(double acceleration_pure, double speed,
                                  Chassis::GearPosition gear_position);

  // For calibration version 1.0, initialize interpolation2D function.
  bool SetInterpolation2D(
      const ControlCalibrationTable& calibration_table,
      const std::unique_ptr<Interpolation2D>& interpolation_2d_ptr);

  double throttle_lowerbound_ = 0.0;
  double brake_lowerbound_ = 0.0;
  VehicleDriveParamsProto vehicle_drive_params_;
  std::unique_ptr<Interpolation2D> control_interpolation_2d_;

  bool enable_calibration_v2_ = false;  // enable calibration version 2.0
  PiecewiseLinearFunction<double> idle_v_a_plf_;
  PiecewiseLinearFunction<double> a_throttle_plf_;
  PiecewiseLinearFunction<double> a_brake_plf_;
  double acceleration_idle_ = 0.0;  // only idle or drag acceleration
  double acceleration_pure_ = 0.0;  // except drag, drive or brake acceleration
};

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_CALIBRATION_MANAGE_H_
