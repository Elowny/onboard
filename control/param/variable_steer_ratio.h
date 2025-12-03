#ifndef ONBOARD_CONTROL_PARAM_VARIABLE_STEER_RATIO_H_
#define ONBOARD_CONTROL_PARAM_VARIABLE_STEER_RATIO_H_

#include <optional>

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

// Design doc: https://qcraft.feishu.cn/docx/XVOudJWaToU1tLx5VQDcSA6OnTP
class VariableSteerRatio {
 public:
  VariableSteerRatio(
      const VehicleDriveParamsProto& vehicle_drive_params,
      std::optional<VehicleInfoProto::VehicleInterface> vehicle_type);

  double FrontWheelAngleToSteerAngle(double front_wheel_angle) const;

  double SteerAngleToFrontWheelAngle(double steer_angle) const;

  double steer_ratio(double steer_angle) const;

 private:
  PiecewiseLinearFunction<double, double> steer_angle_to_steer_ratio_plf_;
  PiecewiseLinearFunction<double, double> front_wheel_angle_to_steer_angle_plf_;
  PiecewiseLinearFunction<double, double> steer_angle_to_front_wheel_angle_plf_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_PARAM_VARIABLE_STEER_RATIO_H_
