#ifndef ONBOARD_CONTROL_STEERING_CONVERTER_H_
#define ONBOARD_CONTROL_STEERING_CONVERTER_H_

#include <memory>

#include "onboard/control/param/variable_steer_ratio.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {

class SteeringConverter {
 public:
  // Constructor, which may use properties of variable steer ratio.
  SteeringConverter(const VehicleGeometryParamsProto& vehicle_geometry_params,
                    const VehicleDriveParamsProto& vehicle_drive_params,
                    VehicleInfoProto::VehicleInterface vehicle_type);

  // Constructor which certainly don't use properties of variable steer ratio.
  SteeringConverter(const VehicleGeometryParamsProto& vehicle_geometry_params,
                    const VehicleDriveParamsProto& vehicle_drive_params);

  // Constructor which certainly don't use properties of variable steer ratio.
  SteeringConverter(double wheel_base, double steer_ratio,
                    double max_steer_angle);

  // Angle or kappa converter.
  double KappaToFrontWheelAngle(double kappa) const;

  double KappaToSteerAngle(double kappa) const;

  double KappaToSteerPct(double kappa) const;

  double SteerPctToSteerAngle(double steer_pct) const;

  double SteerPctToFrontWheelAngle(double steer_pct) const;

  double SteerPctToKappa(double steer_pct) const;

  double FrontWheelAngleToKappa(double front_wheel_angle) const;

  double SteerAngleToKappa(double steer_angle) const;

  double SteerAngleToFrontWheelAngle(double steer_angle) const;

  double SteerAngleToSteerPct(double steer_angle) const;

  double FrontWheelAngleToSteerAngle(double front_wheel_angle) const;

  // Angular rate or kappa rate converter.
  double KappaRateToSteerRate(double kappa_rate, double kappa) const;

  double FrontWheelOmegaToKappaRate(double front_wheel_omega,
                                    double front_wheel_angle) const;

  double FrontWheelOmegaToSteerRate(double front_wheel_omega) const;

  double SteerRateToKappaRate(double steer_rate, double steer_angle) const;

  double KappaRateToFrontWheelOmega(double kappa_rate, double kappa) const;

  double SteerRateToFrontWheelOmega(double steer_rate,
                                    double steer_angle) const;

  // Clamp kappa or steer angle by vehicle's max_steer_angle.
  double ClampKappaByMaxSteerAngle(double kappa) const;

  double ClampSteerAngleByMaxSteerAngle(double steer_angle) const;

 private:
  double wheel_base_;
  double max_steer_angle_;
  std::unique_ptr<VariableSteerRatio> variable_steer_ratio_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_STEERING_CONVERTER_H_
