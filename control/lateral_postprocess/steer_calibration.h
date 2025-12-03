#ifndef ONBOARD_CONTROL_LATERAL_POSTPROCESS_STEER_GAIN_CALIBRATION_H_
#define ONBOARD_CONTROL_LATERAL_POSTPROCESS_STEER_GAIN_CALIBRATION_H_
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

class SteerCalibration {
 public:
  SteerCalibration(const ControllerConf& control_conf,
                   const SteeringConverter* steering_converter);

  // steer_kappa_past is previous 0.1s steer_kappa
  double SteerCalibrationMain(double kappa_cmd, double kappa_cmd_past,
                              double pose_kappa, double kappa_cmd_before_delay,
                              double speed, double roll,
                              SteerCalibrationDebugProto* debug);

 private:
  double LatAccSteerGain(double speed, double kappa) const;
  double LatAccKappaCompensate(double speed, double kappa, double pose_kappa,
                               double kappa_cmd_before_delay) const;
  double RollCompensate(double roll) const;
  double RollAngleFilter(double roll, double roll_past);
  double QueryDynamicGain(double speed);

  const SteeringConverter* steering_converter_ = nullptr;
  bool enable_dynamic_prediction_pose_ = false;

  VehDynamicModelConf dynamic_conf_;
  double sliding_factor_ = 0.0;
  PiecewiseLinearFunction<double> steering_gain_wrt_speed_plf_;

  SteerDeadZoneAdaptorConf deadzone_conf_;
  double steer_gap_kappa_compensate_ = 0.0;
  double anti_gap_sign_ = 1.0;

  LatAccGainConf lat_gain_conf_;
  double roll_angle_filted_ = 0.0;
  PiecewiseLinearFunction<double> lat_acc_steer_plf_;
  PiecewiseLinearFunction<double> roll_steer_plf_;
};

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_LATERAL_POSTPROCESS_STEER_GAIN_CALIBRATION_H_
