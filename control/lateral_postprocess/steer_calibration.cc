#include "onboard/control/lateral_postprocess/steer_calibration.h"

#include <algorithm>
#include <cmath>
#include <ostream>

#include "onboard/control/control_defs.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
namespace qcraft {
namespace control {
namespace {

double UpdateAntiGapSign(double anti_gap_sign_last, double delta_kappa) {
  constexpr double kDiffKappathreshold = 0.000003;  // m^-1
  double anti_gap_sign = anti_gap_sign_last;
  if ((anti_gap_sign_last > 0.0) && (delta_kappa < -kDiffKappathreshold)) {
    anti_gap_sign = -1.0;
  } else if ((anti_gap_sign_last < 0.0) &&
             (delta_kappa > kDiffKappathreshold)) {
    anti_gap_sign = 1.0;
  }
  return anti_gap_sign;
}

double UpdateSteerGapCompensation(double steer_gap_kappa_compensate_last,
                                  double anti_gap_sign,
                                  double steer_gap_kappa) {
  // Compensate one side of the steer gap in 1s.
  const double max_diff_kappa_compensate = steer_gap_kappa * kControlInterval;
  double steer_gap_kappa_compensate = steer_gap_kappa_compensate_last +
                                      max_diff_kappa_compensate * anti_gap_sign;
  steer_gap_kappa_compensate =
      std::clamp(steer_gap_kappa_compensate, -steer_gap_kappa, steer_gap_kappa);
  return steer_gap_kappa_compensate;
}

}  // namespace

double SteerCalibration::QueryDynamicGain(double speed) {
  constexpr double kMaxDynamicGain = 2.5;
  constexpr double kMinDynamicGain = 0.99;
  double dynamic_gain = (1 - sliding_factor_ * speed * speed);
  QCHECK(dynamic_gain > kMinDynamicGain && dynamic_gain < kMaxDynamicGain)
      << "[control] Dynamic_gain is over bound. dynamic_gain: " << dynamic_gain;
  return dynamic_gain;
}

double SteerCalibration::LatAccSteerGain(double speed, double kappa) const {
  double lat_acc_steer_gain = 1.0;
  const double lateral_acceleration = std::abs(Sqr(speed) * kappa);
  if (speed > lat_gain_conf_.velocity_threshold()) {
    lat_acc_steer_gain = lat_acc_steer_plf_(lateral_acceleration);
  }
  return lat_acc_steer_gain;
}

double SteerCalibration::LatAccKappaCompensate(
    double speed, double kappa, double pose_kappa,
    double kappa_cmd_before_delay) const {
  double lat_acc_kappa_compensate = 0.0;
  const double lateral_acceleration = std::abs(Sqr(speed) * kappa);
  if ((speed > lat_gain_conf_.velocity_threshold()) &&
      (lateral_acceleration > lat_gain_conf_.lat_acc_threshold())) {
    lat_acc_kappa_compensate = lat_gain_conf_.kappa_compensate_ratio() *
                               (kappa_cmd_before_delay - pose_kappa);
  }
  return lat_acc_kappa_compensate;
}

double SteerCalibration::RollAngleFilter(double roll, double roll_past) {
  constexpr double kMaxRollAngle = 0.1;  // rad
  const double weight_filter = kControlInterval;
  const double roll_clamp =
      std::clamp(NormalizeAngle(roll), -kMaxRollAngle, kMaxRollAngle);
  const double roll_angle_updated =
      std::clamp(NormalizeAngle(roll_past * (1 - weight_filter) +
                                weight_filter * roll_clamp),
                 -kMaxRollAngle, kMaxRollAngle);
  return roll_angle_updated;
}

double SteerCalibration::RollCompensate(double roll) const {
  double roll_compesate = roll_steer_plf_(roll);
  return steering_converter_->SteerAngleToKappa(roll_compesate);
}

double SteerCalibration::SteerCalibrationMain(
    double kappa_cmd, double kappa_cmd_past, double pose_kappa,
    double kappa_cmd_before_delay, double speed, double roll,
    SteerCalibrationDebugProto* debug) {
  SCOPED_QTRACE("SteerCalibration::SteerCalibrationMain");
  double steer_gain = 1.0;
  double speed_gain = 1.0;
  double dynamic_gain = 1.0;
  double lat_acc_steer_gain = 1.0;
  double lat_acc_kappa_compensate = 0.0;
  double kappa_roll_compensate = 0.0;

  if (dynamic_conf_.enable_dynamic_model_compensation()) {
    dynamic_gain = QueryDynamicGain(speed);
    steer_gain *= dynamic_gain;
  } else if (lat_gain_conf_.enable_lat_acc_gain() &&
             enable_dynamic_prediction_pose_) {
    lat_acc_steer_gain = LatAccSteerGain(speed, kappa_cmd);
    steer_gain *= lat_acc_steer_gain;
    lat_acc_kappa_compensate = LatAccKappaCompensate(
        speed, kappa_cmd, pose_kappa, kappa_cmd_before_delay);
    constexpr double kMaxLatAccCompensation = 0.8;  // m/s^2.
    const double max_compensation =
        kMaxLatAccCompensation /
        Sqr(std::max(lat_gain_conf_.velocity_threshold(), speed));
    lat_acc_kappa_compensate = std::clamp(lat_acc_kappa_compensate,
                                          -max_compensation, max_compensation);
  }
  anti_gap_sign_ =
      UpdateAntiGapSign(anti_gap_sign_, kappa_cmd - kappa_cmd_past);
  steer_gap_kappa_compensate_ =
      UpdateSteerGapCompensation(steer_gap_kappa_compensate_, anti_gap_sign_,
                                 deadzone_conf_.steer_gap_kappa());

  if (dynamic_conf_.enable_roll_compensation()) {
    roll_angle_filted_ = RollAngleFilter(roll, roll_angle_filted_);
    kappa_roll_compensate = RollCompensate(roll_angle_filted_);
  }

  const double output_steer_kappa =
      kappa_cmd * steer_gain + steer_gap_kappa_compensate_ +
      lat_acc_kappa_compensate + kappa_roll_compensate;

  const double output_steer_kappa_clamped =
      steering_converter_->ClampKappaByMaxSteerAngle(output_steer_kappa);
  const double output_steer_percentage =
      steering_converter_->KappaToSteerPct(output_steer_kappa_clamped);

  debug->set_roll_angle_filted(roll_angle_filted_);
  debug->set_speed_gain(speed_gain);
  debug->set_dynamic_gain(dynamic_gain);
  debug->set_post_process_gain(steer_gain);
  debug->set_sliding_gain(lat_acc_steer_gain);
  debug->set_steer_gap_kappa_compensate(steer_gap_kappa_compensate_);
  debug->set_lat_acc_kappa_compensate(lat_acc_kappa_compensate);
  debug->set_input_steer_kappa(kappa_cmd);
  debug->set_output_steer_kappa(output_steer_kappa);
  debug->set_output_steer_percentage(output_steer_percentage);
  debug->set_input_steer_kappa_past(kappa_cmd_before_delay);
  debug->set_roll_compensate_kappa(kappa_roll_compensate);

  return output_steer_percentage;
}

SteerCalibration::SteerCalibration(const ControllerConf& control_conf,
                                   const SteeringConverter* steering_converter)
    : steering_converter_(steering_converter) {
  QCHECK_NOTNULL(steering_converter_);

  // Dynamic_conf init.
  dynamic_conf_ = control_conf.veh_dynamic_model_conf();
  const double mf = dynamic_conf_.mass_fl() + dynamic_conf_.mass_fr();
  const double mr = dynamic_conf_.mass_rl() + dynamic_conf_.mass_rr();
  const double cf = dynamic_conf_.c_fl() + dynamic_conf_.c_fr();
  const double cr = dynamic_conf_.c_rl() + dynamic_conf_.c_rr();
  const double wheelbase_f = dynamic_conf_.wheelbase_f();
  const double wheelbase_r = dynamic_conf_.wheelbase_r();
  const double wheelbase = wheelbase_f + wheelbase_r;

  if (std::fabs(wheelbase * wheelbase * cf * cr) > 0.01) {
    sliding_factor_ = (mf + mr) * (cf * wheelbase_f - cr * wheelbase_r) /
                      (wheelbase * wheelbase * cf * cr);
  }
  // Deadzone_conf init.
  deadzone_conf_ = control_conf.steer_deadzone_adaptor_conf();

  // Lateral kappa feedback.
  lat_gain_conf_ = control_conf.lat_acc_gain_conf();
  if (lat_gain_conf_.has_lat_acc_steer_plf()) {
    lat_acc_steer_plf_ =
        PiecewiseLinearFunctionFromProto(lat_gain_conf_.lat_acc_steer_plf());
  }
  if (dynamic_conf_.has_roll_steer_plf()) {
    roll_steer_plf_ =
        PiecewiseLinearFunctionFromProto(dynamic_conf_.roll_steer_plf());
  }
  enable_dynamic_prediction_pose_ =
      control_conf.enable_dynamic_prediction_pose();
}

}  // namespace control
}  // namespace qcraft
