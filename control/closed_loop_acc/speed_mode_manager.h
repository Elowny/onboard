#ifndef ONBOARD_CONTROL_CLOSED_LOOP_ACC_SPEED_MODE_MANAGER_H_
#define ONBOARD_CONTROL_CLOSED_LOOP_ACC_SPEED_MODE_MANAGER_H_

#include <deque>

#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/proto/control_cmd.pb.h"

/**
 * Manage speed
 *  https://docs.google.com/document/d/1J9ZSnC1SAWxIQ2cXL4RwHffdI_8mR2Cqzz4SgFsWXj0/edit#
 * Closed loop acc
 * https://qcraft.feishu.cn/docs/doccnNDVOnD3BUk13Fs2cki1kCc#oU4bVF
 */
namespace qcraft {
namespace control {
// Control throttle/brake to track acc cmd from mpc
// TODO(yangyu): add unit tests for closed loop acc controller.

class ClosedLoopAcc {
 public:
  ClosedLoopAcc(const ControllerConf& control_conf, bool is_available_idle,
                double throttle_lowerbound, double brake_lowerbound);

  // Main function, return acc command.
  double UpdateAccCommand(bool is_auto_mode, bool is_full_stop,
                          double acc_calib, double acc_cmd, double acc_offset,
                          double acc_idle,
                          ControllerDebugProto* controller_debug_proto);

  double UpdateCalibrationCmd(bool is_auto_mode, bool is_full_stop,
                              double calib_value, double acc_feedback,
                              double steer_rad_abs,
                              ControllerDebugProto* controller_debug_proto);
  SpeedMode GetCurrSpeedMode();
  double GetLongitudinalDelay(double brake_delay, double throttle_delay);

 private:
  // Update speed mode by acc cmd and counter.
  void UpdateSpeedMode();
  // Limit upper bound or lower bound of acc cmd by speed mode.
  void LimitAccBySpeedMode();
  // Reset when veh is not auto.
  void ResetSpeedMode();

  void CalculteSpeedModeAccBound(double acc_idle);
  void ResetControlStatus();
  void UpdateThrottle();
  void UpdateBrake();

  SpeedMode speed_mode_ = SpeedMode::DISABLE;
  SpeedMode speed_mode_last_ = SpeedMode::DISABLE;

  bool is_available_idle_ = false;
  int acc_counter_ = 0;
  int dec_counter_ = 0;
  int acc_counter_threshold_ = 0;
  int dec_counter_threshold_ = 0;
  bool is_first_frame_ = true;
  double steer_rad_abs_ = 0.0;
  double acc_calib_ = 0.0;
  double acc_cmd_ = 0.0;
  double acc_bound_ = 0.0;
  double acc_lowerbound_ = 0.0;
  double dec_upperbound_ = 0.0;
  double hysteresis_zone_ = 0.0;
  double acc_feedback_ = 0.0;
  std::deque<double> acc_cmd_past_;

  int acc_deque_size_ = 0;
  int throttle_delay_cycle_ = 0;
  int brake_delay_cycle_ = 0;
  double value_ = 0.0;
  double calib_value_ = 0.0;
  double delta_value_ = 0.0;
  double delta_value_p_ = 0.0;
  double delta_value_i_ = 0.0;
  double delta_value_d_ = 0.0;
  double calib_value_raw_ = 0.0;

  double throttle_deadzone_ = 0.0;
  double throttle_windup_ = 0.0;
  double kp_acc_ = 0.0;
  double ki_acc_ = 0.0;
  double kd_acc_ = 0.0;
  double throttle_last_ = 0.0;
  double diff_acc_last_ = 0.0;
  double throttle_integral_ = 0.0;
  double diff_acc_ = 0.0;
  double max_throttle_ = 0.0;
  double min_delta_throttle_ = 0.0;
  double max_delta_throttle_ = 0.0;
  double steer2throttle_ratio_ = 0.0;
  double brake_deadzone_ = 0.0;
  double brake_windup_ = 0.0;
  double kp_dec_ = 0.0;
  double ki_dec_ = 0.0;
  double kd_dec_ = 0.0;
  double brake_last_ = 0.0;
  double diff_dec_last_ = 0.0;
  double brake_integral_ = 0.0;
  double diff_dec_ = 0.0;
  double max_brake_ = 0.0;
  double min_delta_brake_ = 0.0;
  double max_delta_brake_ = 0.0;
  double keep_brake_decrease_threshold_ = 0.0;
  double acc_offset_ = 0.0;
  double expt_acc_ = 0.0;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CLOSED_LOOP_ACC_SPEED_MODE_MANAGER_H_
