#include "onboard/proto/control_cmd.pb.h"

/**
 * Manage speed
 *  https://docs.google.com/document/d/1J9ZSnC1SAWxIQ2cXL4RwHffdI_8mR2Cqzz4SgFsWXj0/edit#
 * Closed loop acc
 * https://qcraft.feishu.cn/docs/doccnNDVOnD3BUk13Fs2cki1kCc#oU4bVF
 */
namespace qcraft {
namespace control {

struct ClosedLoopAccInput {
  bool is_available_idle = false;
  bool is_auto_mode = false;
  bool is_full_stop = false;
  double acc_feedback = 0.0;
  double acc_idle = 0.0;
  double acc_target_past = 0.0;
  double acc_offset = 0.0;
  double throttle_deadzone = 0.0;
  double brake_deadzone = 0.0;
  double vel = 0.0;
  double steer_rad_abs = 0.0;
};

struct ClosedLoopAccState {
  qcraft::SpeedMode speed_mode = qcraft::SpeedMode::ACC_MODE;
  qcraft::SpeedMode speed_mode_last = qcraft::SpeedMode::ACC_MODE;
  int dec2acc_counter = 0;
  int acc2dec_counter = 0;
  bool is_first_frame = true;
  double acc_calib = 0.0;
  double acc_cmd = 0.0;
  double acc_bound = 0.0;
  double expt_acc = 0.0;
  double acc_lowerbound = 0.0;
  double dec_upperbound = 0.0;

  double executor_percentage = 0.0;
  double calib_value = 0.0;
  double delta_value = 0.0;
  double delta_value_p = 0.0;
  double delta_value_i = 0.0;
  double delta_value_d = 0.0;
  double calib_value_raw = 0.0;

  // Only for record in proto.
  double throttle_last = 0.0;
  double diff_acc_last = 0.0;
  double throttle_integral = 0.0;
  double diff_acc = 0.0;
  double brake_last = 0.0;
  double diff_dec_last = 0.0;
  double brake_integral = 0.0;
  double diff_dec = 0.0;
  int lon_delay_step = 15;
};

}  // namespace control
}  // namespace qcraft
