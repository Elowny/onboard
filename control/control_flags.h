#ifndef ONBOARD_CONTROL_CONTROL_FLAGS_H_
#define ONBOARD_CONTROL_CONTROL_FLAGS_H_

#include "gflags/gflags.h"
DECLARE_bool(control_replay_pose);
DECLARE_bool(control_replay_chassis);

DECLARE_double(control_estop_hard_brake_jerk);
DECLARE_double(control_estop_hard_brake_acceleration);
DECLARE_double(control_estop_soft_brake_jerk);
DECLARE_double(control_estop_soft_brake_acceleration);

DECLARE_double(control_max_error_warning_factor);

DECLARE_bool(control_error_kickout_slack_mode);

DECLARE_double(max_steering_angle_diff_threshold);
DECLARE_double(engage_protection_min_speed);

// Station error gain for apa speed interface
DECLARE_double(control_apa_speed_gain);
DECLARE_double(control_apa_min_distance);
DECLARE_double(control_apa_fullstop_distance);
DECLARE_double(control_apa_fullstop_speed);

DECLARE_bool(use_dynamic_steer_angle_bias);

DECLARE_bool(apply_control_code_param);

DECLARE_double(longitudinal_acc_jerk_limit);
DECLARE_double(longitudinal_dec_jerk_limit);

DECLARE_bool(force_use_lon_tob_mpc_controller);
DECLARE_bool(force_use_lat_dm_mpc_controller);

DECLARE_double(reference_acc_slack_factor);
DECLARE_double(reference_jerk_slack_factor);

DECLARE_double(control_fullstop_distance);
DECLARE_double(control_fullstop_speed);

DECLARE_double(lss_control_predict_time);
DECLARE_bool(torque_use_past_steer_cmd);
#endif  // ONBOARD_CONTROL_CONTROL_FLAGS_H_
