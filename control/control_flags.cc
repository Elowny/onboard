#include "onboard/control/control_flags.h"

#include <cmath>

DEFINE_bool(control_replay_pose, false,
            "Use pose from log as input to compare intermediate results.");
DEFINE_bool(control_replay_chassis, false,
            "Use chassis from log as input to compare intermediate results.");

DEFINE_double(control_estop_hard_brake_jerk, -4.0,
              "m/s^3, the longitudinal jerk at control emergency stop state.");
DEFINE_double(control_estop_hard_brake_acceleration, -2.0,
              "m/s^2, the minimal longitudinal acceleration at control "
              "emergency stop state.");
DEFINE_double(
    control_estop_soft_brake_jerk, -1.0,
    "m/s^3, the longitudinal jerk at control emergency stop soft brake state.");
DEFINE_double(control_estop_soft_brake_acceleration, -1.0,
              "m/s^2, the minimal longitudinal acceleration at control "
              "emergency stop soft brake state.");

DEFINE_double(control_max_error_warning_factor, 0.8,
              "The ratio to start send out kickout warning.");

// Values which will cause kick-out if reached.
DEFINE_bool(control_error_kickout_slack_mode, false,
            "apply slack threshold for control large error kickouts.");

DEFINE_double(max_steering_angle_diff_threshold, 0.5 * M_PI_2,
              "The max steering angle diff between steering anlge cmd and "
              "steering angle feedback");
DEFINE_double(engage_protection_min_speed, 1.0,
              " m/s, only apply engage protection when speed is over it");

// Station error gain for apa speed interface
DEFINE_double(control_apa_speed_gain, 1.0, "s error gain feedback to speed");
DEFINE_double(control_apa_min_distance, 0.1,
              "control_apa_min_distance for m5, when not fullstop");
DEFINE_double(control_apa_fullstop_distance, 0.1,
              "control_apa_fullstop_distance for m5");
DEFINE_double(control_apa_fullstop_speed, 0.3,
              "control_apa_fullstop_speed for m5");

DEFINE_bool(use_dynamic_steer_angle_bias, true,
            "use online identification steer angle bias rather than vehicle "
            "params config");

DEFINE_bool(apply_control_code_param, false,
            "overwrite controller conf with the param of vehicle "
            "classification level. ");
DEFINE_double(longitudinal_acc_jerk_limit, 3.0,
              "longitudinal acceleration jerk limitation");
DEFINE_double(longitudinal_dec_jerk_limit, -5.0,
              "longitudinal deceleration jerk limitation");

DEFINE_bool(force_use_lon_tob_mpc_controller, false,
            "use longitudinal third order kinematic mpc controller");
DEFINE_bool(force_use_lat_dm_mpc_controller, false,
            "apply lateral dynamic model based mpc controller.");

DEFINE_double(reference_acc_slack_factor, 1.0, "reference acc slack factor.");
DEFINE_double(reference_jerk_slack_factor, 1.0, "reference jerk slack factor.");
DEFINE_double(control_fullstop_speed, 0.3, "control_fullstop_speed");
DEFINE_double(control_fullstop_distance, 1.2, "control_fullstop_distance");

DEFINE_double(lss_control_predict_time, 0.1, "control lka predict time");
DEFINE_bool(torque_use_past_steer_cmd, false,
            "use past steer cmd for torque controller.");
