#include "onboard/control/control_monitoring.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "onboard/control/control_defs.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/counter.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"

namespace qcraft::control {

namespace {

void QEventBrakeInfo(const ControlCacheManager& control_cache_manager) {
  const ControlCacheManager::DecModeStat dec_mode_stat =
      control_cache_manager.CountDecMode();

  constexpr int kBrakeNumThreshold = 5;
  if (dec_mode_stat.count > kBrakeNumThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "control_brake_too_frequent",
                           /*seconds=*/3.0, [&](QEvent* qevent) {
                             qevent->AddField("brake_num", dec_mode_stat.count);
                           });
  }

  constexpr double kBrakeJerkThreshold =
      -2.0;  // m/s3, equivalent to -0.4 m/s2 lasting for 0.2s.
  double max_brake_jerk = 0.0;
  double brake_duration = 0.0;
  double brake_accl_cmd = 0.0;

  for (int i = 1; i < dec_mode_stat.brake_duration.size(); ++i) {
    constexpr double kMinDuration = 0.1;
    const double duration =
        std::max(dec_mode_stat.brake_duration[i], kMinDuration);
    const double avg_accl = dec_mode_stat.avg_accl_cmd[i];
    const double brake_jerk = avg_accl / duration;

    if (brake_jerk < max_brake_jerk) {
      max_brake_jerk = brake_jerk;
      brake_duration = duration;
      brake_accl_cmd = avg_accl;
    }
  }
  if (max_brake_jerk < kBrakeJerkThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "control_brake_uncomfortable",
                           /*seconds=*/3.0, [&](QEvent* qevent) {
                             qevent->AddField("brake_num", dec_mode_stat.count)
                                 .AddField("brake_duration", brake_duration)
                                 .AddField("brake_accl_cmd", brake_accl_cmd);
                           });
  }
}

void QEventMinRadius(double speed,
                     const ControlCacheManager& control_cache_manager) {
  constexpr double kDuration = 3.0;                // s.
  constexpr double kControlSteerThreshold = 98.0;  // percentage.
  constexpr double kChassisSteerThreshold = 90.0;  // percentage.
  constexpr double kSpeedThreshold = 2.0;          // m/s.

  const int steps = RoundToInt(kDuration * kControlFrequency);

  if (!control_cache_manager.IsInAlwaysAutoMode(steps)) return;

  const double control_min_steer_pct =
      control_cache_manager.QueryMinControlSteerPct(steps);
  const double chassis_min_steer_pct =
      control_cache_manager.QueryMinChassisSteerPct(steps);

  // Only consider left u turn case.
  if (control_min_steer_pct > kControlSteerThreshold &&
      chassis_min_steer_pct > kChassisSteerThreshold &&
      speed > kSpeedThreshold) {
    const double mean_kappa = std::max(
        1e-3, std::abs(control_cache_manager.QueryMeanKappaPose(steps)));
    const double min_radius = 1.0 / mean_kappa;

    QEVENT_EVERY_N_SECONDS(
        "zhichao", "minimum_turning_radius",
        /*seconds=*/10.0, [&](QEvent* qevent) {
          qevent->AddField("min_radius", min_radius)
              .AddField("control_min_steer_pct", control_min_steer_pct)
              .AddField("chassis_min_steer_pct", chassis_min_steer_pct)
              .AddField("speed", speed);
        });
  }
}

void QCounterLatControlError(const ControlError& control_error) {
  const int control_lateral_error_cm =
      RoundToInt(control_error.lateral_error() * 100.0);
  if (control_lateral_error_cm != 0) {
    QCOUNTER("control_lateral_error_cm", control_lateral_error_cm);
    QCOUNTER("abs_lateral_error_cm", std::abs(control_lateral_error_cm));
  }
  const int abs_heading_error_deg_1e6 =
      RoundToInt(std::abs(r2d(control_error.heading_error() * 1e6)));
  if (abs_heading_error_deg_1e6 != 0) {
    QCOUNTER("abs_heading_error_deg_1e6", abs_heading_error_deg_1e6);
  }
  const int steer_pct_error_100 =
      RoundToInt(control_error.steer_pct_error() * 100.0);
  QCOUNTER("steer_pct_error_100", steer_pct_error_100);
  QCOUNTER("abs_steer_pct_error_100", std::abs(steer_pct_error_100));
}

void QCounterLonControlError(const ControlError& control_error) {
  const int control_station_error_cm =
      RoundToInt(control_error.station_error() * 100.0);
  if (control_station_error_cm != 0) {
    QCOUNTER("control_station_error_cm", control_station_error_cm);
    QCOUNTER("abs_control_station_error_cm",
             std::abs(control_station_error_cm));
  }

  const int control_speed_error_cm =
      RoundToInt(control_error.speed_error() * 100.0);
  if (control_speed_error_cm != 0) {
    QCOUNTER("control_speed_error_cm/s", control_speed_error_cm);
    QCOUNTER("abs_control_speed_error_cm/s", std::abs(control_speed_error_cm));
  }

  const int control_acc_error_100 =
      RoundToInt(control_error.acceleration_error() * 100.0);
  if (control_acc_error_100 != 0) {
    QCOUNTER("control_acc_error_100", control_acc_error_100);
    QCOUNTER("abs_control_acc_error_100", std::abs(control_acc_error_100));
  }
}

void QEventVehicleMoveOff(int intented_to_move_step,
                          const ControlCacheManager& control_cache_manager) {
  const std::vector<double> move_off_time{1.0, 2.0, 3.0};
  std::vector<int> intented_to_move_step_after;
  intented_to_move_step_after.reserve(move_off_time.size());
  for (int i = 0; i < move_off_time.size(); ++i) {
    const int step = intented_to_move_step -
                     RoundToInt(move_off_time[i] * kControlFrequency);
    intented_to_move_step_after.push_back(step);
  }

  const double move_off_response_time =
      (intented_to_move_step - control_cache_manager.CalculateStepsMoveOff()) *
      kControlInterval;

  QEVENT("zhichao", "vehicle_move_off", [&](QEvent* event) {
    event->AddField("response_time", move_off_response_time)
        .AddField("1_s_speed", control_cache_manager.QuerySpeedPose(
                                   intented_to_move_step_after[0]))
        .AddField("2_s_speed", control_cache_manager.QuerySpeedPose(
                                   intented_to_move_step_after[1]))
        .AddField("3_s_speed", control_cache_manager.QuerySpeedPose(
                                   intented_to_move_step_after[2]));
  });
}

// U-turn only consider left u turn.
void QEventUTurn(double speed,
                 const ControlCacheManager& control_cache_manager) {
  if (speed < 2.0) return;

  constexpr double kMonitoringTime = 2.0;
  constexpr double kSteerCmdPctThr = 90.0;

  const double min_control_steer_pct =
      control_cache_manager.QueryMinControlSteerPct(
          RoundToInt(kMonitoringTime * kControlFrequency));
  const double min_canbus_steer_pct =
      control_cache_manager.QueryMinChassisSteerPct(
          RoundToInt(kMonitoringTime * kControlFrequency));

  if (min_control_steer_pct > kSteerCmdPctThr &&
      min_canbus_steer_pct > kSteerCmdPctThr) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "u-turn",
        /*seconds=*/10.0, [&](QEvent* qevent) {
          qevent->AddField("min_control_steer_pct", min_control_steer_pct)
              .AddField("min_canbus_steer_pct", min_canbus_steer_pct)
              .AddField("speed", speed);
        });
  }
}

}  // namespace

void QCounterPose(const PoseProto& pose_proto) {
  const double yaw_bias_by_vel =
      std::atan2(pose_proto.vel_body().y(),
                 std::max(pose_proto.vel_body().x(), /*min_speed*/ 0.01));
  QCOUNTER("yaw_bias_by_vel*1000", RoundToInt(r2d(yaw_bias_by_vel * 1000)));
  QCOUNTER("lat_acc*1000", RoundToInt(pose_proto.accel_body().y() * 1000));
  QCOUNTER("av_speed_kph", RoundToInt(Mps2Kph(pose_proto.speed())));
  QCOUNTER("roll_angle_deg*1000", RoundToInt(r2d(pose_proto.roll()) * 1000));
  QCOUNTER("vertical_acc*1000", RoundToInt(pose_proto.accel_body().z() * 1000));
}

void QCounterControlError(const VehicleStateProto& vehicle_state,
                          const ControlError& control_error) {
  if (vehicle_state.is_auto_speed()) {
    QCounterLonControlError(control_error);
  }

  if (vehicle_state.is_auto_steer()) {
    QCounterLatControlError(control_error);
  }
}

void QEventControlCache(double speed,
                        const ControlCacheManager& control_cache_manager) {
  // Monitor frequent braking refer to: go/frequent-brake.
  QEventBrakeInfo(control_cache_manager);

  QEventMinRadius(speed, control_cache_manager);

  const int intented_to_move_step =
      RoundToInt(kMoveOffMonitoringTime * kControlFrequency);
  if (control_cache_manager.IsVehicleIntentedToMove(intented_to_move_step)) {
    QEventVehicleMoveOff(intented_to_move_step, control_cache_manager);
  }

  QEventUTurn(speed, control_cache_manager);
}

void QEventDiscomfortable(const ControlCacheManager& control_cache_manager,
                          const SteeringConverter& steering_converter,
                          const PoseProto& pose) {
  constexpr double kDuration = 1.0;  // s.
  const int step = static_cast<int>(kDuration * kControlFrequency);
  const double delta_control_acc_cmd =
      control_cache_manager.QueryMaxControlAcc(step) -
      control_cache_manager.QueryMinControlAcc(step);

  const double max_kappa_cmd = control_cache_manager.QueryMaxKappaCmd(step);
  const double min_kappa_cmd = control_cache_manager.QueryMinKappaCmd(step);
  const double delta_control_steer_angle =
      steering_converter.KappaToSteerAngle(max_kappa_cmd) -
      steering_converter.KappaToSteerAngle(min_kappa_cmd);

  QEVENT("driver", "discomfort", [&](QEvent* event) {
    event->AddField("speed", pose.speed())
        .AddField("curvature", pose.curvature())
        .AddField("accel_x", pose.accel_body().x())
        .AddField("accel_y", pose.accel_body().y())
        .AddField("delta_accel_cmd_last_sec", delta_control_acc_cmd)
        .AddField("delta_steer_last_sec", delta_control_steer_angle);
  });
}

void QEventHardBrake(double control_cmd_accel, double traj_min_accel,
                     double traj_ref_accel,
                     const ControlCacheManager& control_cache_manager) {
  constexpr double kDuration = 1.0;  // s.
  const int step = static_cast<int>(kDuration * kControlFrequency);
  const double min_planner_accel_last_sec =
      control_cache_manager.QueryMinAccPlanner(step);

  QEVENT_EVERY_N_SECONDS(
      "zhichao", "control_hard_brake", /*seconds=*/3.0, [&](QEvent* qevent) {
        qevent->AddField("control_accel", control_cmd_accel)
            .AddField("planner_accel", traj_ref_accel)
            .AddField("traj_min_accel", traj_min_accel)
            .AddField("min_planner_accel_last_sec", min_planner_accel_last_sec);
      });
}

void QEventTrackingError(double steer_ref_pct, double speed_ref,
                         const ControlCacheManager& control_cache_manager) {
  // Large lat or lon error qevent;
  constexpr double kLatErrorThreshold = 0.3;  // m.
  constexpr double kLonErrorThreshold = 1.0;  // m.

  const ControlError curr_control_error =
      control_cache_manager.QueryControlError(/*steps*/ 0);
  if (std::abs(curr_control_error.lateral_error()) > kLatErrorThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "large_lat_error",
        /*seconds=*/5.0, [&](QEvent* qevent) {
          qevent->AddField("lat_error", curr_control_error.lateral_error())
              .AddField("steer_ref_pct", steer_ref_pct);
        });
  }

  if (std::abs(curr_control_error.station_error()) > kLonErrorThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "large_lon_error",
        /*seconds=*/5.0, [&](QEvent* qevent) {
          qevent->AddField("lon_error", curr_control_error.station_error())
              .AddField("speed_ref", speed_ref);
        });
  }

  // Continuous large lat or lon error qevent;
  constexpr double kLatErrorCacheThreshold = 0.15;  // m.
  constexpr double kLonErrorCacheThreshold = 0.5;   // m.
  constexpr double kDuration = 5.0;                 // s.
  const int step = RoundToInt(kDuration * kControlFrequency);

  const double min_abs_lat_error =
      control_cache_manager.QueryMinAbsLatError(step);
  if (min_abs_lat_error > kLatErrorCacheThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "continuous_large_lat_error",
                           /*seconds=*/5.0, [&](QEvent* qevent) {
                             qevent->AddField("min_lat_error_in_last_5s",
                                              min_abs_lat_error);
                           });
  }

  const double min_abs_lon_error =
      control_cache_manager.QueryMinAbsLonError(step);
  if (min_abs_lon_error > kLonErrorCacheThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "continuous_large_lon_error",
                           /*seconds=*/5.0, [&](QEvent* qevent) {
                             qevent->AddField("min_lon_error_in_last_5s",
                                              min_abs_lon_error);
                           });
  }
}

void QEventLatVehPosDiff(const VehPose& vehpos_km, const VehPose& vehpos_dm) {
  constexpr double kXYDistanceThreshold = 0.05;               // m.
  constexpr double kHeadingDiffThreshold = d2r(1.0);          // rad.
  constexpr double kMovingDirectionDiffThreshold = d2r(1.0);  // rad.
  constexpr double kLatVelDiffThreshold = 0.2;                // m/s.

  const VehPose veh_pose_diff = vehpos_km - vehpos_dm;
  const double xy_distance =
      std::sqrt(Sqr(veh_pose_diff.x) + Sqr(veh_pose_diff.y));
  if (xy_distance > kXYDistanceThreshold ||
      std::abs(veh_pose_diff.heading) > kHeadingDiffThreshold ||
      std::abs(veh_pose_diff.moving_direction) >
          kMovingDirectionDiffThreshold ||
      std::abs(veh_pose_diff.lateral_velocity) > kLatVelDiffThreshold) {
    QLOG(INFO) << veh_pose_diff.DebugString();
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "veh_pose_predict_diff_too_large",
        /*seconds=*/5.0, [&](QEvent* qevent) {
          qevent->AddField("xy_distance_diff", xy_distance)
              .AddField("heading_diff", veh_pose_diff.heading)
              .AddField("moving_direction_diff", veh_pose_diff.moving_direction)
              .AddField("lateral_velocity_diff",
                        veh_pose_diff.lateral_velocity);
        });
  }
}

}  // namespace qcraft::control
