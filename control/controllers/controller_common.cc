#include "onboard/control/controllers/controller_common.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/proto/vehicle_signal.pb.h"

namespace qcraft {
namespace control {

namespace {

ControlError CombineControlErrors(const LatControlError& lat_control_error,
                                  const LonControlError& lon_control_error,
                                  double lat_error_canbus,
                                  double steer_pct_error) {
  ControlError control_error;
  control_error.set_lateral_error(lat_control_error.lateral_error);
  control_error.set_heading_error(lat_control_error.heading_error);
  control_error.set_speed_error(lon_control_error.speed_error);
  control_error.set_station_error(lon_control_error.station_error);
  control_error.set_lateral_error_canbus(lat_error_canbus);
  control_error.set_steer_pct_error(steer_pct_error);
  return control_error;
}

}  // namespace

LatControlError CalculateLatControlError(const Vec2d& av_xy, double av_yaw,
                                         const PathPoint& ref_point) {
  const Vec2d ref_point_xy(ref_point.x(), ref_point.y());
  const Vec2d ref_point_tangent = Vec2d::UnitFromAngle(ref_point.theta());
  const Vec2d ref_point_normal = ref_point_tangent.Perp();
  const double ref_heading = ref_point.theta();

  LatControlError lat_control_error;
  lat_control_error.lateral_error =
      (av_xy - ref_point_xy).dot(ref_point_normal);
  lat_control_error.heading_error = NormalizeAngle(av_yaw - ref_heading);

  return lat_control_error;
}

LonControlError CalculateLonControlError(
    const Vec2d& av_xy, double av_speed, double acc_error,
    const ApolloTrajectoryPointProto& ref_point) {
  const Vec2d ref_xy(ref_point.path_point().x(), ref_point.path_point().y());
  const Vec2d ref_tangent =
      Vec2d::UnitFromAngle(ref_point.path_point().theta());

  LonControlError lon_control_error;
  lon_control_error.station_error = (av_xy - ref_xy).dot(ref_tangent);
  lon_control_error.speed_error = av_speed - ref_point.v();
  lon_control_error.acceleration_error = acc_error;

  return lon_control_error;
}

// In simulation, lat controller and lon controller may use different trajectory
// to calculate controller errors.
ControlError CalculateControlError(
    bool enable_yaw_consider_slip, const VehicleStateProto& vehicle_state,
    const TrajectoryInterface& lon_controller_trajectory_interface,
    const TrajectoryInterface& lat_controller_trajectory_interface,
    double lat_error_canbus, double acc_target_past, double steer_target_pct) {
  QCHECK(!lat_controller_trajectory_interface.GetAllTrajPoints().empty());
  QCHECK(!lon_controller_trajectory_interface.GetAllTrajPoints().empty());

  const auto closest_path_point = lat_controller_trajectory_interface
                                      .QueryNearestTrajPointByXY(Vec2d(
                                          vehicle_state.x(), vehicle_state.y()))
                                      .path_point();
  const Vec2d av_xy(vehicle_state.x(), vehicle_state.y());
  const double av_yaw = enable_yaw_consider_slip
                            ? vehicle_state.moving_direction()
                            : vehicle_state.yaw();
  LatControlError lat_control_error =
      CalculateLatControlError(av_xy, av_yaw, closest_path_point);

  const double control_relative_time =
      vehicle_state.timestamp() -
      lon_controller_trajectory_interface.GetPlannerStartTime();
  const ApolloTrajectoryPointProto ref_traj_point =
      lon_controller_trajectory_interface.QueryTrajPointByRelativeTime(
          control_relative_time);
  LonControlError lon_control_error = CalculateLonControlError(
      av_xy, vehicle_state.linear_velocity(),
      vehicle_state.linear_acceleration() - acc_target_past, ref_traj_point);
  // steer pct
  const double steer_pct_error =
      steer_target_pct - vehicle_state.chassis_steering_percentage();
  return CombineControlErrors(lat_control_error, lon_control_error,
                              lat_error_canbus, steer_pct_error);
}

std::vector<double> CalcSControlHorizonSpeedSequence(
    bool is_stationary, double ts, double speed_at_beginning,
    absl::Span<const double> t_control_acc,
    Chassis::GearPosition chassis_gear) {
  // For preventing large steer in low speed
  // Todo: (yangyu) be removed after fix it.
  constexpr double kMinSpeedMPCThreshold = 0.1;  // m/s;
  const int s_control_horizon = t_control_acc.size();
  if (is_stationary) {
    return std::vector<double>(s_control_horizon, 0.0);
  }
  std::vector<double> speed;
  speed.reserve(s_control_horizon);
  speed.push_back(speed_at_beginning);
  for (int i = 0; i < s_control_horizon - 1; ++i) {
    double v = speed.back() + t_control_acc[i] * ts;
    // Cutoff cross-zero speed wrt chassis gear.
    switch (chassis_gear) {
      case Chassis::GEAR_DRIVE:
        v = std::max(v, kMinSpeedMPCThreshold);
        break;
      case Chassis::GEAR_REVERSE:
        v = std::min(v, -kMinSpeedMPCThreshold);
        break;
      case Chassis::GEAR_PARKING:
        v = 0.0;
        break;
      case Chassis::GEAR_NEUTRAL:
        v = v * speed_at_beginning < 0.0 ? 0.0 : v;
        break;
      default:
        break;
    }
    speed.push_back(v);
  }
  return speed;
}

ControlError CalculateLateralPredictionError(
    const VehicleStateProto& vehicle_state,
    const ControlCacheManager& control_cache_mgr, double steer_delay) {
  const auto& predict_pose = control_cache_mgr.QueryPredictPose(
      FloorToInt(steer_delay * kControlFrequency));
  bool is_history_auto = control_cache_mgr.IsInAlwaysAutoMode(
      FloorToInt(steer_delay * kControlFrequency));

  ControlError control_error;
  if (!is_history_auto) {
    control_error.set_heading_error(0.0);
    control_error.set_lateral_error(0.0);
    return control_error;
  }

  const Vec2d vs_pos(predict_pose.x(), predict_pose.y());

  const Vec2d vehicle_state_xy(vehicle_state.x(), vehicle_state.y());
  const Vec2d vehicle_state_tangent = Vec2d::UnitFromAngle(vehicle_state.yaw());

  const Vec2d vehicle_state_normal = vehicle_state_tangent.Perp();
  control_error.set_heading_error(NormalizeAngle(
      predict_pose.heading() - vehicle_state.moving_direction()));
  control_error.set_lateral_error(
      (vs_pos - vehicle_state_xy).dot(vehicle_state_normal));

  return control_error;
}

std::vector<double> ComputeStepLengthFromTControl(
    const std::vector<double>& speed_vec, double mpc_period) {
  QCHECK_EQ(speed_vec.size(), kSControlHorizon);
  std::vector<double> t_control_s_vec;
  t_control_s_vec.assign(kSControlHorizon, 0.0);
  t_control_s_vec[0] = 0.5 * (speed_vec[0] + speed_vec[1]) * mpc_period;
  for (int i = 1; i < kSControlHorizon - 1; ++i) {
    t_control_s_vec[i] = t_control_s_vec[i - 1] +
                         0.5 * (speed_vec[i] + speed_vec[i + 1]) * mpc_period;
  }
  // The last s is approximated with uniform motion.
  t_control_s_vec[kSControlHorizon - 1] =
      t_control_s_vec[kSControlHorizon - 2] +
      speed_vec[kSControlHorizon - 1] * mpc_period;

  return t_control_s_vec;
}

void UpdateStepLengthToProto(const std::vector<double>& t_control_s_vec,
                             ControllerDebugProto::MPCDebugProto* mpc_debug) {
  for (int i = 0; i < t_control_s_vec.size(); ++i) {
    const double s0 = i == 0 ? 0.0 : t_control_s_vec[i - 1];
    const double s1 = t_control_s_vec[i];
    mpc_debug->add_step_length(s1 - s0);
  }
}

void LightControl(const TrajectoryProto& trajectory,
                  ControlCommand* control_cmd) {
  switch (trajectory.turn_signal()) {
    case TURN_SIGNAL_LEFT:
      control_cmd->mutable_signal()->set_emergency_light(false);
      control_cmd->mutable_signal()->set_turn_signal(VehicleSignal::TURN_LEFT);
      break;
    case TURN_SIGNAL_RIGHT:
      control_cmd->mutable_signal()->set_emergency_light(false);
      control_cmd->mutable_signal()->set_turn_signal(VehicleSignal::TURN_RIGHT);
      break;
    case TURN_SIGNAL_EMERGENCY:
      control_cmd->mutable_signal()->set_emergency_light(true);
      control_cmd->mutable_signal()->set_turn_signal(VehicleSignal::TURN_NONE);
      break;
    case TURN_SIGNAL_NONE:
      control_cmd->mutable_signal()->set_emergency_light(false);
      control_cmd->mutable_signal()->set_turn_signal(VehicleSignal::TURN_NONE);
      break;
  }
}

void DoorControl(const TrajectoryProto& trajectory,
                 ControlCommand* control_cmd) {
  control_cmd->set_door_open(trajectory.has_door_decision() &&
                             trajectory.door_decision().door_state() ==
                                 DoorDecision::DOOR_OPEN);
}

absl::StatusOr<Chassis::GearPosition> GenerateGearCmd(
    Chassis::GearPosition gear_fb, Chassis::GearPosition gear_target,
    double av_speed) {
  constexpr double kStopStandStillSpeedThreshold = 0.2;  // m/s;
  // No need to shift gear.
  if (gear_fb == gear_target) {
    return gear_target;
  }
  // Need to shift gear below.
  const auto is_stop_standstill =
      std::fabs(av_speed) < kStopStandStillSpeedThreshold;
  if (!is_stop_standstill) {
    // Allow to set gear from N to D or R when av is moving.
    if (gear_fb == Chassis::GEAR_NEUTRAL) {
      return gear_target;
    }
    // Unreasonable gear shifting from R to D, or D to R when av is moving.
    return absl::InternalError(absl::StrCat(
        "Gear shifting is not reasonable \n, speed: ", av_speed,
        " | stop standstill speed threshold: ", kStopStandStillSpeedThreshold,
        " | chassis_gear: ", Chassis::GearPosition_Name(gear_fb),
        " | trajectory_gear: ", Chassis::GearPosition_Name(gear_target)));
  }

  // Need to go through N firstly when D -> R , R -> D , R -> P , D -> P.
  if ((gear_fb == Chassis::GEAR_DRIVE &&
       gear_target == Chassis::GEAR_REVERSE) ||
      (gear_fb == Chassis::GEAR_REVERSE &&
       gear_target == Chassis::GEAR_DRIVE) ||
      (gear_fb == Chassis::GEAR_REVERSE &&
       gear_target == Chassis::GEAR_PARKING) ||
      (gear_fb == Chassis::GEAR_DRIVE &&
       gear_target == Chassis::GEAR_PARKING)) {
    return Chassis::GEAR_NEUTRAL;
  } else {
    return gear_target;
  }
}

// TODO(shijun): delete full stop condition related configs.
bool IsFullStop(double trajectory_accumulate_s, double av_speed,
                bool is_freesapce) {
  if (is_freesapce) {
    return std::fabs(trajectory_accumulate_s) <
               FLAGS_control_apa_fullstop_distance &&
           std::fabs(av_speed) < FLAGS_control_apa_fullstop_speed;
  }
  return std::fabs(trajectory_accumulate_s) < FLAGS_control_fullstop_distance &&
         std::fabs(av_speed) < FLAGS_control_fullstop_speed;
}

bool IsStandstill(double av_speed) {
  constexpr double kStandStillSpeed = 0.03;  // m/s
  return std::fabs(av_speed) < kStandStillSpeed;
}

}  // namespace control
}  // namespace qcraft
