#include "onboard/control/control_cache_manager.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <vector>

#include "absl/strings/str_cat.h"

#include "onboard/control/control_defs.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

namespace {

// Query the ControlStateData last n steps ago.
const ControlCacheManager::ControlStateData& QueryControlStateData(
    int steps,
    const boost::circular_buffer<ControlCacheManager::ControlStateData>&
        control_state_cache) {
  const int size = control_state_cache.size();
  QCHECK_GE(steps, 0);
  QCHECK_LT(steps, size);

  return control_state_cache[size - 1 - steps];
}

template <typename T>
std::string VectorToString(const std::vector<T>& vec) {
  std::stringstream debug_msg;
  for (auto it = vec.begin(); it != vec.end(); ++it) {
    if (it != vec.begin()) {
      debug_msg << ", ";
    }
    debug_msg << *it;
  }
  return debug_msg.str();
}

}  // namespace

ControlCacheManager::ControlCacheManager() {
  ControlStateData default_control_state_date;
  control_state_cache_.assign(kCacheSize, default_control_state_date);
}

void ControlCacheManager::UpdateCacheData(
    const ControlCommand& control_cmd, const VehicleStateProto& vehicle_state,
    OptionalArgs optional_args) {
  ControlStateData control_state;
  control_state.is_auto_mode = vehicle_state.is_auto_mode();
  control_state.is_auto_steer = vehicle_state.is_auto_steer();
  control_state.is_auto_speed = vehicle_state.is_auto_speed();

  control_state.kappa_cmd = control_cmd.curvature();
  control_state.steer_pct_control = control_cmd.steering_target();
  control_state.steer_pct_chassis = vehicle_state.chassis_steering_percentage();
  control_state.control_acceleration = control_cmd.acceleration();
  control_state.steer_speed_target = control_cmd.steer_speed_target();
  control_state.acc_target =
      control_cmd.debug().simple_mpc_debug().acceleration_cmd_closeloop();
  control_state.acc_planner =
      control_cmd.debug().simple_mpc_debug().acceleration_reference();
  control_state.speed_mode = control_cmd.speed_mode();
  control_state.kappa_pose = vehicle_state.kappa();
  control_state.speed_pose = vehicle_state.linear_velocity();
  control_state.lateral_acceleration_pose =
      vehicle_state.pose().accel_body().y();
  control_state.acc_pose = vehicle_state.pose().accel_body().x();
  control_state.control_error.CopyFrom(control_cmd.debug().control_error());

  if (optional_args.control_debug) {
    const auto& debug_info =
        optional_args.control_debug->speed_mode_debug_proto();
    control_state.is_full_stop = debug_info.is_full_stop();
    control_state.is_standstill = debug_info.standstill();
  }

  if (optional_args.trajectory_interface &&
      !optional_args.trajectory_interface->GetAllTrajPoints().empty()) {
    const double relative_time =
        vehicle_state.timestamp() -
        optional_args.trajectory_interface->GetPlannerStartTime() +
        control_cmd.debug().bias_estimation_debug().steer_delay_online();
    control_state.planner_kappa =
        optional_args.trajectory_interface
            ->QueryTrajPointByRelativeTime(relative_time)
            .path_point()
            .kappa();
  }

  if (optional_args.veh_predicted_pose) {
    control_state.veh_predicted_pose = *optional_args.veh_predicted_pose;
  }

  // Record lateral error due to canbus noise.
  // https://qcraft.feishu.cn/docs/doccnAswCIYivABjOOv3zxCj4Jb
  if (optional_args.delay_time && optional_args.steering_converter) {
    const double delay_time = optional_args.delay_time.value();
    const double steer_pct_chassis_now =
        QuerySteerPctChassis(/*step*/ 0) + control_cmd.steer_angle_bias();
    const int delay_steps = static_cast<int>(delay_time * kControlFrequency);
    const double steer_pct_control_delay = QuerySteerPctControl(delay_steps);
    const double canbus_kappa_now =
        optional_args.steering_converter->SteerPctToKappa(
            steer_pct_chassis_now);
    const double control_kappa_delay =
        optional_args.steering_converter->SteerPctToKappa(
            steer_pct_control_delay);

    control_state.lat_error_canbus =
        IsInAlwaysSteerMode(delay_steps)
            ? Sqr(vehicle_state.linear_velocity() * kControlInterval) *
                  (canbus_kappa_now - control_kappa_delay)
            : 0.0;
  }

  control_state_cache_.push_back(control_state);
}

double ControlCacheManager::QueryKappaCmd(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).kappa_cmd;
}

double ControlCacheManager::QueryPlannerKappa(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).planner_kappa;
}

double ControlCacheManager::QuerySteerPctChassis(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).steer_pct_chassis;
}

double ControlCacheManager::QuerySteerPctControl(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).steer_pct_control;
}

double ControlCacheManager::QuerySteerSpeedTarget(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).steer_speed_target;
}

double ControlCacheManager::QueryAccTarget(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).acc_target;
}

double ControlCacheManager::QueryAccPose(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).acc_pose;
}

double ControlCacheManager::QuerySpeedPose(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).speed_pose;
}

const VehPoseProto& ControlCacheManager::QueryPredictPose(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).veh_predicted_pose;
}

ControlError ControlCacheManager::QueryControlError(int steps) const {
  return QueryControlStateData(steps, control_state_cache_).control_error;
}

std::vector<double> ControlCacheManager::QueryKappaCmdVector(
    int recent_steps) const {
  std::vector<double> result;
  result.reserve(recent_steps);
  for (int i = 0; i < recent_steps; ++i) {
    result.push_back(QueryKappaCmd(recent_steps - i - 1));
  }

  return result;
}

std::vector<double> ControlCacheManager::QuerySteerSpeedTargetVector(
    int recent_steps) const {
  std::vector<double> result;
  result.reserve(recent_steps);
  for (int i = 0; i < recent_steps; ++i) {
    result.push_back(QuerySteerSpeedTarget(recent_steps - i - 1));
  }

  return result;
}

std::vector<double> ControlCacheManager::QueryAccTargetVector(
    int recent_steps) const {
  std::vector<double> result;
  result.reserve(recent_steps);
  for (int i = 0; i < recent_steps; ++i) {
    result.push_back(QueryAccTarget(recent_steps - i - 1));
  }

  return result;
}

std::vector<double> ControlCacheManager::QueryAccPoseVector(
    int recent_steps) const {
  std::vector<double> result;
  result.reserve(recent_steps);
  for (int i = 0; i < recent_steps; ++i) {
    result.push_back(QueryAccPose(recent_steps - i - 1));
  }

  return result;
}

double ControlCacheManager::QueryMaxControlAcc(int steps) const {
  auto state = control_state_cache_.rbegin();
  double result = state->control_acceleration;
  for (int i = 0; i < steps; ++state, ++i) {
    result = state->is_auto_steer
                 ? std::max(result, state->control_acceleration)
                 : result;
  }
  return result;
}

double ControlCacheManager::QueryMinControlAcc(int steps) const {
  auto state = control_state_cache_.rbegin();
  double result = state->control_acceleration;
  for (int i = 0; i < steps; ++state, ++i) {
    result = state->is_auto_speed
                 ? std::min(result, state->control_acceleration)
                 : result;
  }
  return result;
}

double ControlCacheManager::QueryMaxKappaCmd(int steps) const {
  auto state = control_state_cache_.rbegin();
  double result = state->kappa_cmd;
  for (int i = 0; i < steps; ++state, ++i) {
    result = state->is_auto_steer ? std::max(result, state->kappa_cmd) : result;
  }
  return result;
}

double ControlCacheManager::QueryMinKappaCmd(int steps) const {
  auto state = control_state_cache_.rbegin();
  double result = state->kappa_cmd;
  for (int i = 0; i < steps; ++state, ++i) {
    result = state->is_auto_steer ? std::min(result, state->kappa_cmd) : result;
  }
  return result;
}

double ControlCacheManager::QueryMinAccPlanner(int steps) const {
  const int start_index = std::max(kCacheSize - steps, 0);
  const auto data = std::min_element(
      control_state_cache_.begin() + start_index, control_state_cache_.end(),
      [](const ControlStateData& data0, const ControlStateData& data1) {
        return data0.acc_planner < data1.acc_planner;
      });

  return data->acc_planner;
}

double ControlCacheManager::IntegrateLatErrorCanbus(int steps) const {
  double result = 0.0;
  auto state = control_state_cache_.rbegin();
  for (int i = 0; i < steps && state != control_state_cache_.rend();
       ++state, ++i) {
    result += state->lat_error_canbus;
  }
  return result;
}

double ControlCacheManager::QueryMinAbsLatError(int steps) const {
  const int start_index = std::max(kCacheSize - steps, 0);
  const auto data = std::min_element(
      control_state_cache_.begin() + start_index, control_state_cache_.end(),
      [](const ControlStateData& data0, const ControlStateData& data1) {
        return std::abs(data0.control_error.lateral_error()) <
               std::abs(data1.control_error.lateral_error());
      });

  return data->control_error.lateral_error();
}

double ControlCacheManager::QueryMinAbsLatACC(int steps) const {
  const int start_index = std::max(kCacheSize - steps, 0);
  const auto data = std::min_element(
      control_state_cache_.begin() + start_index, control_state_cache_.end(),
      [](const ControlStateData& data0, const ControlStateData& data1) {
        return std::abs(data0.lateral_acceleration_pose) <
               std::abs(data1.lateral_acceleration_pose);
      });

  return data->lateral_acceleration_pose;
}

double ControlCacheManager::QueryMinAbsLonError(int steps) const {
  const int start_index = std::max(kCacheSize - steps, 0);
  const auto data = std::min_element(
      control_state_cache_.begin() + start_index, control_state_cache_.end(),
      [](const ControlStateData& data0, const ControlStateData& data1) {
        return std::abs(data0.control_error.station_error()) <
               std::abs(data1.control_error.station_error());
      });

  return data->control_error.station_error();
}

double ControlCacheManager::QueryMinControlSteerPct(int steps) const {
  const int start_index = std::max(kCacheSize - steps, 0);
  const auto data = std::min_element(
      control_state_cache_.begin() + start_index, control_state_cache_.end(),
      [](const ControlStateData& data0, const ControlStateData& data1) {
        return data0.steer_pct_control < data1.steer_pct_control;
      });
  return data->steer_pct_control;
}

double ControlCacheManager::QueryMinChassisSteerPct(int steps) const {
  const int start_index = std::max(kCacheSize - steps, 0);
  const auto data = std::min_element(
      control_state_cache_.begin() + start_index, control_state_cache_.end(),
      [](const ControlStateData& data0, const ControlStateData& data1) {
        return data0.steer_pct_chassis < data1.steer_pct_chassis;
      });
  return data->steer_pct_chassis;
}

double ControlCacheManager::QueryMeanKappaPose(int steps) const {
  QCHECK_GT(steps, 0);

  double kappa_sum = 0.0;
  auto it = control_state_cache_.rbegin();
  for (int i = 0; i < steps && it != control_state_cache_.rend(); ++i) {
    kappa_sum += it->kappa_pose;
    ++it;
  }

  return kappa_sum * (1.0 / steps);
}

bool ControlCacheManager::IsInAlwaysAutoMode(int steps) const {
  QCHECK_GT(steps, 0) << "The steps should be a positive integer.";
  for (auto rit = control_state_cache_.rbegin();
       rit != control_state_cache_.rend() && steps > 0; ++rit, --steps) {
    if (!rit->is_auto_mode) return false;
  }
  return true;
}

bool ControlCacheManager::IsInAlwaysSteerMode(int steps) const {
  QCHECK_GT(steps, 0) << "The steps should be a positive integer.";
  for (auto rit = control_state_cache_.rbegin();
       rit != control_state_cache_.rend() && steps > 0; ++rit, --steps) {
    if (!rit->is_auto_steer) return false;
  }
  return true;
}

bool ControlCacheManager::IsInAlwaysSpeedMode(int steps) const {
  QCHECK_GT(steps, 0) << "The steps should be a positive integer.";
  for (auto rit = control_state_cache_.rbegin();
       rit != control_state_cache_.rend() && steps > 0; ++rit, --steps) {
    if (!rit->is_auto_speed) return false;
  }
  return true;
}

void ControlCacheManager::DecModeStat::CountOneBrake(double duration,
                                                     double interval,
                                                     double sum_accl) {
  ++count;
  brake_duration.push_back(duration);
  brake_interval.push_back(count == 1 ? 0.0 : interval);
  avg_accl_cmd.push_back(sum_accl / duration);
}

bool ControlCacheManager::DecModeStat::operator==(
    const ControlCacheManager::DecModeStat& compared) const {
  if (count != compared.count) return false;
  if (brake_duration != compared.brake_duration) return false;
  if (brake_interval != compared.brake_interval) return false;

  return true;
}

std::string ControlCacheManager::DecModeStat::DebugString() const {
  return absl::StrCat(
      "count: ", count, "; brake_duration: [", VectorToString(brake_duration),
      "]; brake_interval: [", VectorToString(brake_interval),
      "], average_acceleration: [", VectorToString(avg_accl_cmd), "].");
}

ControlCacheManager::DecModeStat ControlCacheManager::CountDecMode() const {
  ControlCacheManager::DecModeStat dec_mode_stat;

  if (!IsInAlwaysSpeedMode(kCacheSize)) return dec_mode_stat;

  struct StepCounter {
    int duration_step = 0;
    int interval_step = 0;
    double sum_accl = 0.0;

    void Reset() {
      duration_step = 0;
      interval_step = 0;
      sum_accl = 0.0;
    }
  };

  StepCounter step_counter;
  for (int i = 0; i < control_state_cache_.size(); ++i) {
    const SpeedMode& curr_mode = control_state_cache_[i].speed_mode;
    switch (curr_mode) {
      case ACC_MODE:
      case DISABLE:
      case IDLE_MODE:
        ++step_counter.interval_step;
        break;
      case DEC_MODE:
        ++step_counter.duration_step;
        step_counter.sum_accl +=
            control_state_cache_[i].control_acceleration * kControlInterval;
        break;
    }

    if (i == 0) continue;

    const SpeedMode& prev_mode = control_state_cache_[i - 1].speed_mode;
    if (prev_mode == SpeedMode::DEC_MODE && curr_mode == SpeedMode::ACC_MODE) {
      dec_mode_stat.CountOneBrake(kControlInterval * step_counter.duration_step,
                                  kControlInterval * step_counter.interval_step,
                                  step_counter.sum_accl);
      step_counter.Reset();
    }
  }

  return dec_mode_stat;
}

bool ControlCacheManager::IsVehicleIntentedToMove(int steps_ago) const {
  if (steps_ago < 1 || steps_ago > kCacheSize - 1) return false;

  auto rit = control_state_cache_.rbegin();
  for (; rit != control_state_cache_.rend() && steps_ago > 0;
       ++rit, --steps_ago) {
    if (rit->is_full_stop) return false;
  }

  if (rit->is_full_stop) return true;

  return false;
}

int ControlCacheManager::CalculateStepsMoveOff() const {
  int steps = 0;

  for (auto rit = control_state_cache_.rbegin();
       rit != control_state_cache_.rend(); ++rit, ++steps) {
    if (rit->is_standstill) return steps;
  }

  return steps;
}

}  // namespace control
}  // namespace qcraft
