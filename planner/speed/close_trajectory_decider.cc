#include "onboard/planner/speed/close_trajectory_decider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/st_close_trajectory.h"

namespace qcraft::planner {
namespace {
using SpeedLimitRange = SpeedLimit::SpeedLimitRange;
using StNearestPoint = StCloseTrajectory::StNearestPoint;

constexpr double kMaxDecelByCloseTraj = -1.5;  // m/s^2.

constexpr double kSpeedLimitBeforeRangeDist = 10.0;  // m.
constexpr double kSpeedLimitAfterRangeDist = 3.0;    // m.

const std::vector<double> kCloseObjectLatDistRange = {0.0, 0.5, 1.0, 1.5,
                                                      2.0};  // m.

const PiecewiseLinearFunction<double, double> kMovingVehicleMaxRelVPlf = {
    kCloseObjectLatDistRange, {3.0, 4.0, 6.0, 8.0, 10.0}};  // m/s.
const PiecewiseLinearFunction<double, double> kMovingCyclistMaxRelVPlf = {
    kCloseObjectLatDistRange, {3.0, 4.0, 6.0, 8.0, 10.0}};  // m/s.
const PiecewiseLinearFunction<double, double> kMovingPedestrianMaxRelVPlf = {
    kCloseObjectLatDistRange, {3.0, 4.0, 6.0, 8.0, 10.0}};  // m/s.

const PiecewiseLinearFunction<double, double> kTimeAttenuationPlf = {
    {0.0, 10.0}, {1.0, 1.2}};
const PiecewiseLinearFunction<double, double> kProbabilityGainPlf = {
    {0.0, 1.0}, {1.2, 1.0}};

const PiecewiseLinearFunction<double, double> kTimeAttenuationPlfForCutIn = {
    {3.0, 7.0, 10.0}, {1.0, 1.08, 1.2}};

double GetMaxSpeedByNearestPoint(const StNearestPoint& nearest_point,
                                 StBoundaryProto::ObjectType object_type) {
  double max_rel_speed = 0.0;
  const double lat_dist = std::fabs(nearest_point.lat_dist);
  switch (object_type) {
    case StBoundaryProto::VEHICLE:
      max_rel_speed = kMovingVehicleMaxRelVPlf(lat_dist);
      break;
    case StBoundaryProto::CYCLIST:
      max_rel_speed = kMovingCyclistMaxRelVPlf(lat_dist);
      break;
    case StBoundaryProto::PEDESTRIAN:
      max_rel_speed = kMovingPedestrianMaxRelVPlf(lat_dist);
      break;
    case StBoundaryProto::STATIC:
    case StBoundaryProto::IMPASSABLE_BOUNDARY:
    case StBoundaryProto::PATH_BOUNDARY:
    case StBoundaryProto::VIRTUAL:
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::UNKNOWN_OBJECT:
      max_rel_speed = kMovingVehicleMaxRelVPlf(lat_dist);
  }
  return max_rel_speed + std::max(nearest_point.v, 0.0);
}

SpeedLimitRange MakeSpeedLimitRange(
    double time, absl::string_view id,
    std::vector<const StCloseTrajectory*> st_close_trajs, double path_length,
    double av_speed) {
  double probability = 0.0;
  StNearestPoint nearest_pt;
  for (const auto* st_close_traj : st_close_trajs) {
    const auto temp_pt = st_close_traj->GetNearestPointByTime(time);
    const double temp_probability = st_close_traj->probability();
    QCHECK(temp_pt.has_value());
    nearest_pt.s += temp_pt->s * temp_probability;
    nearest_pt.v += temp_pt->v * temp_probability;
    nearest_pt.lat_dist += temp_pt->lat_dist * temp_probability;
    probability += temp_probability;
  }
  QCHECK_GT(probability, 0.0);
  nearest_pt.s /= probability;
  nearest_pt.v /= probability;
  nearest_pt.lat_dist /= probability;
  double speed_limit =
      GetMaxSpeedByNearestPoint(nearest_pt, st_close_trajs[0]->object_type());
  const double time_gain = kTimeAttenuationPlf(time);
  const double prob_gain = kProbabilityGainPlf(probability);
  speed_limit *= time_gain * prob_gain;
  const double start_s =
      std::max(0.0, nearest_pt.s - kSpeedLimitBeforeRangeDist);
  const double end_s =
      std::min(path_length, nearest_pt.s + kSpeedLimitAfterRangeDist);
  // Prevent hard braking.
  const double decel =
      (Sqr(speed_limit) - Sqr(av_speed)) / ((2.0 * start_s) + 1e-3);
  if (decel < kMaxDecelByCloseTraj) {
    speed_limit = std::sqrt(
        std::max(0.0, 2.0 * kMaxDecelByCloseTraj * start_s + Sqr(av_speed)));
  }
  std::string debug_info = absl::StrFormat(
      "CLOSE_OBJECT(moving) Id: %s, lat_dis: %.2f, obj_lon_v: %.2f, time: "
      "%.2f, prob: %.2f, value: %.2f",
      id, nearest_pt.lat_dist, nearest_pt.v, time, probability, speed_limit);
  return SpeedLimitRange{.start_s = start_s,
                         .end_s = end_s,
                         .speed_limit = speed_limit,
                         .info = std::move(debug_info)};
}

std::optional<SpeedLimit> MakeMovingSpeedLimit(
    double time, absl::Span<const StCloseTrajectory> st_close_trajs,
    double path_length, double av_speed) {
  std::vector<SpeedLimitRange> speed_limit_ranges;
  absl::flat_hash_map<std::string, std::vector<const StCloseTrajectory*>>
      obj_close_trajs_map;
  for (const StCloseTrajectory& st_close_traj : st_close_trajs) {
    if (!InRange(time, st_close_traj.min_t(), st_close_traj.max_t())) {
      continue;
    }
    obj_close_trajs_map[st_close_traj.object_id()].emplace_back(&st_close_traj);
  }
  for (const auto& [id, st_close_trajs] : obj_close_trajs_map) {
    auto speed_limit_range =
        MakeSpeedLimitRange(time, id, st_close_trajs, path_length, av_speed);
    speed_limit_ranges.push_back(std::move(speed_limit_range));
  }
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}
}  // namespace

std::vector<std::optional<SpeedLimit>> GetMovingCloseTrajSpeedLimits(
    absl::Span<const StCloseTrajectory> st_close_trajs, double path_length,
    double av_speed, double time_step, double max_time) {
  FUNC_QTRACE();
  if (st_close_trajs.empty()) return {};
  std::vector<std::optional<SpeedLimit>> dynamic_speed_limit;
  dynamic_speed_limit.resize(kTrajectorySteps + 1);
  constexpr double kTimeHorizon = kTrajectorySteps * kTrajectoryTimeStep;
  const double max_search_time = std::min(kTimeHorizon, max_time);
  for (double time = st_close_trajs.front().min_t(); time <= max_search_time;
       time += time_step) {
    auto speed_limit =
        MakeMovingSpeedLimit(time, st_close_trajs, path_length, av_speed);
    if (!speed_limit.has_value()) continue;
    const int time_idx = static_cast<int>(time / time_step);
    dynamic_speed_limit[time_idx] = std::move(speed_limit);
  }
  return dynamic_speed_limit;
}
}  // namespace qcraft::planner
