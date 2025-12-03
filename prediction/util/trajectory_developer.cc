
#include "onboard/prediction/util/trajectory_developer.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>

#include "absl/status/statusor.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/line2d.h"
#include "onboard/math/line_fitter.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kLenEpsilon = 1e-1;
constexpr double kTimeEpsilon = 1e-3;
const PiecewiseLinearFunction<double, double> kPathSpeedLimitPlf(
    std::vector<double>{5.0, 10.0, 15.0, 30.0},
    std::vector<double>{3.0, 5.0, 8.0, 15.0});

// Some lateral speed related params.
constexpr int kSpeedFitMinPts = 3;
constexpr int kLateralSpeedFitMinPts = 3;
constexpr double kLateralSpeedFitMinTime = 0.4;

// Pole placement parameters
const PiecewiseLinearFunction<double, double> kVelToPolesPlf(
    std::vector<double>{0.0, 0.5, 1.8, 6.6, 10.5, 11.5, 35.0},
    std::vector<double>{0.01, 0.2, 0.5, 0.8, 1.0, 1.3, 3.0});

// Safe guard brake trajectory
const PiecewiseLinearFunction<double, double> kSafeGuardBrakeMinSpeedPlf(
    std::vector<double>{1.0, 2.0, 5.0, 10.0, 40.0},
    std::vector<double>{0.0, 1.0, 2.0, 5.0, 20.0});

}  // namespace

std::vector<PredictedTrajectoryPoint>
CombinePathAndSpeedForPredictedTrajectoryPoints(
    const planner::DiscretizedPath& path,
    const planner::SpeedVector& speed_data) {
  std::vector<PredictedTrajectoryPoint> trajectory;
  if (path.size() <= 1) {
    return trajectory;
  }
  QCHECK_GT(speed_data.size(), 1);
  trajectory.reserve(64);
  double t = 0.0;
  auto create_predicted_trajectory =
      [&trajectory](const planner::SpeedPoint& point,
                    const PathPoint& path_point, double t) {
        PredictedTrajectoryPoint traj_point;
        traj_point.set_t(t);
        traj_point.set_s(point.s());
        traj_point.set_pos(Vec2d(path_point.x(), path_point.y()));
        traj_point.set_theta(path_point.theta());
        traj_point.set_kappa(path_point.kappa());
        traj_point.set_v(point.v());
        traj_point.set_a(point.a());
        trajectory.push_back(std::move(traj_point));
      };
  if (path.length() < kLenEpsilon) {
    speed_data.EvaluateByTime(t, kPredictionTimeStep,
                              [&path, &create_predicted_trajectory](
                                  const planner::SpeedPoint& point, double t) {
                                auto& path_point = path.front();
                                create_predicted_trajectory(point, path_point,
                                                            t);
                                return false;
                              });
  } else {
    speed_data.EvaluateByTime(t, kPredictionTimeStep,
                              [&path, &create_predicted_trajectory](
                                  const planner::SpeedPoint& point, double t) {
                                if (point.s() > path.length()) {
                                  return true;
                                }
                                auto path_point = path.Evaluate(point.s());
                                create_predicted_trajectory(point, path_point,
                                                            t);
                                return false;
                              });
  }
  trajectory.shrink_to_fit();
  return trajectory;
}

planner::SpeedVector DevelopConstAccSpeedProfile(double cur_speed, double acc,
                                                 double maintain_acc_time,
                                                 double min_speed, double dt,
                                                 double horizon) {
  double t = 0.0;
  planner::SpeedVector speed_vec;
  speed_vec.reserve(static_cast<int>(horizon / dt) + 1);
  planner::SpeedPoint sp0(t, /*s=*/0.0, cur_speed, acc,
                          /*j=*/0.0);
  speed_vec.push_back(std::move(sp0));
  t += dt;
  while (t <= horizon) {
    const auto& prev = speed_vec.back();
    planner::SpeedPoint sp1;
    sp1.set_t(t);
    if (t > maintain_acc_time) {
      sp1.set_v(prev.v());
      sp1.set_s(prev.s() + prev.v() * dt);
      speed_vec.push_back(std::move(sp1));
    } else {
      const auto target_v =
          std::max(std::max(prev.v() + acc * dt, 0.0), min_speed);
      sp1.set_v(target_v);
      sp1.set_s(prev.s() + 0.5 * (prev.v() + target_v) * dt);
      sp1.set_a((target_v - prev.v()) / dt);
      speed_vec.push_back(std::move(sp1));
    }
    t += dt;
  }
  return speed_vec;
}

double LineFitAccelerationByMotionHistory(
    absl::Span<const ObjectMotionState> history, double fit_time) {
  if (history.size() <= 1) return 0.0;
  QCHECK_GT(fit_time, 0.0);
  const double origin_ts = history.back().timestamp;
  std::vector<Vec2d> data_points;
  data_points.reserve(history.size());
  std::vector<double> weights;
  weights.reserve(history.size());
  constexpr double kWeightFactor = 1.0;
  double prev_v = 0.0;
  double prev_t = 0.0;
  constexpr double kEpsilon = 1e-3;
  constexpr double kBrakeCutOff = -10.0;
  constexpr double kAccCutOff = 4.0;

  for (int i = 0; i < history.size(); ++i) {
    const auto& state = history[i];
    const auto v = state.vel.norm();
    const double time_diff_in_second = state.timestamp - origin_ts;
    // If we find a discontinuity in velocity, we cut off the history from here.
    if (i > 0) {
      const double a_by_finit_diff =
          (v - prev_v) / std::max(time_diff_in_second - prev_t, kEpsilon);
      if (a_by_finit_diff < kBrakeCutOff || a_by_finit_diff > kAccCutOff) {
        weights.clear();
        data_points.clear();
      }
    }
    prev_v = v;
    prev_t = time_diff_in_second;
    // If history longer than needed, continue
    if (std::fabs(time_diff_in_second) > fit_time) {
      continue;
    }
    data_points.emplace_back(Vec2d(time_diff_in_second, v));
    const double normalized_fit_time = time_diff_in_second / fit_time;
    weights.emplace_back(std::exp(
        kWeightFactor * normalized_fit_time));  // use exponential decay.
  }

  if (data_points.size() < kSpeedFitMinPts) {
    return 0.0;
  }
  ASSIGN_OR_RETURN(const auto line, FitLine2dToData(data_points, weights), 0.0);
  const auto tangent = line.tangent();
  if (std::fabs(tangent.x()) < kTimeEpsilon) {
    return 0.0;
  }
  return tangent.y() / tangent.x();
}

planner::DiscretizedPath PredictedTrajectoryPointsToDiscretizedPath(
    absl::Span<const PredictedTrajectoryPoint> points) {
  planner::DiscretizedPath path;
  path.reserve(points.size());
  for (const auto& pt : points) {
    PathPoint path_pt;
    path_pt.set_x(pt.pos().x());
    path_pt.set_y(pt.pos().y());
    path_pt.set_s(pt.s());
    path_pt.set_theta(pt.theta());
    path_pt.set_kappa(pt.kappa());
    path.push_back(std::move(path_pt));
  }
  return path;
}

std::vector<PredictedTrajectoryPoint> DevelopStaticTrajectory(
    const UniCycleState& state, double dt, double horizon) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);
  points.reserve(num_pts);

  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(0.0);
  init_pt.set_a(0.0);
  points.push_back(init_pt);
  for (int i = 1; i < num_pts; ++i) {
    PredictedTrajectoryPoint next_pt = init_pt;
    next_pt.set_t(dt * i);
    points.push_back(std::move(next_pt));
  }
  return points;
}

std::vector<PredictedTrajectoryPoint> DevelopVoidTrajectory(
    const UniCycleState& state) {
  std::vector<PredictedTrajectoryPoint> points;
  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(state.v);
  init_pt.set_a(state.acc);
  points.push_back(init_pt);
  return points;
}

std::vector<PredictedTrajectoryPoint> DevelopForwardCTRATrajectory(
    const UniCycleState& state, double dt, double acc_stop_time,
    double yaw_rate_stop_time, double horizon) {
  QCHECK_GE(state.v, 0);
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);
  points.reserve(num_pts);

  const int acc_stop_pts = static_cast<int>(acc_stop_time / dt) + 1;
  const int yaw_stop_pts = static_cast<int>(yaw_rate_stop_time / dt) + 1;
  auto cur_state = state;
  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(cur_state.x, cur_state.y));
  init_pt.set_theta(cur_state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(cur_state.v);
  init_pt.set_a(cur_state.acc);
  points.push_back(std::move(init_pt));

  for (int i = 1; i < num_pts; ++i) {
    if (i >= acc_stop_pts) {
      cur_state.acc = 0.0;
    }
    if (i >= yaw_stop_pts) {
      cur_state.yaw_rate = 0.0;
    }
    if (cur_state.v + cur_state.acc * dt < 0) {
      cur_state.acc = -cur_state.v / dt;
    }
    const auto next_state = SimulateUniCycleModel(cur_state, dt);
    PredictedTrajectoryPoint next_pt;
    const auto& prev_pt = points.back();
    next_pt.set_t(i * dt);
    next_pt.set_pos(Vec2d(next_state.x, next_state.y));
    next_pt.set_s(prev_pt.s() + (next_pt.pos() - prev_pt.pos()).norm());
    next_pt.set_theta(next_state.heading);
    next_pt.set_v(next_state.v);
    next_pt.set_kappa(0.0);
    next_pt.set_a(next_state.acc);
    cur_state = next_state;
    points.push_back(std::move(next_pt));
  }
  return points;
}

std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const PredictionObject& obj, double dt, double horizon, bool is_reversed) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);
  points.reserve(num_pts);

  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(obj.pos().x(), obj.pos().y()));
  init_pt.set_theta(obj.heading());
  init_pt.set_kappa(0.0);
  init_pt.set_v(obj.v());
  init_pt.set_a(0.0);
  const Vec2d heading_normal = Vec2d::FastUnitFromAngle(init_pt.theta());
  points.push_back(init_pt);
  for (int i = 1; i < num_pts; ++i) {
    PredictedTrajectoryPoint next_pt;
    next_pt.set_t(dt * i);
    next_pt.set_s(init_pt.v() * dt * i);
    if (is_reversed) {
      next_pt.set_theta(NormalizeAngle(init_pt.theta() + M_PI));
      next_pt.set_pos(init_pt.pos() - heading_normal * obj.v() * dt * i);
    } else {
      next_pt.set_theta(init_pt.theta());
      next_pt.set_pos(init_pt.pos() + heading_normal * obj.v() * dt * i);
    }
    next_pt.set_kappa(0.0);
    next_pt.set_v(init_pt.v());
    next_pt.set_a(0.0);
    points.push_back(std::move(next_pt));
  }
  return points;
}
std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const UniCycleState& state, double dt, double horizon, bool is_reversed) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);
  points.reserve(num_pts);

  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(state.v);
  init_pt.set_a(0.0);
  const Vec2d heading_normal = Vec2d::FastUnitFromAngle(init_pt.theta());
  points.push_back(init_pt);
  for (int i = 1; i < num_pts; ++i) {
    PredictedTrajectoryPoint next_pt;
    next_pt.set_t(dt * i);
    next_pt.set_s(init_pt.v() * dt * i);
    if (is_reversed) {
      next_pt.set_theta(NormalizeAngle(init_pt.theta() + M_PI));
      next_pt.set_pos(init_pt.pos() - heading_normal * state.v * dt * i);
    } else {
      next_pt.set_theta(init_pt.theta());
      next_pt.set_pos(init_pt.pos() + heading_normal * state.v * dt * i);
    }
    next_pt.set_kappa(0.0);
    next_pt.set_v(init_pt.v());
    next_pt.set_a(0.0);
    points.push_back(std::move(next_pt));
  }
  return points;
}

double LineFitLateralSpeedByMotionHistory(
    absl::Span<const ObjectMotionState> history,
    const planner::DrivePassage& dp, double fit_time,
    bool clamp_by_lane_width) {
  if (history.size() <= 1) return 0.0;
  const double origin_ts = history.back().timestamp;
  std::vector<Vec2d> data_points;
  data_points.reserve(history.size());
  std::vector<double> weights;
  weights.reserve(history.size());
  constexpr double kWeightFactor = 1.0;
  QCHECK_GT(fit_time, 0.0);
  constexpr double kDefaultHalfLaneWidth = 2.0;
  for (const auto& state : history) {
    const double time_diff_in_second = state.timestamp - origin_ts;
    if (std::fabs(time_diff_in_second) > fit_time) {
      continue;
    }
    const auto obj_pos = state.pos;
    const auto frenet_sl_or = dp.QueryFrenetCoordinateAt(obj_pos);
    if (!frenet_sl_or.ok()) {
      continue;
    }
    double cur_l = frenet_sl_or->l;
    if (clamp_by_lane_width) {
      const auto bounds_or =
          dp.QueryNearestBoundaryLateralOffset(frenet_sl_or->s);
      double min_l = -kDefaultHalfLaneWidth;
      double max_l = kDefaultHalfLaneWidth;
      if (bounds_or.ok()) {
        min_l = std::max(min_l, bounds_or->first);
        max_l = std::min(max_l, bounds_or->second);
      }

      // If vehicle is far from target lane, avoid making a cut-in prediction.
      const double half_lane_width = 0.5 * (max_l - min_l);
      cur_l =
          std::clamp(cur_l, min_l - half_lane_width, max_l + half_lane_width);
    }

    data_points.emplace_back(Vec2d(time_diff_in_second, cur_l));
    const double normalized_fit_time = time_diff_in_second / fit_time;
    weights.emplace_back(std::exp(
        kWeightFactor * normalized_fit_time));  // use exponential decay.
  }
  if (data_points.size() < kLateralSpeedFitMinPts ||
      data_points.front().x() > -kLateralSpeedFitMinTime) {
    return 0.0;
  }
  ASSIGN_OR_RETURN(const auto line, FitLine2dToData(data_points, weights), 0.0);
  const auto tangent = line.tangent();
  if (std::fabs(tangent.x()) < kTimeEpsilon) {
    return 0.0;
  }
  return tangent.y() / tangent.x();
}

void ExtendTrajectoryAlongTangent(
    double horizon, const Vec2d& tangent,
    std::vector<PredictedTrajectoryPoint>* traj_pts) {
  const double angle = tangent.FastAngle();
  double cur_t = traj_pts->back().t();
  cur_t += kPredictionTimeStep;
  constexpr double kTEpsilon = 1e-5;
  while (cur_t < horizon + kTEpsilon) {
    auto pt = traj_pts->back();
    pt.set_t(cur_t);
    pt.set_theta(angle);
    pt.set_s(pt.s() + pt.v() * kPredictionTimeStep);
    pt.set_pos(pt.pos() + tangent * pt.v() * kPredictionTimeStep);
    traj_pts->push_back(std::move(pt));
    cur_t += kPredictionTimeStep;
  }
}

}  // namespace prediction
}  // namespace qcraft
