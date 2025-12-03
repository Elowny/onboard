#include "onboard/planner/speed/decider/st_boundary_modifier_util.h"

#include <memory>
#include <utility>

#include "onboard/math/util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace planner {
namespace {

using prediction::PredictedTrajectoryPoint;

constexpr double kEps = 1e-6;

std::vector<PredictedTrajectoryPoint>
GenerateUniformDecelPredTrajPointsAfterDelay(
    const std::vector<PredictedTrajectoryPoint>& pred_traj_points, double delay,
    double decel) {
  QCHECK(!pred_traj_points.empty());
  QCHECK_LE(decel, 0.0);
  std::vector<PredictedTrajectoryPoint> decel_traj_points;
  decel_traj_points.reserve(pred_traj_points.size());
  const double t_step = pred_traj_points.size() > 1
                            ? pred_traj_points[1].t() - pred_traj_points[0].t()
                            : prediction::kPredictionTimeStep;
  decel_traj_points.push_back(pred_traj_points.front());
  for (int i = 1; i < pred_traj_points.size(); ++i) {
    const double curr_t = i * t_step;
    const auto& traj_pt = pred_traj_points[i];
    if (curr_t < delay) {
      decel_traj_points.push_back(traj_pt);
      continue;
    }
    PredictedTrajectoryPoint next_traj_pt;
    auto& prev_traj_pt = decel_traj_points.back();
    if (prev_traj_pt.v() < kEps) {
      next_traj_pt = prev_traj_pt;
      next_traj_pt.set_t(curr_t);
      next_traj_pt.set_v(0.0);
      next_traj_pt.set_a(0.0);
      decel_traj_points.push_back(std::move(next_traj_pt));
      continue;
    }
    const double init_v = prev_traj_pt.v();
    const double init_s = prev_traj_pt.s();
    const double curr_v = std::max(0.0, init_v + decel * t_step);
    const double curr_s =
        init_s + std::max(0.0, init_v * t_step + 0.5 * decel * Sqr(t_step));
    next_traj_pt = QueryTrajectoryPointByS(pred_traj_points, curr_s);
    next_traj_pt.set_t(curr_t);
    next_traj_pt.set_v(curr_v);
    prev_traj_pt.set_a((curr_v - prev_traj_pt.v()) / t_step);
    decel_traj_points.push_back(std::move(next_traj_pt));
  }
  return decel_traj_points;
}

std::vector<PredictedTrajectoryPoint> GenerateNudgePredTrajPoints(
    const SpacetimeObjectTrajectory& st_traj, double lambda, double max_kappa) {
  const std::vector<PredictedTrajectoryPoint>& pred_traj_points =
      st_traj.trajectory().points();
  QCHECK(!pred_traj_points.empty());
  std::vector<PredictedTrajectoryPoint> nudge_pred_traj;
  nudge_pred_traj.reserve(pred_traj_points.size());
  const double t_step = pred_traj_points.size() > 1
                            ? pred_traj_points[1].t() - pred_traj_points[0].t()
                            : prediction::kPredictionTimeStep;
  const auto& current_pose = st_traj.planner_object().pose();
  const double init_v = current_pose.v();
  const double ds = init_v * t_step;
  const double dkappa = lambda * ds;
  PathPoint path_point;
  path_point.set_x(current_pose.pos().x());
  path_point.set_y(current_pose.pos().y());
  path_point.set_theta(current_pose.theta());
  path_point.set_s(0.0);
  path_point.set_kappa(current_pose.kappa());
  path_point.set_lambda(lambda);
  max_kappa = std::max(max_kappa, std::abs(path_point.kappa()));

  PredictedTrajectoryPoint current_traj_pt;
  SetPredictedTrajectoryPointSpatialInfoFromPathPoint(path_point,
                                                      &current_traj_pt);
  current_traj_pt.set_t(0.0);
  current_traj_pt.set_v(init_v);
  current_traj_pt.set_a(0.0);
  nudge_pred_traj.push_back(std::move(current_traj_pt));

  for (int i = 1; i < pred_traj_points.size(); ++i) {
    const double curr_t = i * t_step;
    if (const double next_kappa = path_point.kappa() + dkappa;
        std::abs(next_kappa) < max_kappa) {
      path_point.set_kappa(next_kappa);
      path_point.set_lambda(lambda);
    }
    path_point = GetPathPointAlongCircle(path_point, ds);
    PredictedTrajectoryPoint next_traj_pt;
    SetPredictedTrajectoryPointSpatialInfoFromPathPoint(path_point,
                                                        &next_traj_pt);
    next_traj_pt.set_t(curr_t);
    next_traj_pt.set_v(init_v);
    next_traj_pt.set_a(0.0);
    nudge_pred_traj.push_back(std::move(next_traj_pt));
  }
  return nudge_pred_traj;
}

}  // namespace

SpacetimeObjectTrajectory CreateSpacetimeTrajectoryByDecelAfterDelay(
    const SpacetimeObjectTrajectory& st_traj, double delay, double decel) {
  auto decel_pred_traj = GenerateUniformDecelPredTrajPointsAfterDelay(
      st_traj.trajectory().points(), delay, decel);
  auto new_pred_traj = st_traj.trajectory();
  *new_pred_traj.mutable_points() = std::move(decel_pred_traj);
  return st_traj.CreateTrajectoryMutatedInstance(std::move(new_pred_traj));
}

SpacetimeObjectTrajectory CreateNudgeSpacetimeTrajectory(
    const SpacetimeObjectTrajectory& st_traj, double lambda, double max_kappa) {
  auto nudge_pred_traj =
      GenerateNudgePredTrajPoints(st_traj, lambda, max_kappa);
  auto new_pred_traj = st_traj.trajectory();
  *new_pred_traj.mutable_points() = std::move(nudge_pred_traj);
  return st_traj.CreateTrajectoryMutatedInstance(std::move(new_pred_traj));
}

double ComputeOncomingObjectDesiredAccel(
    const std::vector<StBoundaryRef>& st_boundaries, double v_ego,
    double yield_time_buffer) {
  constexpr double kEps = 1e-6;
  double desired_acc = 0.0;
  for (const auto& st_boundary : st_boundaries) {
    const auto& bottom_right_point = st_boundary->bottom_right_point();
    const double s = std::max(bottom_right_point.s(), kEps);
    const double t = std::max(bottom_right_point.t() + yield_time_buffer, kEps);
    const double decel_to_stop = -0.5 * (v_ego * v_ego) / s;
    desired_acc = std::min(desired_acc, v_ego < -decel_to_stop * t
                                            ? decel_to_stop
                                            : 2.0 * (s - v_ego * t) / (t * t));
  }
  return desired_acc;
}

}  // namespace planner
}  // namespace qcraft
