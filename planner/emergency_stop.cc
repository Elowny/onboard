#include "onboard/planner/emergency_stop.h"

#include <algorithm>
#include <cmath>
#include <ostream>

#include "glog/logging.h"

#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/path_util.h"

namespace qcraft {
namespace planner {
namespace aeb {

// For Planner 3.0
std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const EmergencyStopParamsProto& /*emergency_stop_params*/,
    const MotionConstraintParamsProto& motion_constraint_params) {
  VLOG(2) << "Generate emergency stop trajectory.";
  TrajectoryPoint plan_start_traj_point;
  plan_start_traj_point.FromProto(plan_start_point);

  return aeb::GenerateStopTrajectory(
      path_s_inc_from_prev, reset, /*forward=*/true,
      motion_constraint_params.max_deceleration(), motion_constraint_params,
      plan_start_traj_point, prev_traj_points);
}

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const MotionConstraintParamsProto& motion_constraint_params,
    const TrajectoryPoint& plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto>& prev_trajectory) {
  ApolloTrajectoryPointProto curr_point;
  plan_start_traj_point.ToProto(&curr_point);
  auto prev_traj = prev_trajectory;

  if (!forward) {
    curr_point.mutable_path_point()->set_theta(
        NormalizeAngle(curr_point.path_point().theta() + M_PI));
    curr_point.mutable_path_point()->set_kappa(
        -curr_point.path_point().kappa());
    curr_point.set_v(-curr_point.v());
    curr_point.set_a(-curr_point.a());
    curr_point.set_j(-curr_point.j());
    curr_point.mutable_path_point()->set_s(-curr_point.path_point().s());
    for (auto& pt : prev_traj) {
      pt.mutable_path_point()->set_theta(
          NormalizeAngle(pt.path_point().theta() + M_PI));
      pt.mutable_path_point()->set_kappa(-pt.path_point().kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_j(-pt.j());
      pt.mutable_path_point()->set_s(-pt.path_point().s());
    }
  }

  std::vector<ApolloTrajectoryPointProto> output_trajectory;
  double accumulate_s = 0.0;
  constexpr double kMinSpeed = 1e-6;
  constexpr double kMinDist = 1e-6;
  for (int i = 0; i < kTrajectorySteps; ++i) {
    const double dist =
        std::max(kMinDist, curr_point.v() * kTrajectoryTimeStep +
                               0.5 * max_deceleration * kTrajectoryTimeStep *
                                   kTrajectoryTimeStep);
    accumulate_s += dist;
    const double accumulate_s_at_prev_traj = accumulate_s + init_s;
    ApolloTrajectoryPointProto next_point;
    const auto& curr_path_point = curr_point.path_point();
    if (prev_traj.empty() || reset ||
        accumulate_s_at_prev_traj >= prev_traj.rbegin()->path_point().s()) {
      *next_point.mutable_path_point() =
          GetPathPointAlongCircle(curr_path_point, dist);
    } else {
      next_point =
          QueryApolloTrajectoryPointByS(prev_traj, accumulate_s_at_prev_traj);
    }
    next_point.set_relative_time(curr_point.relative_time() +
                                 kTrajectoryTimeStep);
    next_point.set_v(std::max(
        kMinSpeed, curr_point.v() + kTrajectoryTimeStep * max_deceleration));
    next_point.set_a(std::max(max_deceleration,
                              -1.0 * next_point.v() / kTrajectoryTimeStep));
    curr_point.set_j(
        std::clamp((next_point.a() - curr_point.a()) / kTrajectoryTimeStep,
                   motion_constraint_params.max_decel_jerk(),
                   motion_constraint_params.max_accel_jerk()));
    output_trajectory.push_back(curr_point);
    curr_point = next_point;
  }

  if (!forward) {
    for (auto& pt : output_trajectory) {
      pt.mutable_path_point()->set_theta(
          NormalizeAngle(pt.path_point().theta() + M_PI));
      pt.mutable_path_point()->set_kappa(-pt.path_point().kappa());
      pt.set_v(-pt.v());
      pt.set_a(-pt.a());
      pt.set_j(-pt.j());
      pt.mutable_path_point()->set_s(-pt.path_point().s());
    }
  }

  return output_trajectory;
}

}  // namespace aeb
}  // namespace planner
}  // namespace qcraft
