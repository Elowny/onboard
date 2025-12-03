#include "onboard/planner/ml/optimizer_auto_tuning/auto_tuning_utils.h"

#include <algorithm>
#include <vector>

#include "Eigen/Core"

#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/trajectory_point.h"

namespace qcraft {
namespace planner {

using Mfob = MixedFourthOrderBicycle;

AccumulatedDiscountedCostsProto PoseTrajectoryToPolicy(
    int trajectory_steps_dat, const DdpOptimizer<Mfob>& solver,
    const TrajectoryProto& trajectory_proto, double gamma) {
  std::vector<TrajectoryPoint> traj_points;
  for (const auto& apollo_point : trajectory_proto.trajectory_point()) {
    traj_points.emplace_back(apollo_point);
  }
  const auto xs = Mfob::FitState(traj_points);
  const auto us = Mfob::FitControl(traj_points, Mfob::GetStateAtStep(xs, 0));

  return solver.EvaluateEachDiscountedAccumulativeCost(
      xs, us, trajectory_steps_dat, gamma);
}

}  // namespace planner
}  // namespace qcraft
