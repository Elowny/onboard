#include "onboard/planner/optimization/ddp/trajectory_optimizer_state.h"

#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

void TrajectoryOptimizerState::FromProto(
    const TrajectoryOptimizerStateProto& proto) {
  const int n = static_cast<int>(proto.last_optimized_trajectory_size());
  last_optimized_trajectory.clear();
  last_optimized_trajectory.reserve(n);
  for (int i = 0; i < n; ++i) {
    last_optimized_trajectory.emplace_back(proto.last_optimized_trajectory(i));
  }
  last_plan_start_time = qcraft::FromProto(proto.last_plan_start_time());
}

TrajectoryOptimizerStateProto TrajectoryOptimizerState::ToProto() const {
  TrajectoryOptimizerStateProto proto;

  for (const auto& traj_pt : last_optimized_trajectory) {
    traj_pt.ToProto(proto.add_last_optimized_trajectory());
  }

  qcraft::ToProto(last_plan_start_time, proto.mutable_last_plan_start_time());

  return proto;
}

}  // namespace planner
}  // namespace qcraft
