#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_

#include <string>
#include <vector>

#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_state.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct TrajectoryOptimizerOutput {
  std::vector<TrajectoryPoint> trajectory;
  std::vector<ApolloTrajectoryPointProto> trajectory_proto;

  std::optional<NudgeOjbectInfo> nudge_object_info;

  // Optimizer Auto Tuning
  AutoTuningTrajectoryProto candidate_auto_tuning_traj_proto;
  AutoTuningTrajectoryProto expert_auto_tuning_traj_proto;

  TrajectoryOptimizerState trajectory_optimizer_state;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_OUTPUT_H_
