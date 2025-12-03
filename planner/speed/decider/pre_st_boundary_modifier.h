#ifndef ONBOARD_PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"

namespace qcraft {
namespace planner {

struct PreStboundaryModifierInput {
  const SpeedFinderParamsProto::StBoundaryPreModifierParamsProto* params =
      nullptr;
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  double current_v = 0.0;
  double current_a = 0.0;
  const DiscretizedPath* path = nullptr;
};

// Modify prediction according to st-boundary overlap info prior to speed
// decision, and generate new st-boundaries. If a prediction trajectory is
// modified in this stage, it won't be modified in the following speed
// decision.

void PreModifyStBoundaries(
    const PreStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_PRE_ST_BOUNDARY_MODIFIER_H_
