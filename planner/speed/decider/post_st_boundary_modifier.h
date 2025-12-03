#ifndef ONBOARD_PLANNER_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/decider/st_boundary_modifier_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"

namespace qcraft::planner {

// TODO(yumeng): rename to StBoundaryLonModificationInfo.
struct StBoundaryModificationInfo {
  StBoundaryModifierProto::ModifierType modifier_type =
      StBoundaryModifierProto::UNKNOWN;
  StBoundaryProto::DecisionType decision = StBoundaryProto::UNKNOWN;
  bool is_decision_changed = false;
  // New trajectory accelerate at 'a' from 't'.
  std::vector<AccelPoint> accel_point_list;
};

struct PostStboundaryModifierInput {
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const absl::flat_hash_map<std::string, StBoundaryModificationInfo>*
      modification_info_map = nullptr;
};

void PostModifyStBoundaries(
    const PostStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_POST_ST_BOUNDARY_MODIFIER_H_
