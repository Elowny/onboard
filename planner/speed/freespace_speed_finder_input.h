#ifndef ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_INPUT_H_
#define ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_INPUT_H_

#include <string>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

// TODO(ping): Delete this file after refactoring speed finder.
struct FreespaceSpeedFinderInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager;
  const SpacetimeTrajectoryManager* obj_mgr;
  const PlannerClusterObjectManager* cluster_obj_mgr;
  const ConstraintManager* constraint_mgr;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const absl::flat_hash_set<PlannerClusterObject::Id>* stalled_cluster_objects;
  const DiscretizedPath* path;
  bool forward;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double plan_start_j = 0.0;
  absl::Time plan_time;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_INPUT_H_
