#ifndef ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_INPUT_H_
#define ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_INPUT_H_

#include <set>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"

#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/object_conflict_manager.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::prediction {
// Define input structures for each submodules of conflict resolver.

struct ObjectConflictResolverInput {
  std::string traj_id;
  const planner::PlannerSemanticMapManager* semantic_map_mgr = nullptr;
  const ObjectConflictManager* obj_con_mgr = nullptr;
  const std::set<mapping::ElementId>* red_tls = nullptr;
  const ConflictResolverParams* params = nullptr;
};
}  // namespace qcraft::prediction
#endif  // ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_INPUT_H_
