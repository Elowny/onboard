#ifndef ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_H_
#define ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_H_

#include <map>
#include <set>

#include "onboard/async/thread_pool.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/post_process_defs.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"

namespace qcraft {
namespace prediction {
void ResolveConflict(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const std::set<mapping::ElementId>& red_tls,
    const ConflictResolverParams& resolver_params,
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions,
    ConflictResolverDebugProto* debug_proto, ThreadPool* thread_pool);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_H_
