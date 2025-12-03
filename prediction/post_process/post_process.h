#ifndef ONBOARD_PREDICTION_POST_PROCESS_POST_PROCESS_H_
#define ONBOARD_PREDICTION_POST_PROCESS_POST_PROCESS_H_

#include <map>
#include <set>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/post_process_defs.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
void RunCutoffByCurbPostProcess(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const ConflictResolverParams& resolver_params,
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions,
    ThreadPool* thread_pool);

absl::Status RunPredictionPostProcess(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const std::set<mapping::ElementId>& red_tls,
    const ConflictResolverParams& resolver_params,
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions,
    ConflictResolverDebugProto* debug_proto, ThreadPool* thread_pool);

// Find all red traffic lights.
std::set<mapping::ElementId> FindRedTrafficLights(
    const planner::PlannerSemanticMapManager& psmm,
    const TrafficLightStatesProto& tl_states);

// Util functions to do type conversion.
std::map<ObjectIDType, ObjectPredictionPostProcess> FromObjectPredictionResults(
    std::map<ObjectIDType, ObjectPredictionResult>* mutable_obj_pred_results);

std::map<ObjectIDType, ObjectPredictionPostProcess> FromObjectPredictions(
    absl::Span<ObjectPrediction> mutable_obj_preds);

}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_POST_PROCESS_POST_PROCESS_H_
