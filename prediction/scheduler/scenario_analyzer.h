#ifndef ONBOARD_PREDICTION_SCHEDULER_SCENARIO_ANALYZER_H_
#define ONBOARD_PREDICTION_SCHEDULER_SCENARIO_ANALYZER_H_
#include <map>

#include "absl/types/span.h"

#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
std::map<ObjectIDType, ObjectPredictionScenario> AnalyzeScenarios(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict);
ObjectPredictionScenario AnalyzeScenarioWithSemanticMapAndObjectProto(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const ObjectProto& obj_proto);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_SCHEDULER_SCENARIO_ANALYZER_H_
