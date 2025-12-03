#ifndef ONBOARD_PREDICTION_SCHEDULER_PRIORITY_ANALYZER_H_
#define ONBOARD_PREDICTION_SCHEDULER_PRIORITY_ANALYZER_H_
#include <map>
#include <string>

#include "absl/types/span.h"

#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"

namespace qcraft {
namespace prediction {
struct ObjectPredictionPriorityInfo {
  ObjectPredictionPriority priority;
  std::string priority_annotation;
};

std::map<ObjectIDType, ObjectPredictionPriorityInfo> AnalyzePriorities(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& obj_scenarios);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_SCHEDULER_PRIORITY_ANALYZER_H_
