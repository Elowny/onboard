#ifndef ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_H_
#define ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_H_
#include <map>

#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
std::map<ObjectIDType, ObjectPredictionResult> SchedulePrediction(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool, PredictionDebugProto* debug);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_H_
