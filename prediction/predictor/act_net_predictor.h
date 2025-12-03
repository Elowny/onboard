#ifndef ONBOARD_PREDICTION_PREDICTOR_ACT_NET_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_ACT_NET_PREDICTOR_H_
#include "absl/types/span.h"  // for Span

#include "onboard/async/thread_pool.h"                    // for ThreadPool
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/inferencer/act_net_inferencer.h"
#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"
#include "onboard/prediction/predicted_trajectory.h"  // for ObjectsActNetPredMap
namespace qcraft {
namespace prediction {
ObjectsActNetPredMap MakeActNetPrediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const actnet::ActNetInferencer* act_net_inferencer_ptr,
    const cutin_sl_net::CutinSLNetInferencer* cutin_sl_inferencer_ptr,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_ACT_NET_PREDICTOR_H_
