#ifndef ONBOARD_PREDICTION_PREDICTOR_ACT_NET_J5_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_ACT_NET_J5_PREDICTOR_H_

#include "absl/types/span.h"  // for Span

#include "onboard/async/thread_pool.h"                    // for ThreadPool
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/act_net_j5_inferencer.h"  // for ActNetInferencer
#include "onboard/prediction/net/horizon/act_net_local_j5_inferencer.h"
#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"
#include "onboard/prediction/predicted_trajectory.h"  // for PredictedTrajectory

namespace qcraft {
namespace prediction {

ObjectsActNetPredMap MakeActNetJ5Prediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const actnetj5::ActNetJ5Inferencer& act_net_j5_inferencer,
    const cutin_sl_net_j5::CutinNetJ5Inferencer* cutin_sl_inferencer_ptr,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool);

ObjectsActNetPredMap MakeActNetLocalJ5Prediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const actnetlocalj5::ActNetLocalJ5Inferencer& act_net_local_j5_inferencer,
    const cutin_sl_net_j5::CutinNetJ5Inferencer* cutin_sl_inferencer_ptr,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_ACT_NET_J5_PREDICTOR_H_
