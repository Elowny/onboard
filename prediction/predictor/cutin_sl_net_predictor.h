#ifndef ONBOARD_PREDICTION_PREDICTOR_CUTIN_SL_NET_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_CUTIN_SL_NET_PREDICTOR_H_

#include <map>
#include <optional>
#include <vector>

#include "absl/types/span.h"  // for Span

#include "onboard/async/thread_pool.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
struct ObjectCutinSLNetPred {
  std::vector<PredictedTrajectory> pred_trajs;
};
using ObjectsCutinSLNetPredMap = std::map<ObjectIDType, ObjectCutinSLNetPred>;
std::optional<ObjectsCutinSLNetPredMap> MakeCutinSLNetPrediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const cutin_sl_net::CutinSLNetInferencer& cutin_sl_net_inferencer,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler,
    ThreadPool* thread_pool);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_CUTIN_SL_NET_PREDICTOR_H_
