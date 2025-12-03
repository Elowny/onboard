#ifndef ONBOARD_PREDICTION_PREDICTOR_CUTIN_SL_NET_J5_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_CUTIN_SL_NET_J5_PREDICTOR_H_

#include <map>
#include <vector>

#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
struct ObjectCutinSLNetPred {
  std::vector<PredictedTrajectory> pred_trajs;
};
using ObjectsCutinSLNetPredMap = std::map<ObjectIDType, ObjectCutinSLNetPred>;

ObjectsCutinSLNetPredMap MakeCutinSLNetPrediction(
    const PredictionContext& prediction_context,
    const std::vector<ObjectIDType>& cutin_sl_candidate_objs,
    const cutin_sl_net_j5::CutinNetJ5Inferencer& cutin_sl_net_inferencer,
    const ObjectHistorySampler& obj_sampler, MapSampler* map_sampler);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_CUTIN_SL_NET_J5_PREDICTOR_H_
