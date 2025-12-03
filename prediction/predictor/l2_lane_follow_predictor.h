#ifndef ONBOARD_PREDICTION_PREDICTOR_L2_LANE_FOLLOW_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_L2_LANE_FOLLOW_PREDICTOR_H_

#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
PredictedTrajectory MakeAssitDrivingLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_L2_LANE_FOLLOW_PREDICTOR_H_
