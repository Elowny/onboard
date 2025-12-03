#ifndef ONBOARD_PREDICTION_PREDICTOR_VEHICLE_LANE_FOLLOW_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_VEHICLE_LANE_FOLLOW_PREDICTOR_H_

#include <vector>

#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"

namespace qcraft {
namespace prediction {
std::vector<PredictedTrajectory> MakeVehicleLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    const ObjectPredictionScenario& scenario, bool ignore_off_road);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_VEHICLE_LANE_FOLLOW_PREDICTOR_H_
