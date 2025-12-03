#ifndef ONBOARD_PREDICTION_PREDICTOR_HEURISTIC_PEDESTRIAN_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_HEURISTIC_PEDESTRIAN_PREDICTOR_H_

#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
PredictedTrajectory MakeHeuristicPedestrianPrediction(
    const ObjectMotionHistory& obj_hist);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_HEURISTIC_PEDESTRIAN_PREDICTOR_H_
