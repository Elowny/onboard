#ifndef ONBOARD_PREDICTION_PREDICTOR_KINEMATIC_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_KINEMATIC_PREDICTOR_H_

#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
PredictedTrajectory MakeStationaryPrediction(
    const ObjectMotionHistory& obj_hist, double prediction_horizon);

PredictedTrajectory MakeCYCVPrediction(const ObjectMotionHistory& obj_hist,
                                       double prediction_horizon);

PredictedTrajectory MakeReverseCYCVPrediction(
    const ObjectMotionHistory& obj_hist, double prediction_horizon);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_KINEMATIC_PREDICTOR_H_
