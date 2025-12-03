
#include "onboard/prediction/predictor/kinematic_predictor.h"

#include <string>
#include <utility>

#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
PredictedTrajectory MakeStationaryPrediction(
    const ObjectMotionHistory& obj_hist, double prediction_horizon) {
  const auto& cur_state = obj_hist.states.back();
  auto traj_pts =
      DevelopStaticTrajectory(ObjectMotionStateToUniCycleState(cur_state),
                              kPredictionTimeStep, prediction_horizon);
  return PredictedTrajectory(
      /*probability=*/1.0, "Stationary Prediction",
      PredictionType::PT_STATIONARY, /*index=*/0, std::move(traj_pts),
      /*is_reversed=*/false);
}

PredictedTrajectory MakeCYCVPrediction(const ObjectMotionHistory& obj_hist,
                                       double prediction_horizon) {
  const auto& cur_state = obj_hist.states.back();
  auto traj_pts =
      DevelopCYCVTrajectory(ObjectMotionStateToUniCycleState(cur_state),
                            kPredictionTimeStep, prediction_horizon,
                            /*is_reversed=*/false);
  PredictedTrajectory predicted_traj = PredictedTrajectory(
      /*probability=*/1.0, "CVCY Prediction", PredictionType::PT_CYCV,
      /*index=*/0, std::move(traj_pts),
      /*is_reversed=*/false);
  return predicted_traj;
}

PredictedTrajectory MakeReverseCYCVPrediction(
    const ObjectMotionHistory& obj_hist, double prediction_horizon) {
  const auto& cur_state = obj_hist.states.back();
  auto traj_pts =
      DevelopCYCVTrajectory(ObjectMotionStateToUniCycleState(cur_state),
                            kPredictionTimeStep, prediction_horizon,
                            /*is_reversed=*/true);
  PredictedTrajectory predicted_traj;
  predicted_traj = PredictedTrajectory(
      /*probability=*/1.0, "Reverse CVCY Prediction",
      PredictionType::PT_REVERSE_CYCV, /*index=*/0, std::move(traj_pts),
      /*is_reversed=*/true);
  return predicted_traj;
}

}  // namespace prediction
}  // namespace qcraft
