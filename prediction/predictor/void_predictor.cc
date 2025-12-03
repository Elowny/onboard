
#include "onboard/prediction/predictor/void_predictor.h"

#include <string>
#include <utility>

#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
PredictedTrajectory MakeVoidPrediction(const ObjectMotionHistory& obj_hist) {
  const auto& cur_state = obj_hist.states.back();
  auto void_traj_pts =
      DevelopVoidTrajectory(ObjectMotionStateToUniCycleState(cur_state));
  return PredictedTrajectory(
      /*probability=*/1.0, "Void Prediction", PredictionType::PT_VOID,
      /*index=*/0, std::move(void_traj_pts),
      /*is_reversed=*/false);
}

}  // namespace prediction
}  // namespace qcraft
