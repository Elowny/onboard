
#ifndef ONBOARD_PREDICTION_UTIL_SAFE_GUARD_UTIL_H_
#define ONBOARD_PREDICTION_UTIL_SAFE_GUARD_UTIL_H_
#include <optional>

#include "absl/types/span.h"

#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
std::optional<PredictedTrajectory> CreateSafeGuardBrakeTrajectory(
    const ObjectMotionHistory& object_motion_history,
    absl::Span<const PredictedTrajectory> prev_trajs, double perception_acc,
    double av_v, PredictionType pred_type);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_UTIL_SAFE_GUARD_UTIL_H_
