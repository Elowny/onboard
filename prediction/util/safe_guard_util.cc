
#include "onboard/prediction/util/safe_guard_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "glog/logging.h"  // for COMPACT_GOOGLE_LOG_INFO, LogMessage, VLOG

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {
// Safe guard brake trajectory
constexpr double kEnableBrakeAccThreshold = -1.0;  // m/s^2.
const PiecewiseLinearFunction<double, double> kEnableBrakeAccThreshold2Plf(
    std::vector<double>{2.0, 2.5, 4.0},
    std::vector<double>{0.0, -0.2, -0.5});  // obj_v : AccThreshold.

const PiecewiseLinearFunction<double, double> kEnableBrakeSpeedDiffPlf(
    std::vector<double>{2.0, 4.0},
    std::vector<double>{1.0, 1.5});        // obj_v : SpeedDiff.
constexpr double kMaxLineFitBrake = -3.0;  // m/s^2.
constexpr double kTargetTime = 2.5;        // s.
const PiecewiseLinearFunction<double, double> kSafeGuardBrakeMinSpeedPlf(
    std::vector<double>{3.0, 5.0, 10.0, 20.0, 40.0},
    std::vector<double>{0.0, 1.0, 2.0, 6.0, 20.0});
constexpr double kDefaultProbForBrakingTrajectory = 0.4;
constexpr double kHardBrakeFlagThreshold = -2.0;

}  // namespace

std::optional<PredictedTrajectory> CreateSafeGuardBrakeTrajectory(
    const ObjectMotionHistory& object_motion_history,
    absl::Span<const PredictedTrajectory> prev_trajs, double perception_acc,
    double av_v, PredictionType pred_type) {
  const auto obj_v = object_motion_history.states.back().vel.norm();
  const auto& obj_id = object_motion_history.id;
  double fitted_acc =
      std::max(kMaxLineFitBrake,
               LineFitAccelerationByMotionHistory(object_motion_history.states,
                                                  kAccelerationFitTime));
  // Consider perception acc.
  if (FLAGS_only_use_perception_acc) {
    fitted_acc = perception_acc;
  } else {
    fitted_acc = std::min(perception_acc, fitted_acc);
  }
  const double target_speed = std::max(obj_v + fitted_acc * kTargetTime,
                                       kSafeGuardBrakeMinSpeedPlf(obj_v));
  std::vector<double> traj_speeds;
  traj_speeds.reserve(prev_trajs.size());
  for (const auto& prev_traj : prev_trajs) {
    const auto& traj_pts = prev_traj.points();
    QCHECK_GT(traj_pts.size(), 0);
    const int target_idx =
        std::min(static_cast<int>(traj_pts.size()) - 1,
                 static_cast<int>(kTargetTime / kPredictionTimeStep));
    traj_speeds.push_back(traj_pts[target_idx].v());
  }
  if (traj_speeds.size() <= 0) return std::nullopt;
  const double min_traj_speed =
      *std::min_element(traj_speeds.begin(), traj_speeds.end());
  const double enable_brake_acc_threshold2 =
      kEnableBrakeAccThreshold2Plf(obj_v);

  if ((fitted_acc < kEnableBrakeAccThreshold &&
       min_traj_speed > target_speed) ||
      (fitted_acc < enable_brake_acc_threshold2 &&
       (min_traj_speed - target_speed) > kEnableBrakeSpeedDiffPlf(obj_v))) {
    QEVENT_EVERY_N("runlin", "actnet_predicted_traj_insufficient_hard_braking",
                   10, [&](QEvent* qevent) {
                     qevent->AddField("object_id", obj_id)
                         .AddField("fitted_acc", fitted_acc);
                   });
    const auto& prev_traj = prev_trajs.back();
    const auto& traj_pts = prev_traj.points();
    const auto path = PredictedTrajectoryPointsToDiscretizedPath(traj_pts);
    const auto speed_vec = DevelopConstAccSpeedProfile(
        obj_v, fitted_acc, kTargetTime, target_speed, kPredictionTimeStep,
        kPredictionDuration);
    auto new_traj_pts =
        CombinePathAndSpeedForPredictedTrajectoryPoints(path, speed_vec);
    // Avoid the case that newly generated trajectory has less points than
    // old one.
    if (new_traj_pts.size() >= traj_pts.size()) {
      std::string annotation =
          "Brake safeguard trajectory, modified to brake with " +
          std::to_string(fitted_acc) + " to " + std::to_string(target_speed);
      const int idx = static_cast<int>(prev_trajs.size());
      const bool is_hard_braking =
          (fitted_acc < kHardBrakeFlagThreshold) && (obj_v < av_v);
      VLOG(2) << "obj id " << obj_id << " idx " << idx << " acc " << fitted_acc
              << " av v " << av_v << " obj v " << obj_v << " is hard braking "
              << is_hard_braking;
      PredictedTrajectory pred_traj(
          /*probability=*/kDefaultProbForBrakingTrajectory, annotation,
          pred_type, idx, new_traj_pts,
          /*is_reversed=*/false, is_hard_braking);
      pred_traj.set_rot_rad(prev_traj.rot_rad());  // For calculation
                                                   // of uncertatiny ellipse.
      return pred_traj;
    }
  }
  return std::nullopt;
}

}  // namespace prediction
}  // namespace qcraft
