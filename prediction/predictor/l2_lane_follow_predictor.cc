#include "onboard/prediction/predictor/l2_lane_follow_predictor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/pole_placement_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kMaxHeadingDiff = M_PI / 6.0;  // rad,
constexpr double kMaxBikeAcc = 2.0;             // m/s^2.
constexpr double kMaxBikeBrake = -6.0;          // m/s^2.

absl::StatusOr<PredictedTrajectory> GenerateLaneFollowTrajectory(
    const ObjectMotionState& state, const planner::DrivePassage& dp,
    double dist_tolerance, const std::string& comment) {
  // Compute lateral offset.
  const auto& pos = state.pos;
  ASSIGN_OR_RETURN(const auto sl, dp.QueryUnboundedFrenetCoordinateAt(pos),
                   absl::NotFoundError("SL mapping failed."));
  if (std::fabs(sl.l) > dist_tolerance) {
    return absl::NotFoundError(
        "Not within lateral distance tolerance threshold.");
  }
  if (sl.s < dp.front_s() || sl.s > dp.end_s()) {
    return absl::NotFoundError("Not within longitudinal tolerance threshold.");
  }
  // Compute angle diff.
  ASSIGN_OR_RETURN(const auto tangent_angle, dp.QueryTangentAngleAtS(sl.s),
                   absl::NotFoundError("Angle diff computation failed."));
  const auto heading_diff =
      std::fabs(NormalizeAngle(tangent_angle - state.heading));
  const bool is_same_dir = (heading_diff < kMaxHeadingDiff);
  const bool is_reverse = (heading_diff > M_PI - kMaxHeadingDiff);

  // If agent heading cannot align to the drive passage, return.
  if ((!is_same_dir) && (!is_reverse)) {
    return absl::NotFoundError("Agent heading cannot align to dp.");
  }

  // Compute trajectory.
  const PiecewiseLinearFunction<double, double> t_offset_plf{
      {0.0, kComfortableHorizon}, {sl.l, sl.l}};
  const auto max_steer = kLengthToMaxFrontSteerPlf(state.bbox.length());
  const auto wheelbase = kLengthToWheelbasePlf(state.bbox.length());
  auto generated_points = DevelopConstVelocityPolePlacementTrajectory(
      ObjectMotionStateToBicycleModelState(state), dp, t_offset_plf,
      kPredictionTimeStep, kComfortableHorizon, state.bbox.length(), max_steer,
      wheelbase, is_reverse);
  const bool is_generation_failed =
      (!generated_points.ok() ||
       generated_points->size() <
           static_cast<int>(kEmergencyGuardHorizon / kPredictionTimeStep));
  if (is_generation_failed) {
    return absl::NotFoundError("Trajectory generation failed.");
  }
  std::string annotation = "L2 lane follow: normal " + comment;
  if (is_reverse) {
    annotation += ", reversed lane follow";
  }
  return PredictedTrajectory(/*probability=*/1.0, annotation,
                             PredictionType::PT_L2_LANE_FOLLOW, 1,
                             std::move(generated_points.value()),
                             /*is_reversed=*/false);
}
}  // namespace
PredictedTrajectory MakeAssitDrivingLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context) {
  SCOPED_QTRACE("MakeAssitDrivingLaneFollowPrediction");
  VLOG(2) << "obj_id =" << obj_hist.id;
  const auto& cur_state = obj_hist.states.back();
  const auto& pos = cur_state.pos;

  // 1. Find nearest drive passage.
  auto dps = FindNearbyDrivePassages(
      context.drive_passages(), pos, cur_state.heading, kDefaultLaneWidth,
      /*max_heading_diff=*/std::numeric_limits<double>::max(), /*max_num=*/1);

  // 2.  Try generate a prediction using the nearest drive passage.
  if (!dps.empty()) {
    // 2.1 If we find the nearest drive passage, consider it as the target_dp.
    const auto* target_dp = dps.front();
    auto traj_opt = GenerateLaneFollowTrajectory(
        cur_state, *target_dp, kDefaultLaneWidth, "target_dp");
    if (traj_opt.ok()) {
      return *traj_opt;
    }
  }

  // 3. Compute a CTRA trajectory for later use.
  const double fitted_acc = std::clamp(
      LineFitAccelerationByMotionHistory(obj_hist.states, kAccelerationFitTime),
      kMaxBikeBrake, kMaxBikeAcc);
  auto ctra_state = ObjectMotionStateToUniCycleState(obj_hist.states.back());
  ctra_state.acc = fitted_acc;
  auto ctra_traj_pts = DevelopForwardCTRATrajectory(
      ctra_state, kPredictionTimeStep, kMaintainAccTime, kMaintainAccTime,
      kSafeHorizon);
  auto void_traj_pts =
      DevelopVoidTrajectory(ObjectMotionStateToUniCycleState(cur_state));

  // 4. If we cannot find a satisfactory trajectory, we check agent's relation
  // with AV drive passage
  const auto* av_dp = context.av_drive_passage();

  // 4.1 If no av drive passage, use CTRA.
  if (av_dp == nullptr) {
    return PredictedTrajectory(
        /*probability=*/1.0, "L2 lane follow: CTRA (no av drive passage)",
        PredictionType::PT_L2_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false);
  }

  // 4.2 If far away from AV drive passage, ignore.
  const auto sl_or = av_dp->QueryLaterallyUnboundedFrenetCoordinateAt(pos);
  const auto av_relation_lat_limit = 2.5 * kDefaultLaneWidth;
  if ((!sl_or.ok()) || (std::fabs(sl_or->l) > av_relation_lat_limit)) {
    return PredictedTrajectory(
        /*probability=*/1.0, "L2 lane follow: away from AV drive passage",
        PredictionType::PT_VOID, 0, std::move(void_traj_pts),
        /*is_reversed=*/false);
  }

  // 4.3 If close enough to AV and heading diff is small.
  auto traj_opt = GenerateLaneFollowTrajectory(cur_state, *av_dp,
                                               av_relation_lat_limit, "av_dp");
  if (traj_opt.ok()) {
    return *traj_opt;
  }

  // 4.4 If no trajectory generated, use CTRA.
  return PredictedTrajectory(
      /*probability=*/1.0,
      "L2 lane follow:  large heading diff with av drive passage.",
      PredictionType::PT_CTRA, 0, std::move(ctra_traj_pts),
      /*is_reversed=*/false);
}
}  // namespace prediction
}  // namespace qcraft
