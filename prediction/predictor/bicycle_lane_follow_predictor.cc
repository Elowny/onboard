
#include "onboard/prediction/predictor/bicycle_lane_follow_predictor.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/drive_passage_util.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/pole_placement_util.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kMaxHeadingDiff = M_PI / 3.0;        // rad,
constexpr double kSafeGuardHeadingDiff = M_PI / 4.0;  // rad,
constexpr double kMaxDistanceLimit = 1.5;             // m.
constexpr double kMinPathLength = 20.0;               // m.
constexpr double kBackwardLength = 10.0;              // m.
constexpr double kPathStepS = 4.0;                    // m.
constexpr double kMaxBikeAcc = 2.0;                   // m/s^2.
constexpr double kMaxBikeBrake = -6.0;                // m/s^2.

std::vector<PredictedTrajectory> DevelopBicycleLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    bool ignore_off_road) {
  const auto& cur_state = obj_hist.states.back();
  const auto& pos = cur_state.pos;
  const auto& semantic_map_mgr = context.semantic_map_manager();

  const auto lane_id_or = FindNearestLaneIdWithBoundaryDistanceLimit(
      semantic_map_mgr, pos, kMaxDistanceLimit);

  const double fitted_acc = std::clamp(
      LineFitAccelerationByMotionHistory(obj_hist.states, kAccelerationFitTime),
      kMaxBikeBrake, kMaxBikeAcc);
  auto ctra_state = ObjectMotionStateToUniCycleState(obj_hist.states.back());
  ctra_state.acc = fitted_acc;
  auto ctra_traj_pts = DevelopForwardCTRATrajectory(
      ctra_state, kPredictionTimeStep, kMaintainAccTime, kMaintainAccTime,
      kSafeHorizon);

  // No lane association, use a simple CTRA prediction.
  if (lane_id_or == std::nullopt) {
    if (ignore_off_road) {
      auto void_traj_pts =
          DevelopVoidTrajectory(ObjectMotionStateToUniCycleState(cur_state));
      return {PredictedTrajectory(
          /*probability=*/1.0, "Bike lane follow: VOID",
          PredictionType::PT_VOID, 0, std::move(void_traj_pts),
          /*is_reversed=*/false)};
    } else {
      return {PredictedTrajectory(
          /*probability=*/1.0, "Bike lane follow: CTRA (no nearest lane)",
          PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
          /*is_reversed=*/false)};
    }
  }

  ASSIGN_OR_DIE(
      const auto closest_lane_pt,
      FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          semantic_map_mgr.GetLevel(), semantic_map_mgr, pos,
          std::vector<mapping::ElementId>({*lane_id_or}), /*heading=*/0.0,
          /*heading_penalty_weight=*/0.0));
  double lane_heading =
      ComputeLanePointLerpTheta(semantic_map_mgr, closest_lane_pt);
  const double angle_diff =
      std::fabs(NormalizeAngle(cur_state.heading - lane_heading));
  // Almost perpendicular to lane, do not associate them, Use a CTRA prediction
  if (angle_diff > kMaxHeadingDiff && angle_diff < M_PI - kMaxHeadingDiff) {
    return {PredictedTrajectory(
        /*probability=*/1.0, "Bike lane follow: CTRA (large heading error)",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }

  // Check if obj is reversed driving or need a safe guard CTRA prediction
  bool is_reversed_driving = false;
  bool need_ctra = false;
  if (angle_diff > M_PI - kMaxHeadingDiff) {
    is_reversed_driving = true;
  }
  if (angle_diff > kSafeGuardHeadingDiff &&
      angle_diff < M_PI - kSafeGuardHeadingDiff) {
    need_ctra = true;
  }
  const auto lane_id = lane_id_or.value();

  const auto cur_v = cur_state.vel.norm();
  const double forward_len =
      std::max(kPredictionDuration * cur_v, kMinPathLength);
  // only consider most straight path.
  auto lane_path = SearchMostStraightLanePath(pos, semantic_map_mgr, lane_id,
                                              forward_len, is_reversed_driving);
  mapping::LanePath extended_lp;
  if (is_reversed_driving) {
    extended_lp = planner::BackwardExtendLanePath(semantic_map_mgr, lane_path,
                                                  kBackwardLength);
  } else {
    extended_lp = planner::ForwardExtendLanePath(semantic_map_mgr, lane_path,
                                                 kBackwardLength);
  }

  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          semantic_map_mgr.GetLevel(), semantic_map_mgr, cur_state.pos,
          extended_lp, cur_state.heading);
  // The projection must be valid.
  QCHECK_OK(closest_lane_point_or.status());

  mapping::LanePath pruned_lane_path;
  if (is_reversed_driving) {
    pruned_lane_path = std::move(extended_lp);
  } else {
    pruned_lane_path = PruneLanePathByLength(
        semantic_map_mgr, extended_lp, *closest_lane_point_or,
        kObjectDrivePassageFrontLength, kObjectDrivePassageBackLength);
  }

  const auto dp = planner::BuildDrivePassageForPredictionWithLaneBoundaryCache(
      semantic_map_mgr, pruned_lane_path, context.lane_boundary_cache(),
      kPathStepS,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0);

  if (!dp.ok()) {
    return {PredictedTrajectory(
        /*probability=*/1.0, "Bike lane follow: CTRA (not valid drive passage)",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }
  const auto sl_or = (*dp).QueryFrenetCoordinateAt(pos);
  double lateral_offset = 0.0;
  if (sl_or.ok()) {
    lateral_offset = sl_or->l;
  }
  const auto max_steer = kLengthToMaxFrontSteerPlf(cur_state.bbox.length());
  const auto wheelbase = kLengthToWheelbasePlf(cur_state.bbox.length());
  const PiecewiseLinearFunction<double, double> t_offset_plf{
      {0.0, kComfortableHorizon}, {lateral_offset, lateral_offset}};
  auto generated_points = DevelopConstVelocityPolePlacementTrajectory(
      ObjectMotionStateToBicycleModelState(cur_state), *dp, t_offset_plf,
      kPredictionTimeStep, kComfortableHorizon, cur_state.bbox.length(),
      max_steer, wheelbase, is_reversed_driving);
  if (!generated_points.ok() ||
      generated_points->size() <
          static_cast<int>(kEmergencyGuardHorizon / kPredictionTimeStep)) {
    const auto ctra_reason = generated_points.ok()
                                 ? "not enough pure pursuit points"
                                 : generated_points.status().message();
    return {PredictedTrajectory(
        /*probability=*/1.0,
        absl::StrFormat("Bike lane follow: CTRA (%s)", ctra_reason),
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }

  std::vector<PredictedTrajectory> trajs;
  trajs.reserve(2);
  double prob = 1.0;
  if (need_ctra) {
    prob = 0.5;
    trajs.push_back(PredictedTrajectory(
        prob, "Bike lane follow: CTRA for safety",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false));
  }
  trajs.push_back(PredictedTrajectory(prob, "Bike lane follow: normal",
                                      PredictionType::PT_BIKE_LANE_FOLLOW, 1,
                                      std::move(generated_points.value()),
                                      /*is_reversed=*/false));
  return trajs;
}

}  // namespace
std::vector<PredictedTrajectory> MakeBicycleLaneFollowPrediction(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    bool ignore_off_road) {
  SCOPED_QTRACE("MakeBicycleLaneFollowPrediction");
  VLOG(2) << "bicycle_id =" << obj_hist.id;
  auto trajs =
      DevelopBicycleLaneFollowPrediction(obj_hist, context, ignore_off_road);
  return trajs;
}
}  // namespace prediction
}  // namespace qcraft
