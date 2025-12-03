#ifndef ONBOARD_PREDICTION_LANE_FOLLOW_UTIL_H_
#define ONBOARD_PREDICTION_LANE_FOLLOW_UTIL_H_

#include <string>  // for string
#include <vector>  // for vector

#include "absl/status/statusor.h"  // for StatusOr
#include "absl/types/span.h"       // for Span

#include "onboard/planner/router/drive_passage.h"  // for DrivePassage
#include "onboard/prediction/predicted_trajectory.h"  // for PredictedTrajectoryPoint
#include "onboard/prediction/prediction_defs.h"       // for ObjectMotionState
#include "onboard/proto/perception/fusion/object.pb.h"  // for ObjectType

namespace qcraft {
namespace prediction {
absl::StatusOr<double> CalculateLaneFollowTargetL(
    absl::Span<const ObjectMotionState> hist, const planner::DrivePassage& dp,
    const ObjectType& obj_type, bool is_in_intersection);

std::string ExtendLaneFollowTraj(
    const planner::DrivePassage& dp, const bool is_mapless, double pred_horizon,
    std::vector<PredictedTrajectoryPoint>* traj_pts);

absl::StatusOr<double> CalcTargetLForObjectCrossingBoundary(
    const ObjectMotionState& cur_state, double cur_lat_speed, double lane_min_l,
    double lane_max_l, const planner::DrivePassage& dp,
    const ObjectType& obj_type, double target_l, bool is_hd_map,
    bool is_slow_cutin);

bool IsObjectApproachingDrivePassage(const planner::DrivePassage& dp,
                                     const ObjectMotionState& cur_state,
                                     const ObjectType& obj_type,
                                     double lane_min_l, double lane_max_l,
                                     double cur_lat_speed);

bool IsSlowCutinObj(const planner::DrivePassage& dp,
                    const ObjectMotionState& cur_state);
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_LANE_FOLLOW_UTIL_H_
