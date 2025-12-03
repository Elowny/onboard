
#ifndef ONBOARD_PREDICTION_UTIL_TRAJECTORY_DEVELOPER_H_
#define ONBOARD_PREDICTION_UTIL_TRAJECTORY_DEVELOPER_H_
#include <vector>

#include "absl/types/span.h"

#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/kinematic_model.h"
namespace qcraft {
namespace prediction {

std::vector<PredictedTrajectoryPoint> DevelopStaticTrajectory(
    const UniCycleState& state, double dt, double horizon);

std::vector<PredictedTrajectoryPoint> DevelopVoidTrajectory(
    const UniCycleState& state);

// Const Turn Rate and Acceleration (CTRA). For now, only for forward maneuver.
// Stop time: the moment which we stop using yaw rate or acceleration value.
std::vector<PredictedTrajectoryPoint> DevelopForwardCTRATrajectory(
    const UniCycleState& state, double dt, double acc_stop_time,
    double yaw_rate_stop_time, double horizon);

// Const Yaw Const Velocity (CYCV)
std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const UniCycleState& state, double dt, double horizon, bool is_reversed);

std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const PredictionObject& obj, double dt, double horizon, bool is_reversed);

planner::SpeedVector DevelopConstAccSpeedProfile(double cur_speed, double acc,
                                                 double stop_time,
                                                 double min_speed, double dt,
                                                 double horizon);

std::vector<PredictedTrajectoryPoint>
CombinePathAndSpeedForPredictedTrajectoryPoints(
    const planner::DiscretizedPath& path,
    const planner::SpeedVector& speed_data);

double LineFitLateralSpeedByMotionHistory(
    absl::Span<const ObjectMotionState> history,
    const planner::DrivePassage& dp, double fit_time, bool clamp_by_lane_width);

double LineFitAccelerationByMotionHistory(
    absl::Span<const ObjectMotionState> history, double fit_time);

std::vector<ObjectMotionState> CutMotionStateBySpeedDiff(
    const std::vector<ObjectMotionState>& history);

planner::DiscretizedPath PredictedTrajectoryPointsToDiscretizedPath(
    absl::Span<const PredictedTrajectoryPoint> points);

void ExtendTrajectoryAlongTangent(
    double horizon, const Vec2d& tangent,
    std::vector<PredictedTrajectoryPoint>* traj_pts);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_UTIL_TRAJECTORY_DEVELOPER_H_
