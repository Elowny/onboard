#ifndef ONBOARD_PREDICTION_UTIL_POLE_PLACEMENT_UTIL_H_
#define ONBOARD_PREDICTION_UTIL_POLE_PLACEMENT_UTIL_H_
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/util/kinematic_model.h"

namespace qcraft {
namespace prediction {
enum class PolePlacementFeedForwardType {
  CURVATURE = 0,     // NOLINT
  HEADING_DIFF = 1,  // NOLINT
};
// A common util to build pole placement trajectory from a set of references.
// Desired number of points is the size of ref_v.
// ref_pts.size() must equal to ref_ks.size() & ref_headings.size(), but they
// can differ from ref_v.size().
std::vector<PredictedTrajectoryPoint> DevelopPolePlacementTrajectory(
    absl::Span<const Vec2d> ref_pts, absl::Span<const double> ref_ks,
    absl::Span<const double> ref_headings, absl::Span<const double> ref_v,
    const BicycleModelState& state, double dt, double obj_len, double max_steer,
    double wheelbase, PolePlacementFeedForwardType ff_type);

absl::StatusOr<std::vector<PredictedTrajectoryPoint>>
DevelopConstVelocityPolePlacementTrajectory(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    const PiecewiseLinearFunction<double, double>& s_offset_plf, double dt,
    double horizon, double obj_len, double max_steer, double wheelbase,
    bool is_reverse_driving);

absl::StatusOr<planner::DiscretizedPath> DevelopPolePlacementPath(
    const BicycleModelState& state, const planner::DrivePassage& dp, double dt,
    double horizon, double lane_offset, double obj_len, double max_steer,
    double wheelbase);

absl::StatusOr<planner::DiscretizedPath>
DevelopPolePlacementPathWithSmoothConvergence(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    const PiecewiseLinearFunction<double, double>& t_offset_plf, double dt,
    double horizon, double obj_len, double max_steer, double wheelbase);

std::vector<PredictedTrajectoryPoint> TrackTrajectoryByPolePlacement(
    absl::Span<const PredictedTrajectoryPoint> ref_pts,
    const BicycleModelState& state, double obj_len, double max_steer,
    double wheelbase, double dt, double horizon);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_UTIL_POLE_PLACEMENT_UTIL_H_
