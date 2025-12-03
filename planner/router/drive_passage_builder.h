#ifndef ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_
#define ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_

#include <memory>
#include <optional>

// IWYU pragma: no_include "absl/hash/hash.h"

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {
using LaneBoundaryCache =
    absl::flat_hash_map<mapping::ElementId,
                        PiecewiseLinearFunction<PredictionLaneBoundary, double,
                                                PredictionLaneBoundaryLerper>>;
// This is the builder function for est planner.
absl::StatusOr<DrivePassage> BuildDrivePassage(
    const PlannerSemanticMapManager& psmm,
    const std::shared_ptr<PlannerSemanticMapManager>& vision_map_ptr,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePath& backward_extended_lane_path,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    const mapping::LanePoint& destination, bool all_lanes_virtual = false,
    std::optional<double> override_speed_limit_mps = std::nullopt,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree);

absl::StatusOr<DrivePassage> BuildDrivePassageFromLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, double backward_extend_len,
    double required_planning_horizon, double required_backward_len,
    std::optional<double> override_speed_limit_mps,
    FrenetFrameType type = FrenetFrameType::kQtfmKdTree,
    bool smooth_lane_path = false);
// TODO(weijun): do not use default value.

absl::StatusOr<DrivePassage> BuildDrivePassageForPrediction(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step_s, bool avoid_loop, double backward_extend_len,
    double max_lateral_boundary,
    FrenetFrameType type = FrenetFrameType::kBruteFroce);

absl::StatusOr<DrivePassage>
BuildDrivePassageForPredictionWithLaneBoundaryCache(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const LaneBoundaryCache& lane_boundary_cache, double step_s,
    bool avoid_loop, double backward_extend_len,
    FrenetFrameType type = FrenetFrameType::kBruteFroce);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_
