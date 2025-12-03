#ifndef ONBOARD_PLANNER_ML_CONDITION_FEATURE_CONDITION_FEATURE_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONDITION_FEATURE_CONDITION_FEATURE_EXTRACTOR_H_  // NOLINT

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/ml/condition_feature_extractor/condition_feature.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace ml {

inline constexpr double kPathBoundarySampleLen = 5.0;  // m.
inline constexpr int kMapLanePointsNum = 20;  // Max points num in a lane.

absl::StatusOr<TrajectoryFeature> ExtractTrajectoryFeature(
    const ContextFeature& context_feature,
    const std::vector<ApolloTrajectoryPointProto>& traj, double start_ts);

absl::StatusOr<LineSegmentsFeature> ExtractLanePathFeature(
    const ContextFeature& context_feature,
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path);

absl::StatusOr<MultiLanePathFeature> ExtractMultiLanePathFeature(
    const ContextFeature& context_feature,
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::LanePath* const> lane_paths, int max_lane_size);

absl::StatusOr<PathBoundaryFeature> ExtractPathBoundaryFeature(
    const ContextFeature& context_feature,
    const PathSlBoundary& path_sl_boundary);

absl::StatusOr<LineSegmentsFeature> ExtractRefPathFeature(
    const ContextFeature& context_feature,
    const PathSlBoundary& path_sl_boundary);

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONDITION_FEATURE_CONDITION_FEATURE_EXTRACTOR_H_
