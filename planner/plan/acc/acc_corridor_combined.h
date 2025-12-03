#ifndef ONBOARD_PLANNER_PLAN_ACC_ACC_CORRIDOR_COMBINED_H_
#define ONBOARD_PLANNER_PLAN_ACC_ACC_CORRIDOR_COMBINED_H_

#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

namespace internal {
std::vector<const mapping::LaneInfo*> FilterLanes(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::LaneInfo*> start_lane_infos,
    const QtfmEnhancedKdTreeFrenetFrame& frenet_frame, double av_heading);

std::optional<double> ComputeLaneCost(
    const QtfmEnhancedKdTreeFrenetFrame& frenet_frame,
    absl::Span<const Vec2d> points, double credible_length);

absl::StatusOr<mapping::ElementId> FindMostPossibleLane(
    const PlannerSemanticMapManager& psmm,
    const QtfmEnhancedKdTreeFrenetFrame& frenet_frame, const DrivingMapTopo& dm,
    const DiscretizedPath& path, double credible_length);

absl::StatusOr<PathPoint> FindConnectingPathPoint(
    const PlannerSemanticMapManager& psmm, const DiscretizedPath& estimate_path,
    double credible_length, mapping::ElementId target_lane_id, double av_v);

absl::StatusOr<DrivePassage> BuildDrivePassageAndCheckValid(
    const PlannerSemanticMapManager& psmm, mapping::ElementId target_lane_id,
    const PathPoint& connecting_pt, double credible_length);

struct ReferenceCenterPoint {
  double x;
  double y;
  double s;
};

absl::StatusOr<std::vector<ReferenceCenterPoint>>
CombineReferenceCenterFromEstimatedPathAndDp(
    const DiscretizedPath& estimate_path, const DrivePassage& dp,
    const PathPoint& connecting_point, double credible_length,
    double corridor_step_s);
PathSlBoundary BuildPathSlBoundaryWithSmoothedPath(
    const DiscretizedPath& smoothed_path);
}  // namespace internal

absl::StatusOr<AccCorridor> BuildPossiblePreviewLaneKeepAccCorridor(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DiscretizedPath& estimate_path,
    const QtfmEnhancedKdTreeFrenetFrame& estimate_ff, const DrivingMapTopo& dm,
    double credible_length, double corridor_step_s, double total_preview_time);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ACC_ACC_CORRIDOR_COMBINED_H_
