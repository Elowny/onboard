#ifndef ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_WITH_MAP_H_
#define ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_WITH_MAP_H_

#include <complex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

namespace internal {
absl::Status FindClosestLanePointToSmoothPointOnLane(
    const PlannerSemanticMapManager& psmm, const Vec2d& query_point,
    const mapping::ElementId& lane_id, std::optional<double> max_allow_distance,
    mapping::LanePoint* lane_point, Vec2d* pos, Vec2d* tangent,
    double* signed_distance);

double EstimateMinDrivePassageDivergeLength(const DrivePassage& dp_base,
                                            const DrivePassage& dp_target,
                                            const Vec2d& start_point);

double ComputeDrivePassageCost(const DrivePassage& dp,
                               absl::Span<const UniCycleState> probing_traj);

std::vector<UniCycleState> GenerateProbingTrajs(
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
    double ds, double desire_length);

void GetLaneBoundaryPointsAndWidth(const PlannerSemanticMapManager& psmm,
                                   const mapping::ElementId& lane_id,
                                   const Vec2d& pos, const Vec2d& tangent,
                                   Vec2d* right, Vec2d* left, double* width);

Box2d BuildLaneBoxApproxByProjection(const PlannerSemanticMapManager& psmm,
                                     const mapping::ElementId& start_lane_id,
                                     const mapping::ElementId& end_lane_id,
                                     const Vec2d& start_pos,
                                     const Vec2d& end_pos,
                                     const Vec2d& start_tangent,
                                     const Vec2d& end_tangent);

std::optional<Box2d> BuildLaneBoxApprox(const PlannerSemanticMapManager& psmm,
                                        const Vec2d& start_pt,
                                        const Vec2d& end_pt,
                                        mapping::ElementId lane_id);

inline bool IsAVConvergeOrNearCenter(double lateral_v, double current_l) {
  constexpr double kLateralSpeedEpsilon = 0.1;    // m/s.
  constexpr double kLateralOffsetEpsilon = 0.15;  // m.
  // If lateral speed is small, not converging to center.
  if (std::fabs<double>(lateral_v) < kLateralSpeedEpsilon) {
    return false;
  }
  // If current position is near center & lateral speed less than certain
  // threshold, then the vehicle is near center.
  if (std::fabs<double>(current_l) < kLateralOffsetEpsilon) {
    // Near center with small lateral speed.
    return std::fabs<double>(lateral_v) < kLateralSpeedEpsilon;
  }
  // Decide by the current movement.
  return lateral_v * current_l < 0.0;
}

bool CanFindClosestSlowObjectOnPathInRegionBox(
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const Box2d& region_box, const QtfmEnhancedKdTreeFrenetFrame& path_ff,
    double av_max_s, double low_speed_threshold, Vec2d* obj_closest_pt);

// Cut-in should consider path boundaries and overlap rate of lanes.
std::set<std::string> FindSlowCutinObjectsInRegionBox(
    absl::Span<const SpacetimeObjectTrajectory> trajectories,
    const Box2d& region_box, const PathSlBoundary& path_sl,
    const QtfmEnhancedKdTreeFrenetFrame& path_ff, double low_speed_threshold);

std::pair<const mapping::LanePath*, mapping::ElementId>
FindClosestLanePathAtPositionOrNull(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::LanePath> lane_paths, const Vec2d& query_pos);

// According to the position relation between AV current pose and drive
// passage. Postpone converging to lane center if AV is moving away from
// center otherwise track lane center offset directly.
absl::StatusOr<PiecewiseLinearFunction<double, double>>
ComputeTimeLateralOffsetPlfFromPose(const DrivePassage& drive_passage,
                                    const PoseProto& pose);

double ComputeCurvatureCost(absl::Span<const PathPoint> pts,
                            double ref_curvature, double look_forward_length);

inline double ComputePathLengthFromPosition(const DiscretizedPath& path,
                                            const Vec2d& pos) {
  const auto sl = path.XYToSL(pos);
  return (path.length() - sl.s);
}

std::optional<DiscretizedPath> SampleSmoothedPathOnLanePath(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double sample_step_s);

std::set<mapping::ElementId> ExpandLaneIdsBySemantic(
    absl::Span<const mapping::LaneInfo* const> lanes);
}  // namespace internal

absl::StatusOr<AccCorridor> BuildAccCorridorFromClosestLanePath(
    const PoseProto& pose, const ApolloTrajectoryPointProto& plan_start_point,
    const PlannerSemanticMapManager& psmm, const DrivingMapTopo& dm,
    const std::optional<double>& steering_percentage,
    const VehicleGeometryParamsProto& vehicle_geom,
    const VehicleDriveParamsProto& vehicle_drive, double corridor_step_s,
    double total_preview_time);

/*
 *
 * Interface only for unit test
 *
 */

std::set<std::string> CrowdedSceneObjects(
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr, const PoseProto& pose,
    const VehicleGeometryParamsProto& vehicle_geom, double look_forward_length,
    double look_backward_length);

absl::StatusOr<std::pair<PathSlBoundary, std::vector<Vec2d>>>
BuildPathSlBoundaryOnReferencePath(
    const DrivePassage& drive_passage, const PathSlBoundary& map_sl_boundary,
    const DiscretizedPath& ref_path, const Vec2d& av_pos, double heading,
    double speed, const FrenetCoordinate& av_frenet_on_dp, double length,
    double corridor_step_s, const VehicleGeometryParamsProto& vehicle_geom);

std::optional<DiscretizedPath> ResampleSmoothedDiscretizedPathOnXY(
    absl::Span<const Vec2d> xy, const std::vector<double>& accum_s,
    double sample_step_s);

std::vector<mapping::LanePath> FindForwardLanePathsAtPoint(
    const Vec2d& pos, double heading, double speed, double curvature,
    const PlannerSemanticMapManager& psmm, double desire_forward_length);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_WITH_MAP_H_
