#ifndef ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromDrivePassage(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage);

absl::StatusOr<PathSlBoundary> BuildPathBoundaryForLaneKeeping(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>&
        online_map_drift_buffer);

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth,
    const absl::flat_hash_set<std::string>& unsafe_object_ids);

absl::StatusOr<PathSlBoundary> BuildPathBoundaryForLaneChange(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>&
        online_map_drift_buffer);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_
