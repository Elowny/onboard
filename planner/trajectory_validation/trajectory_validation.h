#ifndef ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_
#define ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_

#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct StObjectCollisionInfo {
  std::string object_id = "";
  double probability = 0.0;
};

void GetStObjectCollisionInfo(
    const PartialSpacetimeObjectTrajectory& st_object_with_time_range,
    const std::vector<TrajectoryPoint>& traj_points,
    const std::vector<Polygon2d>& av_box, bool skip_no_decision_object,
    StObjectCollisionInfo* info);

std::vector<Polygon2d> GetAvBoxFromTrajPoints(
    const std::vector<TrajectoryPoint>& traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    int obs_check_horizon);

bool ValidateTrajectoryControlLimits(
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    bool check_intrinsic_jerk, bool check_curvature_rate,
    TrajectoryValidationResultProto* result);

bool ValidateTrajectoryInternalConsistency(
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    TrajectoryValidationResultProto* result);

// Basic checks.
bool ValidateTrajectory(
    absl::Span<const TrajectoryPoint> traj_points,
    const TrajectoryValidationOptionsProto& trajectory_validation_options,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    TrajectoryValidationResultProto* result);

bool ValidateEstTrajectoryCurbCollision(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    TrajectoryValidationResultProto* result);

bool ValidateEstTrajectoryObjectCollision(
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    const std::vector<TrajectoryPoint>& traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    int obs_check_horizon, TrajectoryValidationResultProto* result,
    ThreadPool* thread_pool);

bool ValidateTrajectoryOverlap(
    absl::Span<const ApolloTrajectoryPointProto> traj_points,
    absl::Span<const ApolloTrajectoryPointProto> past_points,
    TrajectoryValidationResultProto* result);

void ValidateTrajectoryLateralComfort(
    const std::vector<TrajectoryPoint>& traj_points,
    const MotionConstraintParamsProto& motion_constraint_params);

bool ValidateDrivePassageAndPathBoundaryViolation(
    const std::vector<TrajectoryPoint>& traj_points,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    TrajectoryValidationResultProto* result);

bool ValidateEstTrajectory(
    const PlannerSemanticMapManager& psmm,
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    bool full_stop, const SchedulerOutput& scheduler_output,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto* result, ThreadPool* thread_pool);

bool ValidateEstPrevTrajectory(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto* result);

bool ValidateFreespaceTrajectory(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto* result);

bool ValidateAccTrajectory(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    absl::Span<const ApolloTrajectoryPointProto> past_traj,
    TrajectoryValidationResultProto* result);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_
