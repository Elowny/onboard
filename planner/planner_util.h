#ifndef ONBOARD_PLANNER_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PLANNER_UTIL_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/time/time.h"

#include "common/proto/semantic_map_modifier.pb.h"

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/selector/proto/selector_params.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

////////////////////////////////////////////////////////////////////////////////
// Vehicle kinematics related.
double ComputeLongitudinalJerk(const TrajectoryPoint& traj_point);
double ComputeLateralAcceleration(const TrajectoryPoint& traj_point);
double ComputeLateralJerk(const TrajectoryPoint& traj_point);

////////////////////////////////////////////////////////////////////////////////
// Perception related.
bool IsVulnerableRoadUserType(ObjectType type);
bool IsStaticObjectType(ObjectType type);

////////////////////////////////////////////////////////////////////////////////
// Semantic map and lane path related.

// Preprocess semantic map modifier proto. Convert it to
// PlannerSemanticMapModification data structure. Then it can be used to
// initialize planner semantic map manager directly.
PlannerSemanticMapModification CreateSemanticMapModification(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const mapping::SemanticMapModifierProto& modifier);

mapping::SemanticMapModifierProto PlannerSemanticMapModificationToProto(
    const PlannerSemanticMapModification& modifier);

// Copied from planner module.
std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const TrajectoryProto& prev_traj, bool reset,
    int max_past_point_num);

std::vector<PathPoint> CreatePastDirectionalPathPoints(
    const TrajectoryProto& prev_traj, bool reset,
    double start_point_s_on_prev_directional_path, bool freespace_reset);

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const std::optional<bool>& is_forward_task);

ApolloTrajectoryPointProto ComputePlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

ApolloTrajectoryPointProto ComputeACCPlanStartPointAfterLateralReset(
    const std::optional<ApolloTrajectoryPointProto>& prev_reset_planned_point,
    const PoseProto& pose, const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double t_diff,
    double prev_longitudinal_error);

inline ApolloTrajectoryPointProto ComputePlanStartPointFromPrevPlannedPathPoint(
    const PathPoint& prev_reset_planned_path_point, const PoseProto& pose) {
  ApolloTrajectoryPointProto plan_start_point;
  // Reset longitudinal quantities from pose.
  plan_start_point.set_v(pose.vel_body().x());
  plan_start_point.set_a(pose.accel_body().x());
  plan_start_point.set_j(0.0);

  *plan_start_point.mutable_path_point() = prev_reset_planned_path_point;
  plan_start_point.mutable_path_point()->set_s(0.0);

  return plan_start_point;
}

ApolloTrajectoryPointProto
ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
    const TrajectoryProto& prev_traj, const PoseProto& pose,
    const Chassis& chassis,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

SelectorParamsProto LoadSelectorParamsFromFile(const std::string& file_address);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_UTIL_H_
