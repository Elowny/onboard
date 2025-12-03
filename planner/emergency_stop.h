#ifndef ONBOARD_PLANNER_EMERGENCY_STOP_H_
#define ONBOARD_PLANNER_EMERGENCY_STOP_H_

#include <optional>
#include <vector>

#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace aeb {

struct EmergencyStopInfo {};

// TODO(weijun): Refactor this function by creating another function to compute
// risk area.
std::optional<EmergencyStopInfo> CheckEmergencyStopByCircularMotion(
    const VehicleGeometryParamsProto& vehicle_geom,
    const EmergencyStopParamsProto& emergency_stop_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const PoseProto& vehicle_pose, const ObjectsProto& objects_proto,
    const Chassis& chassis);

std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const EmergencyStopParamsProto& emergency_stop_params,
    const MotionConstraintParamsProto& motion_constraint_params);

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const MotionConstraintParamsProto& motion_constraint_params,
    const TrajectoryPoint& plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto>& prev_trajectory);

}  // namespace aeb
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_EMERGENCY_STOP_H_
