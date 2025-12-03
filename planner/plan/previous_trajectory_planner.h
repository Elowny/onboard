#ifndef ONBOARD_PLANNER_PLAN_PREVIOUS_TRAJECTORY_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_PREVIOUS_TRAJECTORY_PLANNER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct PreviousTrajectoryPlannerOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
};

absl::StatusOr<PreviousTrajectoryPlannerOutput> RunPreviousTrajectoryPlanner(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const std::vector<ApolloTrajectoryPointProto>&
        time_aligned_prev_trajectory);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PREVIOUS_TRAJECTORY_PLANNER_H_
