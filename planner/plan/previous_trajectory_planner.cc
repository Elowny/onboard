#include "onboard/planner/plan/previous_trajectory_planner.h"

#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

#include "onboard/global/trace.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"

namespace qcraft::planner {

absl::StatusOr<PreviousTrajectoryPlannerOutput> RunPreviousTrajectoryPlanner(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const std::vector<ApolloTrajectoryPointProto>&
        time_aligned_prev_trajectory) {
  FUNC_QTRACE();

  if (time_aligned_prev_trajectory.size() != kTrajectorySteps) {
    return absl::UnavailableError("Previous trajectory not available.");
  }

  TrajectoryValidationResultProto result;
  if (!ValidateEstPrevTrajectory(psmm, vehicle_geometry_params,
                                 vehicle_drive_params, motion_constraint_params,
                                 time_aligned_prev_trajectory, &result)) {
    return absl::InternalError(
        absl::StrCat("Validation failed: ", result.DebugString()));
  }

  return PreviousTrajectoryPlannerOutput{.trajectory_points =
                                             time_aligned_prev_trajectory};
}

}  // namespace qcraft::planner
