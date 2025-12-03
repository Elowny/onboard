#ifndef ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_
#define ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_

#include <string>
#include <vector>

#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

std::vector<TrajectoryPoint> SmoothTrajectoryByMixedFourthOrderDdp(
    int trajectory_steps, double trajectory_time_step,
    const std::vector<TrajectoryPoint>& ref_traj,
    const DrivePassage& drive_passage, const std::string& owner,
    const TrajectorySmootherCostWeightParamsProto& smoother_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MFOB_TRAJECTORY_SMOOTHER_H_
