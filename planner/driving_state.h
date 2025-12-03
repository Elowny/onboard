#ifndef ONBOARD_PLANNER_DRIVING_STATE_H_
#define ONBOARD_PLANNER_DRIVING_STATE_H_

#include "onboard/maps/lane_path.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// This function return AV driving state.
DrivingStateProto GetOnRoadDrivingState(
    const VehicleGeometryParamsProto& vehicle_params, bool full_stop,
    const mapping::LanePath& current_lane_path);

DrivingStateProto GetOffRoadDrivingState(
    FreespaceTaskProto::TaskType task_type,
    PathManagerStateProto::DriveState path_manager_drive_state, bool full_stop);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DRIVING_STATE_H_
