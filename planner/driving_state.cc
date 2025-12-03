#include "onboard/planner/driving_state.h"

#include <ostream>

#include "onboard/lite/logging.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"

namespace qcraft {
namespace planner {
DrivingStateProto GetOnRoadDrivingState(
    const VehicleGeometryParamsProto& vehicle_params, bool full_stop,
    const mapping::LanePath& current_lane_path) {
  DrivingStateProto driving_state;
  constexpr double kDistanceToStopThreshold = 1.0;  // m.
  const bool is_av_near_end_of_route =
      current_lane_path.length() - vehicle_params.front_edge_to_center() <
      kDistanceToStopThreshold;
  if (full_stop) {
    if (is_av_near_end_of_route) {
      driving_state.set_type(DrivingStateProto::STOPPED_AT_END_OF_ROUTE);
    } else {
      driving_state.set_type(DrivingStateProto::STOPPED_ON_ROAD);
    }
  }

  return driving_state;
}

DrivingStateProto GetOffRoadDrivingState(
    FreespaceTaskProto::TaskType task_type,
    PathManagerStateProto::DriveState path_manager_drive_state,
    bool full_stop) {
  DrivingStateProto driving_state;
  if (full_stop) {
    switch (path_manager_drive_state) {
      case PathManagerStateProto::UNKNOWN: {
        QLOG(FATAL) << "Unexpected UNKNOWN state.";
      }
      case PathManagerStateProto::SWITCHING_TO_NEXT:
      case PathManagerStateProto::DRIVING: {
        switch (task_type) {
          case FreespaceTaskProto::UNKNOWN_TASK:
            QLOG(FATAL) << "Unexpected UNKNOWN_TASK.";
          case FreespaceTaskProto::PERPENDICULAR_PARKING:
          case FreespaceTaskProto::PARALLEL_PARKING:
          case FreespaceTaskProto::THREE_POINT_TURN:
          case FreespaceTaskProto::DRIVING_TO_LANE:
          case FreespaceTaskProto::FREE_DRIVING:
          case FreespaceTaskProto::CUSTOM_PARKING:
            // TODO(yumeng): Add driving state for this tasks.
            driving_state.set_type(DrivingStateProto::STOPPED_WHEN_PARKING);
            break;
        }
        break;
      }
      case PathManagerStateProto::CENTER_STEER:
      case PathManagerStateProto::REACH_FINAL_GOAL: {
        switch (task_type) {
          case FreespaceTaskProto::UNKNOWN_TASK:
            QLOG(FATAL) << "Unexpected UNKNOWN_TASK.";
          case FreespaceTaskProto::PERPENDICULAR_PARKING:
          case FreespaceTaskProto::PARALLEL_PARKING:
          case FreespaceTaskProto::CUSTOM_PARKING:
            driving_state.set_type(DrivingStateProto::STOPPED_IN_PARKING_SPOT);
            break;
          case FreespaceTaskProto::THREE_POINT_TURN:
          case FreespaceTaskProto::DRIVING_TO_LANE:
          case FreespaceTaskProto::FREE_DRIVING:
            // TODO(yumeng): Add driving state for this tasks.
            driving_state.set_type(DrivingStateProto::STOPPED_WHEN_PARKING);
            break;
        }
        break;
      }
    }
  }

  return driving_state;
}
}  // namespace planner
}  // namespace qcraft
