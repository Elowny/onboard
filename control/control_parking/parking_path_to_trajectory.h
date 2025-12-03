#ifndef ONBOARD_CONTROL_PARKING_PATH_TO_TRAJECTORY_H_
#define ONBOARD_CONTROL_PARKING_PATH_TO_TRAJECTORY_H_

#include "absl/status/status.h"

#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft {
namespace control {
absl::Status ParkingPathToTrajectory(const VehicleModel& vehicle_model,
                                     TrajectoryProto* trajectory);

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_PARKING_PATH_TO_TRAJECTORY_H_
