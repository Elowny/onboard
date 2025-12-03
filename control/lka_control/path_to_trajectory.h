#ifndef ONBOARD_CONTROL_LKA_CONTROL_PATH_TO_TRAJECTORY_H_
#define ONBOARD_CONTROL_LKA_CONTROL_PATH_TO_TRAJECTORY_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

absl::StatusOr<TrajectoryProto> PathToTrajectory(
    const std::vector<PathPoint>& path);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_LKA_CONTROL_PATH_TO_TRAJECTORY_H_
