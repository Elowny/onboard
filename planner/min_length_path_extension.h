#ifndef ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
#define ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

// Transform a trajectory to a path which extends the last trajectory point by
// an arc with a given minimum length, while considering deceleration distance.
absl::StatusOr<std::vector<PathPoint>> ExtendPathAndDeleteUnreasonablePart(
    absl::Span<const ApolloTrajectoryPointProto> trajectory_points,
    double min_length, double max_curvature);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MIN_LENGTH_PATH_EXTENSION_H_
