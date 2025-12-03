#ifndef ONBOARD_PLANNER_DECISION_BEYOND_LENGTH_ALONG_ROUTE_H_
#define ONBOARD_PLANNER_DECISION_BEYOND_LENGTH_ALONG_ROUTE_H_

#include "absl/status/statusor.h"

#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// This function returns beyond length along route constraint: if the ego
// vehicle goes beyond length along route of a lane path, its speed must be
// restricted to an extreme low level.
absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildBeyondLengthAlongRouteConstraint(
    const DrivePassage& dp,
    const MotionConstraintParamsProto& motion_constraint_params,
    double length_along_route, bool borrow_boundary, double ego_v);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_BEYOND_LENGTH_ALONG_ROUTE_H_
