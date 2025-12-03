#ifndef ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_
#define ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<ConstraintProto::AvoidLineProto>>
BuildSolidLineWithinBoundaryConstraint(
    const DrivePassage& drive_passage, const PathSlBoundary& path_boundary,
    const ApolloTrajectoryPointProto& plan_start_point);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_SOLID_LINE_WITHIN_BOUNDARY_H_
