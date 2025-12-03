#ifndef ONBOARD_PLANNER_DECISION_END_OF_PATH_BOUNDARY_H_
#define ONBOARD_PLANNER_DECISION_END_OF_PATH_BOUNDARY_H_

#include "absl/status/statusor.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// This function returns the end of path boundary constraint.
absl::StatusOr<ConstraintProto::StopLineProto> BuildEndOfPathBoundaryConstraint(
    const DrivePassage& passage, const PathSlBoundary& path_boundary);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_END_OF_PATH_BOUNDARY_H_
