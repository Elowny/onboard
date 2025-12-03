#ifndef ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_
#define ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_

#include "absl/status/statusor.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// This function returns end of current lane path constraint: it signifies
// either the end of current trip, or the last lane point before lane change.
absl::StatusOr<ConstraintProto::StopLineProto>
BuildEndOfCurrentLanePathConstraint(const DrivePassage& passage);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_END_OF_CURRENT_LANE_PATH_H_
