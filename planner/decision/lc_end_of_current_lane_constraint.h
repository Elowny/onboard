#ifndef ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
#define ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

absl::StatusOr<ConstraintProto::SpeedRegionProto>
BuildLcEndOfCurrentLaneConstraints(const DrivePassage& dp,
                                   const mapping::LanePath& lane_path_before_lc,
                                   double ego_v);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
