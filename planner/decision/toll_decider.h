#ifndef ONBOARD_PLANNER_DECISION_TOLL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TOLL_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {

namespace planner {
absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildTollConstraints(const PlannerSemanticMapManager& psmm,
                     const DrivePassage& passage,
                     const mapping::LanePath& lane_path_from_start,
                     double s_offset);
}
}  // namespace qcraft

#endif
