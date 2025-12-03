#ifndef ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_
#define ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_

#include <vector>

#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// This function returns speed bump constraints.
std::vector<ConstraintProto::SpeedRegionProto> BuildSpeedBumpConstraints(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_SPEED_BUMP_H_
