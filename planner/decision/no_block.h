#ifndef ONBOARD_PLANNER_DECISION_NO_BLOCK_H_
#define ONBOARD_PLANNER_DECISION_NO_BLOCK_H_

#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// This function return no block constraint. For more details see 'no_block.md'
std::vector<ConstraintProto::SpeedRegionProto> BuildNoBlockConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_NO_BLOCK_H_
