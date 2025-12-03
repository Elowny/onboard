#ifndef ONBOARD_PREDICTION_LANE_PATH_UTIL_H_
#define ONBOARD_PREDICTION_LANE_PATH_UTIL_H_

#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
// This function returs founded lane paths which is near to the predicted CTRAT
// trajectory.
std::vector<qcraft::mapping::LanePath> FindPossibleLanePathsByCTRATPrediction(
    const ObjectMotionState& cur_state,
    const planner::PlannerSemanticMapManager& psmm);

mapping::LanePath BuildLanePathWithoutVirtualLaneAhead(
    const mapping::LanePath& lp, const planner::PlannerSemanticMapManager& psmm,
    const Vec2d& pos, double heading);
}  // namespace prediction
}  // namespace qcraft
#endif
