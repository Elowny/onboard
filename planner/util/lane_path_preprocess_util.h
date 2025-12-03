#ifndef ONBOARD_PLANNER_UTIL_LANE_PATH_PREPROCESS_UTIL_H_
#define ONBOARD_PLANNER_UTIL_LANE_PATH_PREPROCESS_UTIL_H_

#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

std::vector<Vec2d> StraightenedLanePathPoints(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double start_s, double end_s, double step);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_LANE_PATH_PREPROCESS_UTIL_H_
