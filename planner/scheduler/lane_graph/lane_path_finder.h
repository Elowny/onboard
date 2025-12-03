#ifndef ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner {

absl::StatusOr<std::vector<LanePathInfo>> FindBestLanePathsFromStart(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info,
    const RouteNaviInfo& route_navi_info, const v2::LaneGraph& lane_graph,
    ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_PATH_FINDER_H_
