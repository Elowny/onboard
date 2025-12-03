#ifndef ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_GRAPH_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_GRAPH_BUILDER_H_

#include <string>

#include "absl/container/flat_hash_set.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner {

v2::LaneGraph BuildLaneGraph(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, const PlannerObjectManager& obj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const RouteNaviInfo& route_navi_info);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_LANE_GRAPH_LANE_GRAPH_BUILDER_H_
