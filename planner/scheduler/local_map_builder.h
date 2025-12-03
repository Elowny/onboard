#ifndef ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_LOCAL_MAP_BUILDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft::planner {
// This function will return a lane paths vector sorted from left to right.
// Output lane paths num less than the lane num of first route section. The
// input param "route_sections" must start from AV pos.
absl::StatusOr<std::vector<mapping::LanePath>> BuildLocalMap(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info);
}  // namespace qcraft::planner

#endif
