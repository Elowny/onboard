#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_BUILDER_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_BUILDER_H_

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner {
// TODO(zuowei): Refactor later.
absl::StatusOr<RouteNaviInfo> CalcNaviInfoByLaneGraph(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_dist);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_BUILDER_H_
