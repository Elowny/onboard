#pragma once
#include "absl/container/flat_hash_set.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner::v2 {

LaneGraph BuildRouteLaneGraph(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double build_length);

}  // namespace qcraft::planner::v2
