#ifndef ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_H_
#define ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_H_

#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/route_sections_info.h"

namespace qcraft::planner::v2 {
LaneGraph::LaneGraphLayers SplitSectionsAsVertices(
    const RouteSectionsInfo& sections_info, int last_sec_idx,
    double last_sec_frac);

void ConnectEdgesWithinSections(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const LaneGraph::LaneGraphLayers& layers,
    VertexGraph<LaneGraph::VertexId>* graph);

void ConnectEdgesAcrossSections(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    const LaneGraph::LaneGraphLayers& layers, int last_sec_idx,
    VertexGraph<LaneGraph::VertexId>* graph,
    absl::flat_hash_set<LaneGraph::VertexId>* fork_lc_verts,
    absl::flat_hash_set<LaneGraph::EdgeType>* fork_lk_edges);

// Including the target but not the start id.
std::vector<mapping::ElementId> DirectlyConnectedLanesBetween(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const LaneGraph::LaneGraphLayers& layers, int start_layer, int target_layer,
    mapping::ElementId start_id, mapping::ElementId target_id);

double GetSectionsLength(const mapping::v2::SemanticMapManager& v2smm,
                         const std::vector<mapping::ElementId>& lane_ids,
                         double start_frac, double end_frac);
}  // namespace qcraft::planner::v2

#endif  // ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_H_
