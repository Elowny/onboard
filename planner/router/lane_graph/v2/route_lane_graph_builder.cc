#include "onboard/planner/router/lane_graph/v2/route_lane_graph_builder.h"

#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_point.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
#include "onboard/planner/router/lane_graph/v2/lane_graph_util.h"

namespace qcraft::planner::v2 {

LaneGraph BuildRouteLaneGraph(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double build_length) {
  SCOPED_QTRACE("BuildRouteLaneGraph");

  const auto& sections = sections_info.section_segments();
  int last_sec_idx = 0;
  double last_sec_frac = 0.0;

  const bool target_in_horizon = sections_info.length() <= build_length;
  if (target_in_horizon) {
    last_sec_idx = sections_info.size() - 1;
    last_sec_frac = sections_info.end_fraction();
  } else {
    double accum_length = 0.0;
    for (; last_sec_idx < sections.size(); ++last_sec_idx) {
      const auto& section = sections[last_sec_idx];
      if (accum_length + section.length() >= build_length) {
        last_sec_frac = section.start_fraction +
                        (build_length - accum_length) / section.average_length;
        break;
      }
      accum_length += section.length();
    }
  }

  VertexGraph<LaneGraph::VertexId> graph;
  auto layers =
      SplitSectionsAsVertices(sections_info, last_sec_idx, last_sec_frac);

  int vertices_count = 0;
  for (const auto& layer : layers) {
    vertices_count += sections[layer.first].lane_ids.size();
  }
  graph.Reserve(vertices_count);
  for (int i = 0; i < layers.size(); ++i) {
    for (const auto lane_id : sections[layers[i].first].lane_ids) {
      graph.AddVertex(ToVertId(i, lane_id));
    }
  }
  graph.AddVertex(LaneGraph::kTargetVertex);

  ConnectEdgesWithinSections(v2smm, sections_info, avoid_lanes, layers, &graph);

  absl::flat_hash_set<LaneGraph::VertexId> fork_lc_verts;
  absl::flat_hash_set<LaneGraph::EdgeType> fork_lk_edges;
  ConnectEdgesAcrossSections(v2smm, sections_info, avoid_lanes, layers,
                             last_sec_idx, &graph, &fork_lc_verts,
                             &fork_lk_edges);

  // Add normal connection edges to the target vertex.
  const auto last_layer = layers.size() - 1;
  const bool destination_in_horizon =
      target_in_horizon &&
      sections_info.destination().lane_id() != mapping::kInvalidElementId;
  if (destination_in_horizon) {
    // From the target point.
    for (const auto lane_id : sections.back().lane_ids) {
      if (lane_id == sections_info.destination().lane_id()) {
        graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                      0.0);
      } else {
        // Avoid kickout when can not connect to target vertex.
        graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                      LaneGraph::kDeadEndToTargetCost);
      }
    }
  } else {
    // From vertices of the last layer that extends beyond horizon.
    const auto& last_sec = sections[last_sec_idx];
    for (const auto lane_id : last_sec.lane_ids) {
      graph.AddEdge(ToVertId(last_layer, lane_id), LaneGraph::kTargetVertex,
                    0.0);
    }
  }

  return LaneGraph{.layers = std::move(layers),
                   .graph = std::move(graph),
                   .fork_lc_verts = std::move(fork_lc_verts),
                   .fork_lk_edges = std::move(fork_lk_edges)};
}

}  // namespace qcraft::planner::v2
