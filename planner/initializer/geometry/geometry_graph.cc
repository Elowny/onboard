#include "onboard/planner/initializer/geometry/geometry_graph.h"

#include <algorithm>

#include "onboard/planner/initializer/geometry/geometry_state.h"

namespace qcraft::planner {

void XYGeometryGraph::ToProto(GeometryGraphProto* proto) const {
  for (const auto& node : nodes_) {
    auto* new_node = proto->add_nodes();
    new_node->set_x(node.xy.x());
    new_node->set_y(node.xy.y());
    new_node->set_station_index(node.station_index);
    new_node->set_node_index(node.index.value());
    new_node->set_active(node.active);
    new_node->set_resampled(node.resampled);
    new_node->set_reachable(node.reachable);
  }
  for (const auto& edge : edges_) {
    auto* new_edge = proto->add_edges();
    new_edge->set_start_node_index(edge.start.value());
    new_edge->set_end_node_index(edge.end.value());
    new_edge->set_active(edge.active);
    new_edge->set_truncated(edge.truncated);
    constexpr double kSampleS = 0.5;  // m.
    const auto states = edge.geometry->Sample(kSampleS);
    for (const auto& state : states) {
      auto* new_state = new_edge->add_states();
      new_state->set_x(state.xy.x());
      new_state->set_y(state.xy.y());
      new_state->set_h(state.h);
      new_state->set_k(state.k);
    }
  }
  for (const auto& outgoings : nodes_outgoing_edges_) {
    auto* new_outgoings = proto->add_outgoing_edges();
    for (const auto out_index : outgoings) {
      new_outgoings->add_outgoing_edge_index(out_index.value());
    }
  }
  *proto->mutable_end_info() = end_info_;
}

double XYGeometryGraph::GetMaxAccumulatedS() const {
  double max_accumulated_s = 0.0;
  for (const auto& node : nodes_) {
    if (node.reachable) {
      max_accumulated_s =
          std::max<double>(node.accumulated_s, max_accumulated_s);
    }
  }
  return max_accumulated_s;
}

const GeometryNode& XYGeometryGraph::GetStartNode() const {
  for (const auto& node_idx : nodes_layers_.front()) {
    if (nodes_[node_idx].reachable) {
      return nodes_[node_idx];
    }
  }
  return nodes_.front();
}

}  // namespace qcraft::planner
