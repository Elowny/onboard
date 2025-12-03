#include "onboard/planner/initializer/motion_graph.h"

#include <utility>

namespace qcraft::planner {

MotionNodeIndex XYTMotionGraph::AddMotionNode(MotionState node,
                                              GeometryNodeIndex geom_index) {
  MotionNodeIndex index(nodes_.size());
  nodes_.push_back(MotionNode{
      .index = index, .state = std::move(node), .geom_index = geom_index});
  outgoing_edges_.push_back(std::vector<MotionEdgeIndex>());
  return index;
}

MotionEdgeIndex XYTMotionGraph::AddMotionEdge(
    MotionNodeIndex start_node_index, MotionNodeIndex end_node_index,
    const MotionForm* motion_form, GeometryNodeIndex /*end_geom_index*/,
    MotionEdgeIndex prev_edge) {
  MotionEdgeIndex edge_index(edges_.size());
  edges_.push_back(MotionEdge{.start = start_node_index,
                              .end = end_node_index,
                              .motion = motion_form,
                              .prev_edge = prev_edge});

  QCHECK(outgoing_edges_.valid_index(start_node_index));
  outgoing_edges_[start_node_index].emplace_back(edge_index);

  return edge_index;
}

void XYTMotionGraph::ToProto(MotionGraphProto* proto) const {
  proto->mutable_nodes()->Reserve(nodes_.size());

  for (const auto index : nodes_.index_range()) {
    auto* node_proto = proto->add_nodes();
    node_proto->set_index(index.value());
    nodes_[index].state.ToProto(node_proto->mutable_state());
  }

  for (const auto& edge : edges_) {
    auto* edge_proto = proto->add_edges();
    edge_proto->set_start_node_index(edge.start.value());
    edge_proto->set_end_node_index(edge.end.value());
    edge_proto->set_prev_edge_index(edge.prev_edge.value());
  }
}

}  // namespace qcraft::planner
