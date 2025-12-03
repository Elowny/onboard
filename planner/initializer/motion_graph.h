#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_

#include <utility>  // IWYU pragma: keep // std::move for cpplint
#include <vector>

#include "glog/logging.h"

#include "onboard/container/strong_vector.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"

namespace qcraft::planner {
DECLARE_STRONG_VECTOR(MotionNode);
DECLARE_STRONG_VECTOR(MotionEdge);

struct MotionNode {
  MotionNodeIndex index;
  MotionState state;
  GeometryNodeIndex geom_index;
};

struct MotionEdge {
  MotionNodeIndex start;
  MotionNodeIndex end;
  const MotionForm* motion;
  MotionEdgeIndex prev_edge;
};

class MotionGraph {
 public:
  explicit MotionGraph(const GeometryGraph* geom_graph)
      : geometry_graph_(geom_graph) {}
  virtual int edge_size() const = 0;
  virtual const MotionNode& GetMotionNode(MotionNodeIndex i) const = 0;
  virtual const MotionEdge& GetMotionEdge(MotionEdgeIndex i) const = 0;
  virtual MotionNodeIndex AddMotionNode(MotionState node,
                                        GeometryNodeIndex geom_index) = 0;
  virtual MotionEdgeIndex AddMotionEdge(MotionNodeIndex start_node_index,
                                        MotionNodeIndex end_node_index,
                                        const MotionForm* motion_form,
                                        GeometryNodeIndex end_geom_index,
                                        MotionEdgeIndex prev_edge) = 0;

  virtual const GeometryGraph* geometry_graph() const {
    return geometry_graph_;
  }

  virtual ~MotionGraph() {}

  virtual void ToProto(MotionGraphProto* proto) const = 0;

 protected:
  const GeometryGraph* geometry_graph_;
};

class XYTMotionGraph : public MotionGraph {
 public:
  explicit XYTMotionGraph(const GeometryGraph* geom_graph)
      : MotionGraph(geom_graph) {}

  int edge_size() const override { return edges_.size(); }

  const MotionNode& GetMotionNode(MotionNodeIndex i) const override {
    return nodes_[i];
  }
  const MotionEdge& GetMotionEdge(MotionEdgeIndex i) const override {
    return edges_[i];
  }

  MotionNodeIndex AddMotionNode(MotionState node,
                                GeometryNodeIndex geom_index) override;
  MotionEdgeIndex AddMotionEdge(MotionNodeIndex start_node_index,
                                MotionNodeIndex end_node_index,
                                const MotionForm* motion_form,
                                GeometryNodeIndex end_geom_index,
                                MotionEdgeIndex prev_edge) override;

  void ToProto(MotionGraphProto* proto) const override;

 private:
  std::vector<MotionEdgeIndex>* GetOrCreateOutgoingEdge(MotionNodeIndex i);

  MotionNodeVector<MotionNode> nodes_;
  MotionNodeVector<std::vector<MotionEdgeIndex>> outgoing_edges_;
  MotionEdgeVector<MotionEdge> edges_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_
