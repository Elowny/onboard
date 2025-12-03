#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/container/strong_vector.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"

namespace qcraft::planner {

DECLARE_STRONG_VECTOR(GeometryNode);
DECLARE_STRONG_VECTOR(GeometryEdge);

struct GeometryNode {
  GeometryNodeIndex index;
  Vec2d xy;
  double k;
  int station_index = 0;
  double lateral_offset = 0.0;
  double accumulated_s = 0.0;
  // Flag to signify if the node is reachable from start state.
  bool reachable = false;
  bool resampled = false;
  // Flag turned off according to reference line.
  bool active = true;

  std::string DebugString() const {
    return absl::StrFormat(
        "GeometryNode %d(x: %.2f, y: %.2f, k: %.2f, station_index: %d, "
        "accumulated_s: %.2f, reachable: %d, resampled: %d, active: %d)",
        index.value(), xy.x(), xy.y(), k, station_index, accumulated_s,
        reachable, resampled, active);
  }
};

struct GeometryEdge {
  GeometryEdgeIndex index;
  GeometryNodeIndex start;
  GeometryNodeIndex end;
  const GeometryForm* geometry;
  bool truncated = false;
  // Active by default, might be turned off according to reference line
  // searcher result.
  bool active = true;
};

class GeometryGraph {
 public:
  virtual const GeometryNodeVector<GeometryNode>& nodes() const = 0;
  virtual absl::Span<const std::vector<GeometryNodeIndex>> nodes_layers()
      const = 0;
  virtual const GeometryNode& GetNode(GeometryNodeIndex index) const = 0;
  // Start node is on planning start state
  virtual const GeometryNode& GetStartNode() const = 0;
  virtual const GeometryEdgeVector<GeometryEdge>& edges() const = 0;
  virtual const GeometryEdge& GetEdge(GeometryEdgeIndex index) const = 0;

  // Returns the out-going edges of a given node.
  virtual absl::Span<const GeometryEdgeIndex> GetOutgoingEdges(
      GeometryNodeIndex index) const = 0;
  virtual double GetMaxAccumulatedS() const = 0;
  // Return how geometry graph ends.
  virtual const GeometryGraphProto::EndInfo& GetGeometryGraphEndInfo()
      const = 0;

  // Modify geometry node status.
  virtual void DeactivateNode(GeometryNodeIndex) = 0;
  virtual void DeactivateEdge(GeometryEdgeIndex) = 0;

  // Check edge or node status.
  virtual bool IsActive(GeometryNodeIndex) const = 0;
  virtual bool IsActive(GeometryEdgeIndex) const = 0;

  virtual ~GeometryGraph() {}

  virtual void ToProto(GeometryGraphProto* proto) const = 0;
};

class XYGeometryGraph : public GeometryGraph {
 public:
  XYGeometryGraph() = default;
  XYGeometryGraph(
      GeometryNodeVector<GeometryNode> nodes,
      std::vector<std::vector<GeometryNodeIndex>> nodes_layers,
      GeometryEdgeVector<GeometryEdge> edges,
      GeometryNodeVector<std::vector<GeometryEdgeIndex>> outgoing_edges,
      GeometryGraphProto::EndInfo end_info)
      : nodes_(std::move(nodes)),
        nodes_layers_(std::move(nodes_layers)),
        edges_(std::move(edges)),
        nodes_outgoing_edges_(std::move(outgoing_edges)),
        end_info_(std::move(end_info)) {}

  void ToProto(GeometryGraphProto* proto) const override;

  const GeometryNodeVector<GeometryNode>& nodes() const override {
    return nodes_;
  }
  const GeometryNode& GetNode(GeometryNodeIndex index) const override {
    return nodes_[index];
  }
  const GeometryNode& GetStartNode() const override;
  const GeometryEdgeVector<GeometryEdge>& edges() const override {
    return edges_;
  }
  const GeometryEdge& GetEdge(GeometryEdgeIndex index) const override {
    return edges_[index];
  }

  absl::Span<const std::vector<GeometryNodeIndex>> nodes_layers()
      const override {
    return nodes_layers_;
  }

  // Returns the out-going edges of a given node.
  absl::Span<const GeometryEdgeIndex> GetOutgoingEdges(
      GeometryNodeIndex index) const override {
    return nodes_outgoing_edges_[index];
  }

  double GetMaxAccumulatedS() const override;

  const GeometryGraphProto::EndInfo& GetGeometryGraphEndInfo() const override {
    return end_info_;
  }

  // Change geometry graph node status.
  inline void DeactivateNode(GeometryNodeIndex index) override {
    nodes_[index].active = false;
  }

  inline void DeactivateEdge(GeometryEdgeIndex index) override {
    edges_[index].active = false;
  }

  // Get status.
  bool IsActive(GeometryNodeIndex index) const override {
    return nodes_[index].active;
  }
  bool IsActive(GeometryEdgeIndex index) const override {
    return edges_[index].active;
  }

 private:
  GeometryNodeVector<GeometryNode> nodes_;
  // Node indexes organized by layers.
  // This member is particularly useful for Dynamic Programming motion search.
  std::vector<std::vector<GeometryNodeIndex>> nodes_layers_;
  GeometryEdgeVector<GeometryEdge> edges_;
  // The same size with nodes
  GeometryNodeVector<std::vector<GeometryEdgeIndex>> nodes_outgoing_edges_;

  GeometryGraphProto::EndInfo end_info_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_H_
