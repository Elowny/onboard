#include "onboard/planner/initializer/test_util.h"

#include <string>

#include "absl/strings/str_format.h"

namespace qcraft {
namespace planner {

absl::Status CheckGeometryGraphConnectivity(const GeometryGraph& graph) {
  const auto& nodes = graph.nodes();
  // Check reachable and outgoing edges of the nodes should comply.
  for (const auto& node : nodes) {
    const auto& outgoing_edges_per_node = graph.GetOutgoingEdges(node.index);
    if (outgoing_edges_per_node.size() > 0) {
      if (!node.reachable) {
        std::string result = absl::StrFormat(
            "Node %d has non-zero outgoing edges but is recorded as "
            "unreachable. It is resampled? %d.",
            node.index.value(), node.resampled);
        return absl::InternalError(result);
      }
    }
  }
  // Check query node effect.
  for (int i = 0; i < graph.nodes_layers().size(); ++i) {
    const auto& layer = graph.nodes_layers()[i];
    for (const auto& node_idx : layer) {
      const auto& node = graph.GetNode(node_idx);
      const auto& outgoing_edges_per_node = graph.GetOutgoingEdges(node_idx);
      if (outgoing_edges_per_node.size() > 0) {
        if (!node.reachable) {
          std::string result = absl::StrFormat(
              "From nodes_layers: Node %d has non-zero outgoing edges but is "
              "recorded as unreachable. It is resampled? %d. It is at layer "
              "idx: %d",
              node.index.value(), node.resampled, i);
          return absl::InternalError(result);
        }
      }
    }
  }
  return absl::OkStatus();
}

}  // namespace planner
}  // namespace qcraft
