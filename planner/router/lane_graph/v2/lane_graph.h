#ifndef ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_
#define ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "onboard/container/flat_map.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner::v2 {

template <typename T>
struct is_std_pair : std::false_type {};  // NOLINT;

template <typename T, typename U>
struct is_std_pair<std::pair<T, U>> : std::true_type {};  // NOLINT

template <typename T>
constexpr bool is_std_pair_v = is_std_pair<std::decay_t<T>>::value;  // NOLINT

template <typename T>
inline std::string VertexIdToString(T&& e) {
  if constexpr (is_std_pair_v<T>) {
    return absl::StrCat(e.first, "-", e.second);
  } else {
    return std::to_string(e);
  }
}

static constexpr double kInfiniteCost = std::numeric_limits<double>::max();

template <typename VertexId>
struct TravelPath {
  std::vector<VertexId> vertices;
  double total_cost = kInfiniteCost;
};

template <typename VertexId>
struct VertexGraph {
 public:
  // Maps to_id to edge cost, order matters for reproducibility.
  using VertexEdgeMap = FlatMap<VertexId, double>;
  using VertexSet = absl::flat_hash_set<VertexId>;

 public:
  void Reserve(int vertices_count) {
    vertices_.reserve(vertices_count);
    edges_.reserve(vertices_count);
  }

  void AddVertex(const VertexId& vert_id) {
    if (const auto [_, inserted] = vertices_.insert(vert_id); !inserted) {
      QLOG(WARNING) << VertexIdToString(vert_id) << " already exists!";
      return;
    }
    VertexEdgeMap empty_map;
    empty_map.reserve(/*default_capacity=*/5);
    edges_.emplace(vert_id, std::move(empty_map));
  }

  /**
   * @brief It's `UpdateOrInsert` an edge, other than `Insert`. Never check the
   * vertices exists
   * @param from_id
   * @param to_id
   * @param cost
   */
  void AddEdge(const VertexId& from_id, const VertexId& to_id, double cost) {
    if (!vertices_.contains(from_id) || !vertices_.contains(to_id)) {
      QLOG(WARNING) << "please insert vertices first,"
                    << VertexIdToString(from_id) << " -> "
                    << VertexIdToString(to_id);
      return;
    }
    if (cost == kInfiniteCost) return;
    auto& edge_map = edges_[from_id];
    if (const auto [edge, inserted] = edge_map.try_emplace(to_id, cost);
        !inserted) {
      edge->second = std::min(edge->second, cost);
    }
  }

  void RemoveEdge(const VertexId& from_id, const VertexId& to_id) {
    if (!IsEdgeValid(from_id, to_id)) {
      QLOG(WARNING) << "Invalid edge " << VertexIdToString(from_id) << "->"
                    << VertexIdToString(to_id);
      return;
    }
    edges_[from_id].erase(to_id);
  }

  void RemoveEdgesFrom(const VertexId& from_id) {
    if (!vertices_.contains(from_id)) {
      QLOG(WARNING) << VertexIdToString(from_id) << " does not exist!";
      return;
    }
    edges_[from_id].clear();
  }

  void ModifyEdge(const VertexId& from_id, const VertexId& to_id, double cost) {
    if (!IsEdgeValid(from_id, to_id)) {
      QLOG(WARNING) << "Invalid edge ( " << VertexIdToString(from_id) << "->"
                    << VertexIdToString(to_id) << ") !";
      return;
    }
    edges_[from_id][to_id] = cost;
  }

  bool has_vertex(const VertexId& vert_id) const {
    return vertices_.contains(vert_id);
  }
  double edge_cost(const VertexId& from_id, const VertexId& to_id) const {
    if (const auto* edge = FindOrNull(edges_, from_id)) {
      if (const auto* v = FindOrNull(*edge, to_id)) {
        return *v;
      }
    }
    return kInfiniteCost;
  }
  const VertexSet& vertices() const { return vertices_; }
  const VertexEdgeMap& edges_from(const VertexId& from_id) const {
    DCHECK(vertices_.contains(from_id))
        << "Invalid vertex id " << VertexIdToString(from_id);
    return FindOrDieNoPrint(edges_, from_id);
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << vertices_.size() << " vertices in total\n";
    for (const auto& vertex : vertices_) {
      ss << "  " << VertexIdToString(vertex) << ":\n";
      for (const auto& [to_id, cost] : FindOrDieNoPrint(edges_, vertex)) {
        ss << "    edge to " << VertexIdToString(to_id) << " with cost " << cost
           << std::endl;
      }
    }
    return ss.str();
  }

 private:
  bool IsEdgeValid(const VertexId& from_id, const VertexId& to_id) const {
    return vertices_.contains(from_id) && vertices_.contains(to_id) &&
           ContainsKey(FindOrDieNoPrint(edges_, from_id), to_id);
  }

  VertexSet vertices_;
  absl::flat_hash_map<VertexId, VertexEdgeMap> edges_;
};

constexpr inline std::pair<int, std::int64_t> ToVertId(
    int layer, mapping::ElementId lane_id) {
  return {layer, lane_id.value()};
}

constexpr inline std::pair<int, std::int64_t> ToVertId(int layer,
                                                       std::int64_t lane_id) {
  return {layer, lane_id};
}

constexpr inline std::pair<int, mapping::ElementId> FromVertId(
    std::pair<int, std::int64_t> vertex_id) {
  return {vertex_id.first, mapping::ElementId(vertex_id.second)};
}

struct LaneGraph {
  using VertexId = std::pair<int, std::int64_t>;
  using EdgeType = std::pair<VertexId, VertexId>;
  using LaneGraphLayers = std::vector<std::pair<int, double>>;

  static constexpr double kInfiniteCost =
      std::numeric_limits<double>::infinity();
  static constexpr double kDeadEndToTargetCost = 1e10;
  static constexpr VertexId kInvalidVertexId = ToVertId(-1, 0L);
  static constexpr VertexId kTargetVertex = ToVertId(-1, -1L);

  // Maps a layer to the corresponding section index and fraction.
  LaneGraphLayers layers;
  VertexGraph<VertexId> graph;
  absl::flat_hash_set<VertexId> fork_lc_verts;
  absl::flat_hash_set<EdgeType> fork_lk_edges;
  // When the lane graph does not contain the destination in planner, in order
  // to distinguish the lanes in the last section, add a extra cost
  // according to max driving distance. To avoid to affect the target lane path
  // filter, this item should not be included in the past cost.
  absl::flat_hash_map<VertexId, double> to_target_extra_cost;
};

constexpr const LaneGraph::VertexId& InvalidVertexId(LaneGraph::VertexId) {
  return LaneGraph::kInvalidVertexId;
}

// Only for unittest
constexpr std::int64_t InvalidVertexId(std::int64_t) { return -1L; }

}  // namespace qcraft::planner::v2

#endif  // ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_
