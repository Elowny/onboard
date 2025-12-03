#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "onboard/planner/router/lane_graph/v2/lane_graph.h"
namespace qcraft::planner::v2 {

template <typename VertexId>
std::vector<VertexId> RebuildPath(
    const absl::flat_hash_map<VertexId, std::pair<double, VertexId>>&
        cost_and_prev,
    VertexId goal_id) {
  std::vector<VertexId> path;
  path.push_back(goal_id);
  while (cost_and_prev.at(goal_id).second != InvalidVertexId(goal_id)) {
    path.push_back(goal_id = cost_and_prev.at(goal_id).second);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

template <typename VertexId>
absl::StatusOr<TravelPath<VertexId>> Dijkstra(
    const VertexGraph<VertexId>& graph, const VertexId& src_id,
    const VertexId& goal_id) {
  using CostIdPair = std::pair<double, VertexId>;
  if (!graph.has_vertex(src_id)) {
    return absl::NotFoundError(absl::StrCat(
        "Source vertex ", VertexIdToString(src_id), " does not exist!"));
  }
  if (!graph.has_vertex(goal_id)) {
    return absl::NotFoundError(absl::StrCat(
        "Target vertex ", VertexIdToString(goal_id), " does not exist!"));
  }

  absl::flat_hash_map<VertexId, std::pair<double, VertexId>> cost_and_prev;
  cost_and_prev.reserve(graph.vertices().size());
  for (const auto& vert_id : graph.vertices()) {
    cost_and_prev[vert_id] = {std::numeric_limits<double>::infinity(),
                              InvalidVertexId(goal_id)};
  }
  cost_and_prev[src_id].first = 0.0;
  constexpr int kDefaultCapacity = 1024;
  std::vector<CostIdPair> vec;
  vec.reserve(kDefaultCapacity);
  std::priority_queue<CostIdPair, std::vector<CostIdPair>,
                      std::greater<CostIdPair>>
      q(std::greater<CostIdPair>(), std::move(vec));
  q.push({0.0, src_id});
  while (!q.empty()) {
    const auto [cur_cost, cur_id] = q.top();
    q.pop();
    if (cost_and_prev[cur_id].first != cur_cost)
      continue;  // Not the first visit.
    if (cur_id == goal_id) {
      return TravelPath<VertexId>{
          .vertices = RebuildPath(cost_and_prev, goal_id),
          .total_cost = cur_cost};
    }

    for (const auto& [to_id, edge_cost] : graph.edges_from(cur_id)) {
      double new_cost = cur_cost + edge_cost;
      auto& item = cost_and_prev[to_id];
      if (new_cost < item.first) {
        item = {new_cost, cur_id};
        q.push({new_cost, to_id});
      }
    }
  }

  return absl::NotFoundError("Path not found!");
}

}  // namespace qcraft::planner::v2
