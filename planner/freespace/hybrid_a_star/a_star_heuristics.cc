#include "onboard/planner/freespace/hybrid_a_star/a_star_heuristics.h"

#include <algorithm>
#include <cmath>
#include <queue>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

struct Node2d {
  Node2d(int x, int y) : grid_x(x), grid_y(y) {
    id = static_cast<unsigned int>(grid_x) +
         (static_cast<unsigned int>(grid_y) << 10);
  }
  int grid_x = 0;
  int grid_y = 0;
  double cost = 0.0;
  unsigned int id = 0;
};

struct Motion {
  int dx;
  int dy;
  double dcost;
};

bool CheckNodeValidity(
    const FreespaceMap& freespace_map,
    const std::vector<std::pair<std::string, Polygon2d>>& stationary_objects,
    double vehicle_radius, int max_x, int max_y, double grid_resolution,
    const Node2d& node) {
  if (node.grid_x < 0 || node.grid_x >= max_x || node.grid_y < 0 ||
      node.grid_y >= max_y) {
    return false;
  }
  const double x =
      freespace_map.region.min_x() + grid_resolution * (0.5 + node.grid_x);
  const double y =
      freespace_map.region.min_y() + grid_resolution * (0.5 + node.grid_y);
  Vec2d pt(x, y);
  for (const auto& named_obj : stationary_objects) {
    if (named_obj.second.DistanceTo(pt) < vehicle_radius) return false;
  }
  for (const auto& boundary : freespace_map.boundaries) {
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      const Segment2d seg(boundary.points[i], boundary.points[i + 1]);
      if (seg.DistanceTo(pt) < vehicle_radius) return false;
    }
  }
  return true;
}

bool CheckNodeValidityWithKDTree(
    const AABox2d& region, const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map,
    double vehicle_radius, double vehicle_radius_sqr, int max_x, int max_y,
    double grid_resolution, const Node2d& node) {
  if (node.grid_x < 0 || node.grid_x >= max_x || node.grid_y < 0 ||
      node.grid_y >= max_y) {
    return false;
  }
  const double x = region.min_x() + grid_resolution * (0.5 + node.grid_x);
  const double y = region.min_y() + grid_resolution * (0.5 + node.grid_y);
  constexpr double kDistanceBuffer = 0.5;
  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      x, y, vehicle_radius + kDistanceBuffer);
  for (const auto& named_segment : nearby_named_segments) {
    const auto iter = objects_map.find(named_segment.second);
    if (iter != objects_map.end()) {
      if (iter->second->contour.DistanceSquareTo(Vec2d(x, y)) <
          vehicle_radius_sqr) {
        return false;
      }
    } else {
      const auto boundary_iter = boundaries_map.find(named_segment.second);
      QCHECK(boundary_iter != boundaries_map.end());
      if (named_segment.first->DistanceSquareTo(Vec2d(x, y)) <
          vehicle_radius_sqr) {
        return false;
      }
    }
  }
  return true;
}

bool AStarHeuristics::GenerateCostMap(
    const SegmentMatcherKdtree& segments_kd_tree,
    const absl::flat_hash_map<std::string, const FreespaceObject*>& objects_map,
    const absl::flat_hash_map<std::string, const FreespaceBoundary*>&
        boundaries_map) {
  if (cost_map_.empty() || cost_map_.front().empty()) return false;
  // Problem set up.
  absl::flat_hash_set<unsigned int> explored_nodes;
  const auto cmp = [](const Node2d& a, const Node2d& b) {
    return a.cost > b.cost;
  };
  std::priority_queue<Node2d, std::vector<Node2d>, decltype(cmp)> open_pq(cmp);
  if (end_grid_x_ < 0 || end_grid_x_ >= cost_map_.size() || end_grid_y_ < 0 ||
      end_grid_y_ >= cost_map_.front().size()) {
    return false;
  }
  const Node2d start_node(end_grid_x_, end_grid_y_);
  explored_nodes.emplace(start_node.id);
  open_pq.emplace(start_node);

  const double vehicle_radius_sqr = Sqr(vehicle_radius_);
  const std::vector<Motion> motions = {
      {-1, -1, M_SQRT2}, {-1, 0, 1.0},     {-1, 1, M_SQRT2}, {0, -1, 1.0},
      {0, 1, 1.0},       {1, -1, M_SQRT2}, {1, 0, 1.0},      {1, 1, M_SQRT2}};

  // Dijkstra search.
  while (!open_pq.empty()) {
    const auto cur_node = open_pq.top();
    open_pq.pop();
    cost_map_[cur_node.grid_x][cur_node.grid_y] = cur_node.cost;

    for (const auto& motion : motions) {
      const int next_x = cur_node.grid_x + motion.dx;
      const int next_y = cur_node.grid_y + motion.dy;
      Node2d next_node(next_x, next_y);
      if (!CheckNodeValidityWithKDTree(
              region_, segments_kd_tree, objects_map, boundaries_map,
              vehicle_radius_, vehicle_radius_sqr, cost_map_.size(),
              cost_map_.front().size(), grid_resolution_, next_node)) {
        continue;
      }
      next_node.cost =
          std::min(cost_map_[next_node.grid_x][next_node.grid_y],
                   cur_node.cost + motion.dcost * grid_resolution_);
      if (explored_nodes.find(next_node.id) == explored_nodes.end()) {
        explored_nodes.emplace(next_node.id);
        open_pq.emplace(next_node);
      }
    }
  }
  return true;
}

double AStarHeuristics::GetHeuristicsCost(double x, double y) const {
  const int row = FloorToInt((x - region_.min_x()) / grid_resolution_);
  const int column = FloorToInt((y - region_.min_y()) / grid_resolution_);
  if (row < 0 || row >= cost_map_.size() || column < 0 ||
      column >= cost_map_.front().size()) {
    return std::numeric_limits<double>::infinity();
  }
  return cost_map_[row][column];
}
}  // namespace planner
}  // namespace qcraft
