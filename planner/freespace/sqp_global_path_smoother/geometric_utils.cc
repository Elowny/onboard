#include "onboard/planner/freespace/sqp_global_path_smoother/geometric_utils.h"

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/hyperplane.h"
#include "onboard/math/geometry/intersection_util.h"

namespace qcraft {
namespace planner {
namespace {
// Sort points in the counter-clockwise order.
void SortPointsCcw(std::vector<Vec2d>* pts) {
  if (pts->empty()) return;
  // Calculate center point.
  const Vec2d avg =
      std::accumulate(pts->begin(), pts->end(), Vec2d::Zero()) / pts->size();
  const auto calc_theta = [&avg](const Vec2d& pt) {
    return fast_math::Atan2(pt.y() - avg.y(), pt.x() - avg.x());
  };
  std::sort(pts->begin(), pts->end(),
            [&calc_theta](const Vec2d& pt, const Vec2d& other_pt) {
              return calc_theta(pt) < calc_theta(other_pt);
            });
}

// Find intersection between multiple lines.
// Line: [tangent, point]
std::vector<Vec2d> GetLineIntersects(
    const std::vector<std::pair<Vec2d, Vec2d>>& lines) {
  std::vector<Vec2d> pts;
  pts.reserve(lines.size());
  for (int i = 0; i < lines.size(); ++i) {
    for (int j = i + 1; j < lines.size(); ++j) {
      Vec2d intersection;
      const auto is_found = FindIntersectionBetweenLinesWithTangents(
          lines[i].second, lines[i].first, lines[j].second, lines[j].first,
          &intersection);
      if (is_found) {
        pts.push_back(intersection);
      }
    }
  }
  return pts;
}
}  // namespace

// Find extreme points of ConvexRegion.
std::vector<Vec2d> CalVertices(const ConvexRegion& convex_region) {
  std::vector<std::pair<Vec2d, Vec2d>> lines;
  const auto& vs = convex_region.hyperplanes();
  for (int i = 0; i < vs.size(); ++i) {
    const Vec2d v = vs[i].n().Perp();
    lines.push_back(std::make_pair(v, vs[i].p()));
  }
  auto vts = GetLineIntersects(lines);
  convex_region.RemovePointsNotInside(&vts);
  SortPointsCcw(&vts);

  return vts;
}
}  // namespace planner
}  // namespace qcraft
