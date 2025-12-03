#include "onboard/planner/common/path_approx.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "onboard/math/geometry/segment2d.h"

namespace qcraft {
namespace planner {

namespace {

std::vector<int> ComputePointToSegmentIndex(
    const absl::Span<const PathSegment> segments) {
  QCHECK(!segments.empty());

  std::vector<int> indices;
  indices.resize(segments.back().last_index() + 1, -1);

  for (int i = 0; i < segments.size(); ++i) {
    for (int j = segments[i].first_index(); j <= segments[i].last_index();
         ++j) {
      indices[j] = i;
    }
  }
  return indices;
}

}  // namespace

PathApprox::PathApprox(std::vector<PathSegment> segments,
                       const SegmentMatcherKdtree* path_kd_tree)
    : segments_(std::move(segments)),
      point_to_segment_index_(ComputePointToSegmentIndex(segments_)),
      path_kd_tree_(path_kd_tree) {}

PathApprox BuildPathApprox(const absl::Span<const PathPoint> path_points,
                           const OffsetRect& rect, double tolerance,
                           const SegmentMatcherKdtree* path_kd_tree) {
  QCHECK(!path_points.empty());
  std::vector<PathSegment> segments;

  const double box_width = 2.0 * rect.half_width();

  const double ra_to_fb = rect.offset_to_front();
  const double ra_to_rb = rect.offset_to_rear();

  Vec2d prev_dir = Vec2d::FastUnitFromAngle(path_points[0].theta());
  Vec2d prev_ra(path_points[0].x(), path_points[0].y());
  int i = 0;
  while (i < path_points.size()) {
    int j = i + 1;
    Vec2d last_fb = prev_ra + prev_dir * ra_to_fb;
    while (j < path_points.size()) {
      const auto& p_j = path_points[j];
      const Vec2d pt(p_j.x(), p_j.y());
      const Vec2d dir = Vec2d::FastUnitFromAngle(p_j.theta());
      const double lat_err = prev_dir.CrossProd(pt + dir * ra_to_fb - prev_ra);
      // Build path segment from i to j - 1 if the lateral error of j too large.
      if (std::abs(lat_err) > tolerance) {
        const Vec2d prev_rb = prev_ra + prev_dir * ra_to_rb;
        const Vec2d last_ra(path_points[j - 1].x(), path_points[j - 1].y());
        segments.push_back(
            PathSegment(i, j - 1, prev_ra, last_ra, path_points[i].s(),
                        path_points[j - 1].s(),
                        Box2d(Segment2d(prev_rb, last_fb), box_width)));
        prev_dir = dir;
        prev_ra = pt;
        break;
      }
      last_fb = pt + dir * ra_to_fb;
      ++j;
    }
    // j reached the end, we should build the last segment.
    if (j >= path_points.size()) {
      const Vec2d prev_rb = prev_ra + prev_dir * ra_to_rb;
      const Vec2d tail_ra(path_points[j - 1].x(), path_points[j - 1].y());
      segments.push_back(
          PathSegment(i, j - 1, prev_ra, tail_ra, path_points[i].s(),
                      path_points[j - 1].s(),
                      Box2d(Segment2d(prev_rb, last_fb), box_width)));
    }
    i = j;
  }
  return PathApprox(std::move(segments), path_kd_tree);
}

}  // namespace planner
}  // namespace qcraft
