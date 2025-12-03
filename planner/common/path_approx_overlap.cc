#include "onboard/planner/common/path_approx_overlap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>
// IWYU pragma: no_include "Eigen/Core"

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d_util.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_approx.h"

namespace qcraft {
namespace planner {

namespace {

// Approximates the polygon as a circle and quickly check if we need to compute
// overlap.
bool MaybeOverlap(const Box2d& box, const Polygon2d& polygon,
                  double max_lat_dist) {
  if (Sqr(box.radius() + polygon.CircleRadius() + max_lat_dist) <
      polygon.CircleCenter().DistanceSquareTo(box.center())) {
    return false;
  }
  if (std::abs(box.tangent().CrossProd(polygon.CircleCenter() - box.center())) >
      max_lat_dist + box.half_width() + polygon.CircleRadius()) {
    return false;
  }
  if (std::abs(box.tangent().Dot(polygon.CircleCenter() - box.center())) >
      max_lat_dist + box.half_length() + polygon.CircleRadius()) {
    return false;
  }
  return true;
}

AgentOverlap ConvertToAgentOverlap(
    const PathApprox& path_approx, int segment_index,
    const polygon2d::PolygonBoxOverlap& overlap) {
  const auto& segment = path_approx.segment(segment_index);
  double start_ra_s =
      std::clamp(segment.last_s() - (segment.length() - overlap.in),
                 segment.first_s(), segment.last_s());
  double last_ra_s =
      std::clamp(segment.first_s() + overlap.out, start_ra_s, segment.last_s());
  return AgentOverlap{.first_ra_s = start_ra_s,
                      .last_ra_s = last_ra_s,
                      .ra_heading = segment.heading(),
                      .lat_dist = overlap.lat_dist};
}

double ConvertToHalfPlaneMinRaS(const PathSegment& path_segment, double in) {
  return std::clamp(path_segment.last_s() - (path_segment.length() - in),
                    path_segment.first_s(), path_segment.last_s());
}

bool ClampAgentOverlap(double low, double high, AgentOverlap* overlap) {
  if (low > overlap->last_ra_s || high < overlap->first_ra_s) {
    return false;
  }
  overlap->first_ra_s = std::max(overlap->first_ra_s, low);
  overlap->last_ra_s = std::min(overlap->last_ra_s, high);
  return true;
}

std::optional<std::pair<int, int>> ComputeMinMaxIndexNearPoint(
    const SegmentMatcherKdtree& kd_tree, const Vec2d& search_point,
    double search_radius, int first_index, int last_index) {
  const auto indices = kd_tree.GetSegmentIndexInRadius(
      search_point.x(), search_point.y(), search_radius);
  if (indices.empty()) return std::nullopt;
  const auto [min_index, max_index] =
      std::minmax_element(indices.begin(), indices.end());
  first_index = std::clamp(*min_index, first_index, last_index);
  last_index = std::clamp(*max_index, first_index, last_index);
  return std::pair<int, int>(first_index, last_index);
}

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeadingHelper(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double theta,
    double max_heading_diff) {
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;

  std::vector<AgentOverlap> agent_overlaps;
  double min_abs_dist = std::numeric_limits<double>::max();

  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    polygon2d::PolygonBoxOverlap geom_overlap;
    const auto& path_segment = path_approx.segment(i);
    const double heading_diff = NormalizeAngle(theta - path_segment.heading());
    if (std::abs(heading_diff) > max_heading_diff) continue;
    if (lat_buffer == 0.0 && lon_buffer == 0.0) {
      if (!MaybeOverlap(path_segment, polygon, max_lat_dist)) continue;
      geom_overlap = polygon2d::ComputePolygonBoxOverlap(polygon, path_segment);
    } else {
      const Box2d extended_path_segment(
          path_segment.half_length() + lon_buffer,
          path_segment.half_width() + lat_buffer, path_segment.center(),
          path_segment.heading(), path_segment.tangent());
      if (!MaybeOverlap(extended_path_segment, polygon, max_lat_dist)) continue;
      geom_overlap =
          polygon2d::ComputePolygonBoxOverlap(polygon, extended_path_segment);
      geom_overlap.in -= lon_buffer;
      geom_overlap.out -= lon_buffer;
    }

    if (geom_overlap.in == geom_overlap.out && geom_overlap.lat_dist == 0.0) {
      continue;
    }

    // Skip if this is not the laterally nearest overlap.
    if (std::abs(geom_overlap.lat_dist) > min_abs_dist) continue;

    min_abs_dist = std::abs(geom_overlap.lat_dist);

    auto agent_overlap = ConvertToAgentOverlap(path_approx, i, geom_overlap);

    if (!ClampAgentOverlap(first_s, last_s, &agent_overlap)) continue;

    agent_overlaps.push_back(agent_overlap);
  }

  // Prune all overlaps that is larger than min_abs_dist.
  agent_overlaps.erase(
      std::remove_if(agent_overlaps.begin(), agent_overlaps.end(),
                     [min_abs_dist](const auto& o) {
                       return std::abs(o.lat_dist) > min_abs_dist;
                     }),
      agent_overlaps.end());
  return agent_overlaps;
}
}  // namespace

std::vector<AgentOverlap> ComputeAgentOverlapsWithBuffer(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
    } else {
      return {};
    }
  }

  return ComputeAgentOverlapsWithBufferAndHeadingHelper(
      path_approx, step_length, first_index, last_index, polygon, max_lat_dist,
      lat_buffer, lon_buffer, /*theta=*/0.0,
      /*max_heading_diff=*/std::numeric_limits<double>::infinity());
}

std::vector<AgentOverlap> ComputeAgentOverlaps(const PathApprox& path_approx,
                                               double step_length,
                                               int first_index, int last_index,
                                               const Polygon2d& polygon,
                                               double max_lat_dist,
                                               double search_radius) {
  return ComputeAgentOverlapsWithBuffer(
      path_approx, step_length, first_index, last_index, polygon, max_lat_dist,
      /*lat_buffer=*/0.0, /*lon_buffer=*/0.0, search_radius);
}

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
    } else {
      return {};
    }
  }
  return ComputeAgentOverlapsWithBufferAndHeadingHelper(
      path_approx, step_length, first_index, last_index, polygon, max_lat_dist,
      lat_buffer, lon_buffer, theta, max_heading_diff);
}

bool HasPathApproxOverlapWithPolygon(const PathApprox& path_approx,
                                     double step_length, int first_index,
                                     int last_index, const Polygon2d& polygon,
                                     double search_radius) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
    } else {
      return false;
    }
  }
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;

  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    const auto& path_segment = path_approx.segment(i);
    if (!MaybeOverlap(path_segment, polygon, /*max_lat_dist=*/0.0)) continue;
    const auto geom_overlap =
        polygon2d::ComputePolygonBoxOverlap(polygon, path_segment);
    const AgentOverlap agent_overlap =
        ConvertToAgentOverlap(path_approx, i, geom_overlap);
    if (geom_overlap.in != geom_overlap.out &&
        agent_overlap.first_ra_s <= last_s &&
        agent_overlap.last_ra_s >= first_s) {
      return true;
    }
  }
  return false;
}

std::optional<double> ComputeHalfPlaneOverlaps(const PathApprox& path_approx,
                                               double step_length,
                                               int first_index, int last_index,
                                               const HalfPlane& half_plane,
                                               double search_radius) {
  return ComputeHalfPlaneOverlapsWithLateralGap(
      path_approx, step_length, first_index, last_index, half_plane,
      /*lateral_gap=*/0.0, search_radius);
}

std::optional<double> ComputeHalfPlaneOverlapsWithLateralGap(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const HalfPlane& half_plane, double lateral_gap,
    double search_radius) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), half_plane.center(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
    } else {
      return std::nullopt;
    }
  }
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;

  const auto maybe_overlap_with_half_plane = [](const Box2d& path_segment,
                                                const HalfPlane& half_plane) {
    const AABox2d aabox(half_plane.start(), half_plane.end());
    return aabox.HasOverlap(path_segment.aabox());
  };

  std::optional<double> min_ra_s;
  const Segment2d half_plane_seg(half_plane.start(), half_plane.end());

  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    const auto& path_segment = path_approx.segment(i);
    Vec2d first, last;
    if (lateral_gap == 0.0) {
      if (!maybe_overlap_with_half_plane(path_segment, half_plane)) continue;
      if (!Polygon2d(path_segment).GetOverlap(half_plane_seg, &first, &last)) {
        continue;
      }
    } else {
      const Box2d extended_path_segment(
          path_segment.half_length(), path_segment.half_width() + lateral_gap,
          path_segment.center(), path_segment.heading(),
          path_segment.tangent());
      if (!maybe_overlap_with_half_plane(extended_path_segment, half_plane)) {
        continue;
      }
      if (!Polygon2d(extended_path_segment)
               .GetOverlap(half_plane_seg, &first, &last)) {
        continue;
      }
    }
    const HalfPlane center_hp(path_segment.RearCenterPoint(),
                              path_segment.FrontCenterPoint(),
                              path_segment.tangent());
    const double first_lon = center_hp.lon_proj(first);
    const double last_lon = center_hp.lon_proj(last);
    const double in =
        std::clamp(std::min(first_lon, last_lon), 0.0, path_segment.length());
    const auto curr_min_ra_s = ConvertToHalfPlaneMinRaS(path_segment, in);
    if (!InRange(curr_min_ra_s, first_s, last_s)) continue;
    min_ra_s = curr_min_ra_s;
    break;
  }
  return min_ra_s;
}
}  // namespace planner
}  // namespace qcraft
