#include "onboard/planner/common/circle_path_overlap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

constexpr double kInf = std::numeric_limits<double>::infinity();

namespace {

inline double InverseMonotonicCosine(double cosine) {
  QCHECK_LE(cosine, 1.0);
  if (cosine < -3.0) {
    return kInf;
  } else if (cosine > -1.0) {
    return std::acos(cosine);
  } else {
    return 2.0 * M_PI - std::acos(-2.0 - cosine);
  }
}

double SegmentFirstOverlapMonotonicCosine(const Segment2d& segment,
                                          const Vec2d& center, bool ccw,
                                          const Segment2d& other_segment) {
  if (segment.HasIntersect(other_segment)) return 1.0;
  double res = -kInf;
  res = std::max(res, MonotonicCosineOfPointOverlapWithSegment(
                          segment.start(), center, ccw, other_segment));
  res = std::max(res, MonotonicCosineOfPointOverlapWithSegment(
                          segment.end(), center, ccw, other_segment));
  res = std::max(res, MonotonicCosineOfPointOverlapWithSegment(
                          other_segment.start(), center, !ccw, segment));
  res = std::max(res, MonotonicCosineOfPointOverlapWithSegment(
                          other_segment.end(), center, !ccw, segment));
  return res;
}

double SegmentFirstOverlapMonotonicCosine(const Segment2d& segment,
                                          const Vec2d& center, bool ccw,
                                          const Box2d& box) {
  double max_cosine = -kInf;
  const auto corners = box.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      segment, center, ccw, Segment2d(corners[0], corners[1])));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      segment, center, ccw, Segment2d(corners[1], corners[2])));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      segment, center, ccw, Segment2d(corners[2], corners[3])));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      segment, center, ccw, Segment2d(corners[3], corners[0])));
  return max_cosine;
}

double SegmentFirstOverlapMonotonicCosine(const Segment2d& segment,
                                          const Vec2d& center, bool ccw,
                                          const Polygon2d& polygon) {
  double max_cosine = -kInf;
  for (const auto& line_segment : polygon.line_segments()) {
    max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                          segment, center, ccw, line_segment));
  }
  return max_cosine;
}

}  // namespace

double MonotonicCosineOfPointOverlapWithSegment(const Vec2d& point,
                                                const Vec2d& center, bool ccw,
                                                const Segment2d& segment) {
  constexpr double kEps = 1.0e-6;
  if (segment.IsPointIn(point)) return 1.0;
  const double radius_sqr = center.DistanceSquareTo(point);
  if (radius_sqr < kEps) return -kInf;
  const double signed_dist =
      (center - segment.start()).CrossProd(segment.unit_direction());
  if (Sqr(signed_dist) > radius_sqr) return -kInf;
  // Get two intersections of line and circle.
  const Vec2d chrod_mid =
      center + signed_dist * segment.unit_direction().Perp();
  const Vec2d vec =
      std::sqrt(radius_sqr - Sqr(signed_dist)) * segment.unit_direction();
  const Vec2d intersect_1 = chrod_mid + vec;
  const Vec2d intersect_2 = chrod_mid - vec;

  const double radius_sqr_inv = 1.0 / radius_sqr;
  // Get the cosine when rotating from start to point.
  const auto get_cosine_between_points = [&point, &center, &radius_sqr_inv,
                                          &ccw](const Vec2d& start) {
    const Segment2d segment(point, start);
    const double center_dist_to_chord =
        (center - point).CrossProd(segment.unit_direction());
    double res = 2.0 * Sqr(center_dist_to_chord) * radius_sqr_inv - 1.0;
    if (((start - center).CrossProd(point - center) < 0.0) ^ ccw) {
      res = -2.0 - res;
    }
    return res;
  };

  const auto is_on_segment = [&segment](const Vec2d& pt) {
    return (pt - segment.start()).Dot(pt - segment.end()) <= 0.0;
  };

  // Compute cosine.
  double res = -kInf;
  if (is_on_segment(intersect_1)) {
    res = get_cosine_between_points(intersect_1);
  }
  if (is_on_segment(intersect_2)) {
    res = std::max(res, get_cosine_between_points(intersect_2));
  }
  return res;
}

// Segment2d.
double SegmentFirstOverlapAngle(const Segment2d& segment, const Vec2d& center,
                                bool ccw, const Segment2d& other_segment) {
  return InverseMonotonicCosine(
      SegmentFirstOverlapMonotonicCosine(segment, center, ccw, other_segment));
}

double SegmentFirstOverlapAngle(const Segment2d& segment, const Vec2d& center,
                                bool ccw, const Box2d& box) {
  return InverseMonotonicCosine(
      SegmentFirstOverlapMonotonicCosine(segment, center, ccw, box));
}

double SegmentFirstOverlapAngle(const Segment2d& segment, const Vec2d& center,
                                bool ccw, const Polygon2d& polygon) {
  return InverseMonotonicCosine(
      SegmentFirstOverlapMonotonicCosine(segment, center, ccw, polygon));
}

// Box2d.
double BoxFirstOverlapAngle(const Box2d& box, const Vec2d& center, bool ccw,
                            const Segment2d& segment) {
  double max_cosine = -kInf;
  const auto corners = box.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[0], corners[1]), center, ccw, segment));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[1], corners[2]), center, ccw, segment));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[2], corners[3]), center, ccw, segment));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[3], corners[0]), center, ccw, segment));
  return InverseMonotonicCosine(max_cosine);
}

double BoxFirstOverlapAngle(const Box2d& box, const Vec2d& center, bool ccw,
                            const Box2d& other_box) {
  double max_cosine = -kInf;
  const auto corners = box.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                        Segment2d(corners[0], corners[1]),
                                        center, ccw, other_box));
  max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                        Segment2d(corners[1], corners[2]),
                                        center, ccw, other_box));
  max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                        Segment2d(corners[2], corners[3]),
                                        center, ccw, other_box));
  max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                        Segment2d(corners[3], corners[0]),
                                        center, ccw, other_box));
  return InverseMonotonicCosine(max_cosine);
}

double BoxFirstOverlapAngle(const Box2d& box, const Vec2d& center, bool ccw,
                            const Polygon2d& polygon) {
  double max_cosine = -kInf;
  const auto corners = box.GetCornersCounterClockwise();
  QCHECK_EQ(corners.size(), 4);
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[0], corners[1]), center, ccw, polygon));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[1], corners[2]), center, ccw, polygon));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[2], corners[3]), center, ccw, polygon));
  max_cosine = std::max(
      max_cosine, SegmentFirstOverlapMonotonicCosine(
                      Segment2d(corners[3], corners[0]), center, ccw, polygon));
  return InverseMonotonicCosine(max_cosine);
}

// Polygon2d.
double PolygonFirstOverlapAngle(const Polygon2d& polygon, const Vec2d& center,
                                bool ccw, const Segment2d& segment) {
  double max_cosine = -kInf;
  for (const auto& line_segment : polygon.line_segments()) {
    max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                          line_segment, center, ccw, segment));
  }
  return InverseMonotonicCosine(max_cosine);
}

double PolygonFirstOverlapAngle(const Polygon2d& polygon, const Vec2d& center,
                                bool ccw, const Box2d& box) {
  double max_cosine = -kInf;
  for (const auto& line_segment : polygon.line_segments()) {
    max_cosine = std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                          line_segment, center, ccw, box));
  }
  return InverseMonotonicCosine(max_cosine);
}

double PolygonFirstOverlapAngle(const Polygon2d& polygon, const Vec2d& center,
                                bool ccw, const Polygon2d& other_polygon) {
  double max_cosine = -kInf;
  for (const auto& line_segment : polygon.line_segments()) {
    max_cosine =
        std::max(max_cosine, SegmentFirstOverlapMonotonicCosine(
                                 line_segment, center, ccw, other_polygon));
  }
  return InverseMonotonicCosine(max_cosine);
}

}  // namespace planner
}  // namespace qcraft
