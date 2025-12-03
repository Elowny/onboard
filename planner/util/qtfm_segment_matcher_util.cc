#include "onboard/planner/util/qtfm_segment_matcher_util.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "glog/logging.h"

namespace qcraft::planner {

namespace qtfm_util {
bool SegmentRatioRangeInHalfPlane(const Vec2d& p0, const Vec2d& p1,
                                  const Segment2d& hp, bool reverse,
                                  double* start, double* end) {
  constexpr double kEpsilon = 1e-9;
  DCHECK(start != nullptr);
  DCHECK(end != nullptr);

  double r0 = hp.ProductOntoUnit(p0);
  double r1 = hp.ProductOntoUnit(p1);
  if (reverse) {
    r0 = -r0;
    r1 = -r1;
  }
  const double r01 = r1 - r0;
  if (std::abs(r01) < kEpsilon) {
    if (r0 >= 0.0) {
      // all in.
      *start = 0.0;
      *end = 1.0;
      return true;
    } else {
      // all out.
      return false;
    }
  }
  const double critical_ratio = -r0 / r01;
  if (r01 > 0) {
    if (r1 < 0.0) {
      return false;
    } else {
      // r1 in.
      *end = 1.0;
      *start = std::max(0., critical_ratio);
      return true;
    }
  } else {
    if (r0 < 0.0) {
      return false;
    } else {
      // r0 in.
      *start = 0.0;
      *end = std::min(1.0, critical_ratio);
      return true;
    }
  }
}

bool SegmentIntersectsFanRegion(const Vec2d& p0, const Vec2d& p1,
                                const Segment2d& right_bound,
                                const Segment2d& left_bound) {
  double rbl_start, rbl_end, lbr_start, lbr_end;
  if (!SegmentRatioRangeInHalfPlane(p0, p1, right_bound, /*reverse=*/false,
                                    &rbl_start, &rbl_end)) {
    // No segment part in right bound left.
    return false;
  }
  if (!SegmentRatioRangeInHalfPlane(p0, p1, left_bound, /*reverse=*/true,
                                    &lbr_start, &lbr_end)) {
    // No segment part in left bound right.
    return false;
  }

  return rbl_end >= lbr_start && lbr_end >= rbl_start;
}

bool NoReverseRegion(const AABox2d& box, const Segment2d& prev_div,
                     const Segment2d& cur_div) {
  // p0,p1,p2,p3: four points of the box.
  const std::array<Vec2d, 4> points = box.GetAllCorners();
  for (int i = 0; i < 4; ++i) {
    const int next_i = (i + 1) % 4;
    if (SegmentIntersectsFanRegion(points[i], points[next_i], prev_div,
                                   cur_div)) {
      return false;
    }
  }
  return true;
}

}  // namespace qtfm_util

}  // namespace qcraft::planner
