#ifndef ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_UTIL_H_
#define ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_UTIL_H_

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"

namespace qcraft::planner {

namespace qtfm_util {
// hp.ProductOnToUnit(pt) >= 0. => inside half plane.
// if part of the segment p0 -> p1 is inside the halfplane,
// return true, with start, end be filled with the ratio range
// of the inside part.
// return false, if p0->p1 is out of the half-plane.
bool SegmentRatioRangeInHalfPlane(const Vec2d& p0, const Vec2d& p1,
                                  const Segment2d& hp, bool reverse,
                                  double* start, double* end);

// Fan region is defined as
// {pt \in Vec2d | right_bound.ProductOnToUnit(pt) >= 0,
// left_bound.ProductOnToUnit(pt) <= 0}
bool SegmentIntersectsFanRegion(const Vec2d& p0, const Vec2d& p1,
                                const Segment2d& right_bound,
                                const Segment2d& left_bound);

// For given box, no reverse region is defined as:
// !\exists point p \in box, s.t.
// prev_div.ProductOnToUnit(p) >= 0 && cur_div.ProductOnToUnit(p) <= 0.
// Return true when no reverse region, false otherwise.
bool NoReverseRegion(const AABox2d& box, const Segment2d& prev_div,
                     const Segment2d& cur_div);

}  // namespace qtfm_util

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_QTFM_SEGMENT_MATCHER_UTIL_H_
