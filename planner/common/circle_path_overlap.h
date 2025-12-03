#ifndef ONBOARD_PLANNER_COMMON_CIRCLE_PATH_OVERLAP_H
#define ONBOARD_PLANNER_COMMON_CIRCLE_PATH_OVERLAP_H

#include <cmath>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"

// Please refer to
// https://qcraft.feishu.cn/docs/doccnfgFN3egXvI1wKtvC1JMTfb#Adxpam.

namespace qcraft {
namespace planner {

// Defined in [0, inf), returns cos(x) in [0, pi), -2.0 - cos(x) in [pi,2pi),
// -3.0 in [2pi, inf).
inline double MonotonicCosine(double angle, bool use_fast_math) {
  QCHECK_GE(angle, 0.0);
  if (angle > 2.0 * M_PI) {
    return -3.0;
  } else if (angle < M_PI) {
    return use_fast_math ? fast_math::Cos(angle) : std::cos(angle);
  } else {
    return use_fast_math ? -2.0 - fast_math::Cos(angle)
                         : -2.0 - std::cos(angle);
  }
}

// The point is rotating around center in cw or ccw order, when it first meet
// segment, returns the extend cosine of rotate angle, or negative infinity if
// the point can't meet segment.
double MonotonicCosineOfPointOverlapWithSegment(const Vec2d& point,
                                                const Vec2d& center, bool ccw,
                                                const Segment2d& segment);

// The following functions are: When a segment/box/polygon is rotating around
// center in ccw order, return the rotating angle when it firstly meet the
// segment/box/polygon, or infinity if they can't meet.

// Segment2d.
double SegmentFirstOverlapAngle(const Segment2d& segment, const Vec2d& center,
                                bool ccw, const Segment2d& other_segment);

double SegmentFirstOverlapAngle(const Segment2d& segment, const Vec2d& center,
                                bool ccw, const Box2d& box);

double SegmentFirstOverlapAngle(const Segment2d& segment, const Vec2d& center,
                                bool ccw, const Polygon2d& polygon);

// Box2d.
double BoxFirstOverlapAngle(const Box2d& box, const Vec2d& center, bool ccw,
                            const Segment2d& segment);

double BoxFirstOverlapAngle(const Box2d& box, const Vec2d& center, bool ccw,
                            const Box2d& other_box);

double BoxFirstOverlapAngle(const Box2d& box, const Vec2d& center, bool ccw,
                            const Polygon2d& polygon);

// Polygon2d.
double PolygonFirstOverlapAngle(const Polygon2d& polygon, const Vec2d& center,
                                bool ccw, const Segment2d& segment);

double PolygonFirstOverlapAngle(const Polygon2d& polygon, const Vec2d& center,
                                bool ccw, const Box2d& box);

double PolygonFirstOverlapAngle(const Polygon2d& polygon, const Vec2d& center,
                                bool ccw, const Polygon2d& other_polygon);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_CIRCLE_PATH_OVERLAP_H
