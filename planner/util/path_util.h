#ifndef ONBOARD_PLANNER_UTIL_PATH_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PATH_UTIL_H_

#include <cmath>

#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

inline void LerpPathPoint(const PathPoint& p0, const PathPoint& p1,
                          double alpha, PathPoint* p) {
#define LERP_FIELD(field, lerp) \
  p->set_##field(lerp(p0.field(), p1.field(), alpha));
  LERP_FIELD(x, Lerp)
  LERP_FIELD(y, Lerp)
  LERP_FIELD(z, Lerp)
  LERP_FIELD(s, Lerp)
  LERP_FIELD(theta, LerpAngle)
  LERP_FIELD(kappa, Lerp)
  LERP_FIELD(lambda, Lerp)
#undef LERP_FIELD
}

inline PathPoint LerpPathPoint(const PathPoint& p0, const PathPoint& p1,
                               double alpha) {
  PathPoint p;
  LerpPathPoint(p0, p1, alpha, &p);
  return p;
}

inline Vec2d ToVec2d(const qcraft::PathPoint& point) {
  return Vec2d(point.x(), point.y());
}

// Distance on the x-y plane.
inline double DistanceTo(const PathPoint& from, const PathPoint& to) {
  return ToVec2d(from).DistanceTo(ToVec2d(to));
}

// Direction on the x-y plane.
inline Vec2d Heading(const PathPoint& from, const PathPoint& to) {
  return (ToVec2d(to) - ToVec2d(from)).normalized();
}

inline PathPoint GetPathPointAlongCircle(const PathPoint& point, double s) {
  constexpr double kEps = 1e-6;
  if (std::fabs(s) < kEps) {
    return point;
  }
  Vec2d pos;
  if (std::fabs(point.kappa()) < kEps) {
    pos = ToVec2d(point) + s * Vec2d::FastUnitFromAngle(point.theta());
  } else {
    const double dtheta_half = WrapAngle(s * point.kappa()) * 0.5;
    const double radius = 1.0 / point.kappa();
    pos = ToVec2d(point) +
          radius * std::sin(dtheta_half) * 2.0 *
              Vec2d::FastUnitFromAngle(point.theta() + dtheta_half);
  }
  PathPoint end_point;
  end_point.set_x(pos.x());
  end_point.set_y(pos.y());
  end_point.set_theta(point.theta() + s * point.kappa());
  end_point.set_s(point.s() + s);
  end_point.set_kappa(point.kappa());
  end_point.set_lambda(0.0);

  return end_point;
}

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_PATH_UTIL_H_
