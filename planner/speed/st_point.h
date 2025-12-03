#ifndef ONBOARD_PLANNER_SPEED_ST_POINT_H_
#define ONBOARD_PLANNER_SPEED_ST_POINT_H_

#include <iostream>
#include <string>

#include "absl/strings/str_format.h"

#include "onboard/math/vec.h"

namespace qcraft::planner {

class StPoint {
  // x-axis: t; y-axis: s.
 public:
  StPoint() = default;

  StPoint(double s, double t) : s_(s), t_(t) {}

  double s() const { return s_; }

  double t() const { return t_; }

  void set_s(double s) { s_ = s; }

  void set_t(double t) { t_ = t; }

  std::string DebugString() const {
    return absl::StrFormat("{s:%f, t:%f}", s(), t());
  }

 private:
  double s_ = 0.0;
  double t_ = 0.0;
};

inline Vec2d ToVec2d(const StPoint& point) {
  return Vec2d(point.t(), point.s());
}

inline std::ostream& operator<<(std::ostream& os, const StPoint& point) {
  return os << point.DebugString();
}

inline bool operator==(const StPoint& lhs, const StPoint& rhs) {
  return lhs.s() == rhs.s() && lhs.t() == rhs.t();
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_POINT_H_
