#ifndef ONBOARD_PLANNER_SPEED_VT_POINT_H_
#define ONBOARD_PLANNER_SPEED_VT_POINT_H_

#include <ostream>
#include <string>

#include "absl/strings/str_format.h"

#include "onboard/math/vec.h"

namespace qcraft::planner {

class VtPoint {
  // x-axis: t; y-axis: v.
 public:
  VtPoint() = default;

  VtPoint(double v, double t) : v_(v), t_(t) {}

  double v() const { return v_; }

  double t() const { return t_; }

  std::string DebugString() const {
    return absl::StrFormat("{v:%f, t:%f}", v(), t());
  }

 private:
  double v_ = 0.0;
  double t_ = 0.0;
};

inline std::ostream& operator<<(std::ostream& os, const VtPoint& point) {
  return os << point.DebugString();
}

inline bool operator==(const VtPoint& lhs, const VtPoint& rhs) {
  return lhs.v() == rhs.v() && lhs.t() == rhs.t();
}

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_VT_POINT_H_
