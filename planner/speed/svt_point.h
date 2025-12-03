#ifndef ONBOARD_PLANNER_SPEED_SVT_POINT_H_
#define ONBOARD_PLANNER_SPEED_SVT_POINT_H_

#include <string>

#include "absl/strings/str_format.h"

namespace qcraft::planner {

class SvtPoint {
 public:
  SvtPoint() = default;

  SvtPoint(double s, double v, double t) : s_(s), v_(v), t_(t) {}

  double s() const { return s_; }

  double v() const { return v_; }

  double t() const { return t_; }

  std::string DebugString() const {
    return absl::StrFormat("{ t : %.6f, v : %.6f, s : %.6f }", t_, v_, s_);
  }

 private:
  double s_ = 0.0;
  double v_ = 0.0;
  double t_ = 0.0;
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SVT_POINT_H_
