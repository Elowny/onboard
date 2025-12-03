#ifndef ONBOARD_PLANNER_SPEED_SPEED_POINT_H_
#define ONBOARD_PLANNER_SPEED_SPEED_POINT_H_

#include <string>

#include "absl/strings/str_format.h"

#include "onboard/math/vec.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"

namespace qcraft::planner {

/**
 * @class SpeedPoint
 * @brief Implements a class of SpeedPoint
 */
class SpeedPoint {
 public:
  SpeedPoint() = default;

  SpeedPoint(double t, double s, double v, double a, double j)
      : t_(t), s_(s), v_(v), a_(a), j_(j) {}

  virtual ~SpeedPoint() = default;

  double t() const { return t_; }

  void set_t(double t) { t_ = t; }

  double s() const { return s_; }

  void set_s(double s) { s_ = s; }

  double v() const { return v_; }

  void set_v(double v) { v_ = v; }

  double a() const { return a_; }

  void set_a(double a) { a_ = a; }

  double j() const { return j_; }

  void set_j(double j) { j_ = j; }

  void FromProto(const SpeedPointProto& proto) {
    t_ = proto.t();
    s_ = proto.s();
    v_ = proto.v();
    a_ = proto.a();
    j_ = proto.j();
  }
  void ToProto(SpeedPointProto* proto) const {
    proto->set_t(t_);
    proto->set_s(s_);
    proto->set_v(v_);
    proto->set_a(a_);
    proto->set_j(j_);
  }

  std::string DebugString() const {
    return absl::StrFormat(
        "{ t : %.6f, s : %.6f, v : %.6f, a : %.6f, j : %.6f }", t(), s(), v(),
        a(), j());
  }

 private:
  double t_ = 0.0;
  double s_ = 0.0;
  double v_ = 0.0;
  double a_ = 0.0;
  double j_ = 0.0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_POINT_H_
