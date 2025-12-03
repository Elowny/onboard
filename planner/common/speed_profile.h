#ifndef ONBOARD_PLANNER_SPEED_PROFILE_H_
#define ONBOARD_PLANNER_SPEED_PROFILE_H_

#include <algorithm>
#include <limits>
#include <ostream>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

// Piecewise const speed speed profile defined by a piecewise linear s-t graph
class SpeedProfile {
 public:
  using Plf = PiecewiseLinearFunction<double, double>;

  SpeedProfile() = default;
  explicit SpeedProfile(Plf st) { set_st(std::move(st)); }

  void set_st(Plf st) { st_ = std::move(st); }
  const Plf& st() const { return st_; }

  double GetSAtTime(double t) const { return st_.Evaluate(t); }
  double GetVAtTime(double t) const { return st_.EvaluateSlope(t); }

  double GetTimeAtS(double s) const {
    const auto& ts = st_.x();
    const auto& ss = st_.y();
    const auto it = std::lower_bound(ss.begin(), ss.end(), s);
    if (it == ss.end()) {
      // This s is never reached.
      return std::numeric_limits<double>::infinity();
    }
    if (it == ss.begin()) {
      // This s is before the first point.
      return 0.0;
    }
    const int index = it - ss.begin();
    QCHECK_GT(index, 0);
    QCHECK_LT(index, ss.size());
    QCHECK_GE(s, ss[index - 1]);
    QCHECK_LE(s, ss[index]);
    return Lerp(ts[index - 1], ts[index],
                LerpFactor(ss[index - 1], ss[index], s));
  }

 private:
  // Piecewise-linear s-t function.
  Plf st_;
};

// Piecewise const acceleration speed profile defined by a piecewise linear v-t
// graph and s-t points.
class PiecewiseAccelSpeedProfile {
 public:
  using Plf = PiecewiseLinearFunction<double, double>;

  PiecewiseAccelSpeedProfile() = default;
  // Provide v-t graph with init s
  PiecewiseAccelSpeedProfile(Plf vt, double init_s) {
    set_vt(std::move(vt));
    QCHECK_GT(vt_.x().size(), 1);
    s_.resize(vt_.x().size());
    s_[0] = init_s;
    for (int i = 0; i < vt_.x().size() - 1; ++i) {
      const double v0 = vt_.y()[i];
      const double v1 = vt_.y()[i + 1];
      const double dt = vt_.x()[i + 1] - vt_.x()[i];
      const double a = (v1 - v0) / dt;
      s_[i + 1] = s_[i] + v0 * dt + 0.5 * a * dt * dt;
    }
  }
  // Provide s-t points with init v
  PiecewiseAccelSpeedProfile(std::vector<double> s, std::vector<double> t,
                             double init_v)
      : s_(std::move(s)) {
    QCHECK_GT(s_.size(), 1);
    QCHECK_EQ(s_.size(), t.size());

    std::vector<double> v(t.size());
    v[0] = init_v;
    for (int i = 0; i < t.size() - 1; ++i) {
      const double s0 = s_[i];
      const double s1 = s_[i + 1];
      const double dt = t[i + 1] - t[i];
      v[i + 1] = 2 * (s1 - s0) / dt - v[i];
    }
    set_vt(Plf(std::move(t), std::move(v)));
  }
  // Provide v-t graph and s points
  PiecewiseAccelSpeedProfile(Plf vt, std::vector<double> s) : s_(std::move(s)) {
    set_vt(std::move(vt));
    // Sainity checks.
    QCHECK_EQ(vt_.x().size(), s_.size());
    for (int i = 0; i < vt_.x().size() - 1; ++i) {
      const double v0 = vt_.y()[i];
      const double v1 = vt_.y()[i + 1];
      const double dt = vt_.x()[i + 1] - vt_.x()[i];
      const double a = (v1 - v0) / dt;
      CHECK_NEAR(s_[i + 1], s_[i] + v0 * dt + 0.5 * a * dt * dt, 1.0e-6);
    }
  }

  void set_vt(Plf vt) { vt_ = std::move(vt); }
  const Plf& vt() const { return vt_; }

  double GetSAtTime(double t) const {
    const auto it = std::upper_bound(vt_.x().begin(), vt_.x().end(), t);
    if (it == vt_.x().begin()) {
      const double v = vt_.y()[0];
      const double dt = t - vt_.x()[0];
      return s_[0] + v * dt;
    } else if (it == vt_.x().end()) {
      const double v = vt_.y().back();
      const double dt = t - vt_.x().back();
      return s_.back() + v * dt;
    } else {
      const int index = it - vt_.x().begin() - 1;
      const double v = vt_.y()[index];
      const double a = (vt_.y()[index + 1] - vt_.y()[index]) /
                       (vt_.x()[index + 1] - vt_.x()[index]);
      const double dt = t - vt_.x()[index];
      return s_[index] + v * dt + 0.5 * a * dt * dt;
    }
  }

  double GetVAtTime(double t) const { return vt_.Evaluate(t); }

  void GetSVAAtTime(double t, double* s_t, double* v_t, double* a_t) {
    const auto it = std::upper_bound(vt_.x().begin(), vt_.x().end(), t);
    if (it == vt_.x().begin()) {
      const double v = vt_.y()[0];
      const double dt = t - vt_.x()[0];
      *a_t = 0.0;
      *v_t = v;
      *s_t = s_[0] + v * dt;
      return;
    } else if (it == vt_.x().end()) {
      const double v = vt_.y().back();
      const double dt = t - vt_.x().back();
      *a_t = 0.0;
      *v_t = v;
      *s_t = s_.back() + v * dt;
      return;
    } else {
      const int index = it - vt_.x().begin() - 1;
      const double v = vt_.y()[index];
      const double a = (vt_.y()[index + 1] - vt_.y()[index]) /
                       (vt_.x()[index + 1] - vt_.x()[index]);
      const double dt = t - vt_.x()[index];
      *a_t = a;
      *v_t = v + a * dt;
      *s_t = s_[index] + v * dt + 0.5 * a * dt * dt;
      return;
    }
  }
  // TODO(renjie): implement GetTimeAtS if necessary

 private:
  // Piecewise-linear v-t function.
  Plf vt_;
  std::vector<double> s_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_PROFILE_H_
