#include "onboard/planner/speed/speed_vector.h"

#include <algorithm>
#include <utility>

#include "onboard/math/util.h"

namespace qcraft::planner {

SpeedVector::SpeedVector(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  std::stable_sort(begin(), end(),
                   [](const SpeedPoint& p1, const SpeedPoint& p2) {
                     return p1.t() < p2.t();
                   });
}

std::optional<SpeedPoint> SpeedVector::EvaluateByTime(double t) const {
  if (size() < 2) {
    return std::nullopt;
  }

  if (t > back().t() || t < front().t()) {
    return std::nullopt;
  }

  const auto it_lower = std::lower_bound(
      begin(), end(), t,
      [](const SpeedPoint& sp, double t) { return sp.t() < t; });

  if (it_lower == end()) return back();
  if (it_lower == begin()) return front();
  const auto& p0 = *(it_lower - 1);
  const auto& p1 = *it_lower;
  const double t0 = p0.t();
  const double t1 = p1.t();
  const double alpha = LerpFactor(t0, t1, t);
  return SpeedPoint(t, Lerp(p0.s(), p1.s(), alpha), Lerp(p0.v(), p1.v(), alpha),
                    Lerp(p0.a(), p1.a(), alpha), Lerp(p0.j(), p1.j(), alpha));
}

std::optional<SpeedPoint> SpeedVector::EvaluateByS(double s) const {
  if (size() < 2) {
    return std::nullopt;
  }
  if (s > back().s() || s < front().s()) {
    return std::nullopt;
  }

  const auto it_lower = std::lower_bound(
      begin(), end(), s,
      [](const SpeedPoint& sp, double s) { return sp.s() < s; });

  if (it_lower == end()) return back();
  if (it_lower == begin()) return front();
  const auto& p0 = *(it_lower - 1);
  const auto& p1 = *it_lower;
  const double s0 = p0.s();
  const double s1 = p1.s();
  const double alpha = LerpFactor(s0, s1, s);
  return SpeedPoint(Lerp(p0.t(), p1.t(), alpha), s, Lerp(p0.v(), p1.v(), alpha),
                    Lerp(p0.a(), p1.a(), alpha), Lerp(p0.j(), p1.j(), alpha));
}

double SpeedVector::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}

double SpeedVector::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

}  // namespace qcraft::planner
