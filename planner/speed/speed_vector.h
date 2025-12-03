#ifndef ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_
#define ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_

#include <functional>
#include <optional>
#include <vector>

#include "onboard/math/util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_point.h"

namespace qcraft::planner {

class SpeedVector : public std::vector<SpeedPoint> {
 public:
  SpeedVector() = default;

  explicit SpeedVector(std::vector<SpeedPoint> speed_points);
  __attribute__((always_inline)) inline void EvaluateByTime(
      double start_time, double step,
      const std::function<bool(const SpeedPoint&, double t)>& callback) const {
    if (size() < 2) {
      return;
    }
    const double totalval = TotalTime();
    if (start_time < front().t() || start_time > back().t()) {
      return;
    }
    auto it = begin();
    while (it->t() >= start_time) {
      if (callback(*it, start_time)) return;
      start_time += step;
      if (start_time > totalval) return;
    }
    while (++it != end()) {
      while (it->t() >= start_time) {
        const auto& p0 = *(it - 1);
        const auto& p1 = *it;
        const double t0 = p0.t();
        const double t1 = p1.t();
        const double alpha = LerpFactor(t0, t1, start_time);
        if (callback(SpeedPoint(start_time, Lerp(p0.s(), p1.s(), alpha),
                                Lerp(p0.v(), p1.v(), alpha),
                                Lerp(p0.a(), p1.a(), alpha),
                                Lerp(p0.j(), p1.j(), alpha)),
                     start_time)) {
          return;
        }
        start_time += step;
        if (start_time > totalval) return;
      }
    }
    while (it->t() >= start_time) {
      if (callback(*it, start_time)) return;
      start_time += step;
      if (start_time > totalval) return;
    }
  }

  std::optional<SpeedPoint> EvaluateByTime(double t) const;

  std::optional<SpeedPoint> EvaluateByS(double s) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  // Use container as a template to make it compatible with protobuf's repeated
  // field and vector.
  template <template <class...> class Container>
  void FromProto(const Container<SpeedPointProto>& speed_points) {
    reserve(speed_points.size());
    for (const auto& speed_point : speed_points) {
      emplace_back().FromProto(speed_point);
    }
    std::stable_sort(begin(), end(),
                     [](const SpeedPoint& p1, const SpeedPoint& p2) {
                       return p1.t() < p2.t();
                     });
  }

  void ToProto(SpeedPointsProto* speed_points) const {
    speed_points->Clear();
    for (auto it = begin(); it != end(); ++it) {
      it->ToProto(speed_points->add_speed_points());
    }
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_
