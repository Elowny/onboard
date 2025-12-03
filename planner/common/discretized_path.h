#ifndef ONBOARD_PLANNER_DISCRETIZED_PATH_H_
#define ONBOARD_PLANNER_DISCRETIZED_PATH_H_

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

// The path points to construct a discretized-path should meet the conditions
// that 1) the s is monotonically increasing and 2) the start s is zero.
class DiscretizedPath : public std::vector<PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<PathPoint> path_points);
  DiscretizedPath(const DiscretizedPath& other)
      : std::vector<PathPoint>(other) {
    sl_query_info_ = other.sl_query_info_;
  }
  DiscretizedPath& operator=(const DiscretizedPath& other) {
    std::vector<PathPoint>::operator=(other);
    sl_query_info_ = other.sl_query_info_;
    return *this;
  }
  DiscretizedPath(DiscretizedPath&& other)
      : std::vector<PathPoint>(std::move(other)) {
    sl_query_info_ = std::move(other.sl_query_info_);
  }
  DiscretizedPath& operator=(DiscretizedPath&& other) {
    std::vector<PathPoint>::operator=(std::move(other));
    sl_query_info_ = std::move(other.sl_query_info_);
    return *this;
  }

  double length() const {
    if (empty()) {
      return 0.0;
    }
    return back().s();
  }

  FrenetCoordinate XYToSL(const Vec2d& pos) const;

  PathPoint Evaluate(double path_s) const;

  PathPoint EvaluateReverse(double path_s) const;

  static DiscretizedPath CreateResampledPath(
      std::vector<PathPoint> raw_path_points, double interval) {
    QCHECK_GE(raw_path_points.size(), 2);
    DiscretizedPath raw_path(std::move(raw_path_points));
    double s = 0.0;
    std::vector<PathPoint> path_points;
    path_points.reserve(CeilToInt(raw_path.length() / interval));
    while (s < raw_path.length()) {
      path_points.push_back(raw_path.Evaluate(s));
      s += interval;
    }
    return DiscretizedPath(std::move(path_points));
  }

 private:
  std::vector<PathPoint>::const_iterator QueryLowerBound(double path_s) const;
  std::vector<PathPoint>::const_iterator QueryUpperBound(double path_s) const;

  struct SlQueryInfo {
    std::vector<Vec2d> points;
    std::vector<double> s_knots;
    std::vector<double> length_invs;
    std::vector<Vec2d> tangents;
  };
  mutable SlQueryInfo sl_query_info_;
  mutable absl::Mutex sl_query_info_mutex_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DISCRETIZED_PATH_H_
