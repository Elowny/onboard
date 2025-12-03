#include "onboard/planner/common/discretized_path.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/path_util.h"

namespace qcraft {
namespace planner {

DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points)
    : std::vector<PathPoint>(std::move(path_points)) {
  for (auto it = begin(); it + 1 < end(); ++it) {
    DCHECK_LE(it->s(), (it + 1)->s())
        << "at path point[" << it - begin() << "]";
  }
  QCHECK_EQ(begin()->s(), 0.0);
}

FrenetCoordinate DiscretizedPath::XYToSL(const Vec2d& pos) const {
  // We require the path to have at least two points in order to use XYToSL.
  QCHECK_GT(size(), 1);
  constexpr double kLengthEps = 1e-9;                   // m.
  constexpr double kLengthEpsInv = 1.0 / (kLengthEps);  // 1/m.
  {
    absl::MutexLock lock(&sl_query_info_mutex_);
    if (sl_query_info_.points.empty()) {
      sl_query_info_.points.reserve(size());
      sl_query_info_.s_knots.reserve(size());
      sl_query_info_.length_invs.reserve(size() - 1);
      sl_query_info_.tangents.reserve(size() - 1);
      sl_query_info_.points.emplace_back(begin()->x(), begin()->y());
      sl_query_info_.s_knots.push_back(begin()->s());
      for (auto iter = begin() + 1; iter < end(); ++iter) {
        const Vec2d segment =
            Vec2d(iter->x(), iter->y()) - sl_query_info_.points.back();
        const double length_inv = 1.0 / segment.norm();
        sl_query_info_.points.emplace_back(iter->x(), iter->y());
        sl_query_info_.s_knots.push_back(iter->s());
        sl_query_info_.length_invs.push_back(length_inv);
        sl_query_info_.tangents.push_back(length_inv > kLengthEpsInv
                                              ? Vec2d(0.0, 0.0)
                                              : segment * length_inv);
      }
    }
  }

  double min_d = std::numeric_limits<double>::infinity();
  double l = 0.0;
  double s = 0.0;
  for (int i = 1; i < sl_query_info_.points.size(); ++i) {
    double curr_l = 0.0;
    double curr_s = 0.0;
    const Vec2d p0 = sl_query_info_.points[i - 1];
    const double s0 = sl_query_info_.s_knots[i - 1];
    const double length_inv = sl_query_info_.length_invs[i - 1];
    if (length_inv > kLengthEpsInv) {
      curr_l = pos.DistanceTo(p0);
      curr_s = s0;
    } else {
      const Vec2d p1 = sl_query_info_.points[i];
      const double s1 = sl_query_info_.s_knots[i];
      const Vec2d heading_vec = sl_query_info_.tangents[i - 1];
      const Vec2d query_segment = pos - p0;
      const double projection = query_segment.dot(heading_vec) * length_inv;
      const double production = heading_vec.CrossProd(query_segment);
      if (projection < 0.0 && i > 1) {
        const double sign = production < 0.0 ? -1.0 : 1.0;
        curr_l = p0.DistanceTo(pos) * sign;
        curr_s = s0;
      } else if (projection > 1.0 && i + 1 < sl_query_info_.points.size()) {
        const double sign = production < 0.0 ? -1.0 : 1.0;
        curr_l = p1.DistanceTo(pos) * sign;
        curr_s = s1;
      } else {
        curr_l = production;
        curr_s = Lerp(s0, s1, projection);
      }
    }
    if (std::fabs(curr_l) < min_d) {
      min_d = std::fabs(curr_l);
      l = curr_l;
      s = curr_s;
    }
  }

  return FrenetCoordinate{.s = s, .l = l};
}

PathPoint DiscretizedPath::Evaluate(double path_s) const {
  QCHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }

  PathPoint path_point;
  if ((it_lower - 1)->s() == it_lower->s()) {
    path_point = *(it_lower - 1);
  } else {
    const double alpha =
        (path_s - (it_lower - 1)->s()) / (it_lower->s() - (it_lower - 1)->s());
    LerpPathPoint(*(it_lower - 1), *it_lower, alpha, &path_point);
  }

  return path_point;
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    double path_s) const {
  auto func = [](const PathPoint& tp, double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

PathPoint DiscretizedPath::EvaluateReverse(double path_s) const {
  QCHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }

  PathPoint path_point;
  if ((it_upper - 1)->s() == it_upper->s()) {
    path_point = *(it_upper - 1);
  } else {
    const double alpha =
        (path_s - (it_upper - 1)->s()) / (it_upper->s() - (it_upper - 1)->s());
    LerpPathPoint(*(it_upper - 1), *it_upper, alpha, &path_point);
  }

  return path_point;
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
    double path_s) const {
  auto func = [](const double path_s, const PathPoint& tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}

}  // namespace planner
}  // namespace qcraft
