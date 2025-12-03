#include "onboard/planner/common/lane_path_info.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include "Eigen/Core"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"

namespace qcraft::planner {

namespace {

struct ProjectionResult {
  int index_1;  // left interpolation anchor_point index
  int index_2;  // right interpolation anchor_point index
  double accum_s;
  double signed_l;
  double lerp_factor;
};

// Find the interpolation segment and projection sl within [start_s, end_s].
ProjectionResult ProjectionToLanePath(const Vec2d& xy, double start_s,
                                      double end_s,
                                      const std::vector<Vec2d>& anchor_points,
                                      const std::vector<double>& anchor_s,
                                      const std::vector<Vec2d>& tangents,
                                      const std::vector<double>& seg_len_inv) {
  double min_d = std::numeric_limits<double>::infinity();

  ProjectionResult res;

  for (int i = 1; i < anchor_points.size(); ++i) {
    if (anchor_s[i - 1] < start_s) continue;

    const Vec2d& p0 = anchor_points[i - 1];
    const Vec2d& p1 = anchor_points[i];
    const Vec2d query_seg = xy - p0;
    const double proj =
        query_seg.Dot(tangents[i - 1]) * seg_len_inv[i - 1];  // [0, 1]
    const double prod = tangents[i - 1].CrossProd(query_seg);

    double s, l, lerp_factor;
    if (proj < 0.0 && i > 1) {
      l = std::copysign(p0.DistanceTo(xy), prod);
      s = anchor_s[i - 1];
      lerp_factor = 0.0;
    } else if (proj > 1.0 && i + 1 < anchor_points.size()) {
      l = std::copysign(p1.DistanceTo(xy), prod);
      s = anchor_s[i];
      lerp_factor = 1.0;
    } else {
      l = prod;
      lerp_factor = proj;
      s = Lerp(anchor_s[i - 1], anchor_s[i], lerp_factor);
    }

    if (std::abs(l) < min_d) {
      min_d = std::abs(l);
      res.index_1 = i - 1;
      res.index_2 = i;
      res.accum_s = s;
      res.signed_l = l;
      res.lerp_factor = lerp_factor;
    }

    if (anchor_s[i] > end_s) break;
  }

  QCHECK(!std::isinf(min_d));

  return res;
}

std::pair<int, double> GetInterpolationRange(
    const std::vector<double>& anchor_s, double s) {
  // Allow extra-polation
  const auto it = std::lower_bound(anchor_s.begin(), anchor_s.end(), s);

  if (it == anchor_s.begin()) {
    return {0, (s - *it) / (*(it + 1) - *it)};
  }
  if (it == anchor_s.end()) {
    return {anchor_s.size() - 2, (s - *(it - 2)) / (*(it - 1) - *(it - 2))};
  }

  return {it - 1 - anchor_s.begin(), (s - *(it - 1)) / (*it - *(it - 1))};
}

}  // namespace

LanePathInfo::LanePathInfo(mapping::LanePath lane_path, double len_along_route,
                           double path_cost,
                           const PlannerSemanticMapManager& psmm)
    : length_along_route_(len_along_route),
      path_cost_(path_cost),
      anchor_points_(SampleLanePathPoints(psmm, lane_path)) {
  if (anchor_points_.size() <= 1) {
    // Lane path empty or is a single point, initialize with an empty instance.
    *this = LanePathInfo();
    return;
  }

  lane_path_ = std::move(lane_path);
  anchor_s_.reserve(anchor_points_.size());
  tangents_.reserve((anchor_points_.size()) - 1);
  segment_len_inv_.reserve(anchor_points_.size() - 1);

  anchor_s_.emplace_back(0.0);
  for (int i = 1; i < anchor_points_.size(); ++i) {
    const double len = anchor_points_[i - 1].DistanceTo(anchor_points_[i]);
    segment_len_inv_.emplace_back(1.0 / len);

    const Vec2d tan = anchor_points_[i] - anchor_points_[i - 1];
    tangents_.emplace_back(tan * segment_len_inv_.back());

    anchor_s_.emplace_back(anchor_s_.back() + len);
  }
}

FrenetCoordinate LanePathInfo::ProjectionSL(const Vec2d& xy) const {
  const auto res =
      ProjectionToLanePath(xy, 0.0, anchor_s_.back(), anchor_points_, anchor_s_,
                           tangents_, segment_len_inv_);
  return {res.accum_s, res.signed_l};
}

FrenetCoordinate LanePathInfo::ProjectionSLInRange(const Vec2d& xy,
                                                   double start_s,
                                                   double end_s) const {
  const auto res = ProjectionToLanePath(xy, start_s, end_s, anchor_points_,
                                        anchor_s_, tangents_, segment_len_inv_);
  return {res.accum_s, res.signed_l};
}

Vec2d LanePathInfo::ProjectionXY(FrenetCoordinate sl) const {
  const auto [idx, interp_t] = GetInterpolationRange(anchor_s_, sl.s);
  const auto normal_vec = tangents_[idx].Perp();

  return Lerp(anchor_points_[idx], anchor_points_[idx + 1], interp_t) +
         normal_vec * sl.l;
}

}  // namespace qcraft::planner
