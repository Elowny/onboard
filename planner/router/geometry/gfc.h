#ifndef ONBOARD_PLANNER_ROUTER_GEOMETRY_GFC_H_
#define ONBOARD_PLANNER_ROUTER_GEOMETRY_GFC_H_

#include <cmath>
#include <iterator>
#include <vector>

#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"

namespace qcraft::planner::route {

constexpr double kEarthRadius = 6371118.0;  // 1/3(2Ra+Rb)
constexpr double kRadiansPerMeterLon = 1 / kEarthRadius;

template <typename T>
constexpr T Sqr(const T x) {
  return x * x;
}
struct ProjectToLineStringResult {
  Vec2d match_point = {0.0, 0.0};
  double dist_sqr = 0.0;
  int segment_index = -1;  // belongs to [0,points.size()-1]
  bool valid = false;
};

// TODO(xiang): Optimize this, No need to create Segment2d for each time
template <typename T>
double DistanceSquare(const Vec2<T>& p0, const Vec2<T>& p1, const Vec2<T>& p,
                      Vec2<T>* nearest_pt) {
  const Segment2d seg(p0, p1);
  return seg.DistanceSquareTo(p, nearest_pt);
}

// Approximate geographical distance(sqr) NOT from the Haversine formula
// @see https://en.wikipedia.org/wiki/Geographical_distance  Pythagoras’ theorem
template <typename T>
inline T DistanceSquare(const Vec2<T>& p0, const Vec2<T>& p1, T cos_latitude) {
  const auto dx = p1.x() - p0.x();
  const auto dy = p1.y() - p0.y();
  return Sqr(dx) + Sqr(dy * cos_latitude);
}

template <typename T, typename GeoPointType,
          typename std::enable_if_t<std::declval<GeoPointType>().longitude(),
                                    int> = 0>
inline decltype(auto) DistanceSquare(const Vec2<T>& p0, const GeoPointType& p1,
                                     double cos_latitude) {
  const auto dx = p1.longitude() - p0.x();
  const auto dy = p1.latitude() - p0.y();
  return Sqr(dx) + Sqr(dy * cos_latitude);
}

template <typename GeoPointType,
          typename std::enable_if_t<std::declval<GeoPointType>().longitude(),
                                    int> = 0>
inline double DistanceSquare(const GeoPointType& p0, const GeoPointType& p1,
                             double cos_latitude) {
  const double dx = p1.longitude() - p0.longitude();
  const double dy = p1.latitude() - p0.latitude();
  return Sqr(dx) + Sqr(dy * cos_latitude);
}

// @see [link https://en.wikipedia.org/wiki/Haversine_formula] , Vincenty's
// formulae
template <typename T>
inline double HaversineDistance(const Vec2<T>& p0, const Vec2<T>& p1) {
  const auto d_lon = (p1.x() - p0.x());
  const auto d_lat = (p1.y() - p0.y());
  const auto a = Sqr(std::sin(d_lat / 2)) +
                 std::cos(p1.y()) * std::cos(p0.y()) * Sqr(std::sin(d_lon / 2));
  const auto c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
  return kEarthRadius * c;
}

// Approximate calc the pt (must be on the poly line ) ratio to the points
// [first,last).
template <typename InputIt>
double CalcFraction(InputIt first, InputIt last, int index, const Vec2d& pt,
                    double cos_latitude) {
  std::vector<double> dist;
  dist.reserve(std::distance(first, last));
  dist.push_back(0.0);
  for (auto it = first; it + 1 != last; ++it) {
    dist.push_back(std::sqrt(DistanceSquare(*it, *(it + 1), cos_latitude)));
  }
  for (auto it = dist.begin(); it + 1 != dist.end(); ++it) {
    *(it + 1) += *it;
  }
  return std::clamp((dist[index] + std::sqrt(DistanceSquare(
                                       pt, *(first + index), cos_latitude))) /
                        dist.back(),
                    0.0, 1.0);
}

// TODO(xiang): Optimize this function, avoid use Segment2d and too many
// duplicated computation. segment2d_fn is a filter fn.
template <typename InputIt, typename BinaryPredicate>
ProjectToLineStringResult ProjectToLineString(InputIt first, InputIt last,
                                              const Vec2d& p,
                                              BinaryPredicate pred) {
  int min_segment_index = 0;
  constexpr auto kMaxGeometryDistanceSqr = 10;
  double min_dist_sqr = kMaxGeometryDistanceSqr;
  Vec2d nearest_pt = *first;  // project point on LineString.
  // about 0.11 meters ProjectToLineString point_to_line_string
  constexpr auto kMinDistanceSquareThreshold = Sqr(kRadiansPerMeterLon * 0.11);
  int i = 0;
  bool valid = false;
  for (auto it = first; it + 1 != last; ++it, ++i) {
    if (!pred(*it, *(it + 1))) {
      continue;
    }
    Vec2d pt;
    const auto dist_sqr = DistanceSquare(*it, *(it + 1), p, &pt);
    if (dist_sqr < min_dist_sqr) {
      nearest_pt = pt;
      min_dist_sqr = dist_sqr;
      min_segment_index = i;
      valid = true;
    }
    if (min_dist_sqr <= kMinDistanceSquareThreshold) {
      break;
    }
  }
  return {
      .match_point = nearest_pt,
      .dist_sqr = min_dist_sqr,
      .segment_index = min_segment_index,
      .valid = valid,
  };
}

inline ProjectToLineStringResult ProjectToLineString(
    const std::vector<Vec2d>& points, const Vec2d& p) {
  return ProjectToLineString(points.begin(), points.end(), p,
                             [](const auto&, const auto&) { return true; });
}

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_GEOMETRY_GFC_H_
