#include "onboard/planner/router/thirdparty/coordinate_transformer.h"

#include <algorithm>

#include "onboard/math/geometry/gcj_transform.h"
namespace qcraft::planner::route {

Vec2d CoordinateTransformer::Wgs84ToGcj02(const Vec2d& pt) const {
  return WgsToGcj(pt);
}

std::vector<Vec2d> CoordinateTransformer::Wgs84ToGcj02(
    const std::vector<Vec2d>& points) const {
  std::vector<Vec2d> result(points.size());
  std::transform(points.begin(), points.end(), result.begin(), WgsToGcj);
  return result;
}

Vec2d CoordinateTransformer::Gcj02ToWgs84(const Vec2d& pt) const {
  return GcjToWgsExact(pt);
}

std::vector<Vec2d> CoordinateTransformer::Gcj02ToWgs84(
    const std::vector<Vec2d>& points) const {
  std::vector<Vec2d> result(points.size());
  std::transform(points.begin(), points.end(), result.begin(), GcjToWgsExact);
  return result;
}

}  // namespace qcraft::planner::route
