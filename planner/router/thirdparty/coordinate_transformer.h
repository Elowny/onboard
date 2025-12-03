
#ifndef ONBOARD_PLANNER_ROUTER_THIRDPARTY_COORDINATE_TRANSFORMER_H_
#define ONBOARD_PLANNER_ROUTER_THIRDPARTY_COORDINATE_TRANSFORMER_H_

#include <vector>

#include "onboard/math/vec.h"
namespace qcraft::planner::route {

class CoordinateTransformer {
 public:
  CoordinateTransformer() = default;

 public:
  Vec2d Wgs84ToGcj02(const Vec2d& pt) const;
  std::vector<Vec2d> Wgs84ToGcj02(const std::vector<Vec2d>& points) const;

  Vec2d Gcj02ToWgs84(const Vec2d& pt) const;
  std::vector<Vec2d> Gcj02ToWgs84(const std::vector<Vec2d>& points) const;
};

}  // namespace qcraft::planner::route
#endif  // ONBOARD_PLANNER_ROUTER_THIRDPARTY_COORDINATE_TRANSFORMER_H_
