#include "onboard/planner/router/thirdparty/coordinate_transformer.h"

#include <cstddef>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/util.h"

namespace qcraft::planner::route {

namespace {
TEST(CoordinateTransformer, Wgs84ToGcj02Test) {
  CoordinateTransformer ct;
  Vec2d origin = {0, 0};
  auto tencent_pt = ct.Wgs84ToGcj02(origin);
  auto gps = ct.Gcj02ToWgs84(tencent_pt);
  EXPECT_NEAR(r2d(origin.x()), r2d(gps.x()), 1E-7);
  EXPECT_NEAR(r2d(origin.y()), r2d(gps.y()), 1E-7);
  std::vector<Vec2d> gps_points = {{2., 0.5}, {2.1, 0.51}, {2.11, 0.511}};
  const auto tencent_points = ct.Wgs84ToGcj02(gps_points);
  const auto gps_points2 = ct.Gcj02ToWgs84(tencent_points);
  EXPECT_EQ(gps_points.size(), gps_points2.size());
  for (std::size_t i = 0; i < gps_points.size(); ++i) {
    EXPECT_NEAR(r2d(gps_points[i].x()), r2d(gps_points[i].x()), 1E-7);
    EXPECT_NEAR(r2d(gps_points[i].y()), r2d(gps_points[i].y()), 1E-7);
  }
}
}  // namespace
}  // namespace qcraft::planner::route
