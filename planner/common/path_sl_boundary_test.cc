#include "onboard/planner/common/path_sl_boundary.h"

#include "gtest/gtest.h"

namespace qcraft::planner {

namespace {

TEST(PathSlBoundary, PathSlBoundaryQueryTest) {
  int n = 10;
  std::vector<double> s_vec, center_l, left_l, right_l, target_left_l,
      target_right_l;
  s_vec.reserve(n);
  center_l.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  std::vector<Vec2d> center_xy, left_xy, right_xy, target_left_xy,
      target_right_xy;
  center_xy.reserve(n);
  left_xy.reserve(n);
  right_xy.reserve(n);

  const double step_s = 1.0;
  for (double s = 0.0; s < step_s * n - step_s * 0.1; s += step_s) {
    s_vec.emplace_back(s);
    center_l.emplace_back(0.0);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    center_xy.emplace_back(s, 0.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }
  left_l[4] = 3.0;
  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;
  PathSlBoundary path_bound(
      std::move(s_vec), std::move(center_l), std::move(right_l),
      std::move(left_l), std::move(target_right_l), std::move(target_left_l),
      std::move(center_xy), std::move(right_xy), std::move(left_xy),
      std::move(target_right_xy), std::move(target_left_xy));

  // Query test 1: interpolation
  const auto [right_l_1, left_l_1] = path_bound.QueryBoundaryL(4.5);
  EXPECT_NEAR(left_l_1, 2.5, 0.01);
  EXPECT_NEAR(right_l_1, -2.0, 0.01);
}

}  // namespace

}  // namespace qcraft::planner
