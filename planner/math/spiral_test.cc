#include "onboard/planner/math/spiral.h"

#include <vector>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
TEST(Spiral, StraightLine) {
  const std::vector<double> params0 = {0.0, 0.0, 0.0, 0.0};
  const Spiral sp0(/*x=*/0.0, /*y=*/0.0, /*theta=*/0.0, /*sg=*/1.0,
                   /*params=*/params0);
  const auto sp_end = sp0.Eval(/*s=*/1.0, /*num_steps=*/8);
  EXPECT_LT(std::fabs(sp_end.x - 1.0), 1e-3);
  EXPECT_LT(std::fabs(sp_end.y - 0.0), 1e-3);
  EXPECT_LT(std::fabs(sp_end.k - 0.0), 1e-6);
  EXPECT_LT(std::fabs(sp_end.theta - 0.0), 1e-6);
}

TEST(Spiral, RotatedStraightLine) {
  const std::vector<double> params0 = {0.0, 0.0, 0.0, 0.0};
  const Spiral sp0(/*x=*/0.0, /*y=*/0.0, /*theta=*/M_PI / 4.0, /*sg=*/1.0,
                   /*params=*/params0);
  const auto sp_end = sp0.Eval(/*s=*/1.0, /*num_steps=*/8);
  EXPECT_LT(std::fabs(sp_end.x - std::sqrt(2.0) / 2.0), 1e-3);
  EXPECT_LT(std::fabs(sp_end.y - std::sqrt(2.0) / 2.0), 1e-3);
  EXPECT_LT(std::fabs(sp_end.k - 0.0), 1e-6);
  EXPECT_LT(std::fabs(sp_end.theta - M_PI / 4.0), 1e-6);
}

TEST(Spiral, Circle) {
  const std::vector<double> params0 = {0.0, 0.0, 0.0, 1.0};
  const Spiral sp0(/*x=*/0.0, /*y=*/0.0, /*theta=*/0.0, /*sg=*/M_PI / 2.0,
                   /*params=*/params0);
  const auto sp_end = sp0.Eval(/*s=*/M_PI / 2.0, /*num_steps=*/8);
  EXPECT_LT(std::fabs(sp_end.x - 1.0), 1e-2);
  EXPECT_LT(std::fabs(sp_end.y - 1.0), 1e-2);
  EXPECT_LT(std::fabs(sp_end.k - 1.0), 1e-6);
  EXPECT_LT(std::fabs(sp_end.theta - M_PI / 2.0), 1e-6);
}

TEST(Spiral, RotatedCircle) {
  const std::vector<double> params0 = {0.0, 0.0, 0.0, 1.0};
  const Spiral sp0(/*x=*/0.0, /*y=*/0.0, /*theta=*/M_PI / 2.0, /*sg=*/M_PI,
                   /*params=*/params0);
  const auto sp_end = sp0.Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(std::fabs(sp_end.x - (-2.0)), 5e-2);
  EXPECT_LT(std::fabs(sp_end.y - 0.0), 5e-2);
  EXPECT_LT(std::fabs(sp_end.k - 1.0), 1e-6);
  EXPECT_LT(std::fabs(std::sin(sp_end.theta - (-M_PI / 2.0))), 1e-6);
}
}  // namespace
}  // namespace qcraft::planner
