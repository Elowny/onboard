#include "onboard/planner/speed/st_point.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/test_util.h"

namespace qcraft::planner {
namespace {

TEST(StPointTest, CtorAndSetter) {
  const double s = 1.0;
  const double t = 2.0;
  constexpr double kEps = 1e-6;
  StPoint st_point;
  st_point.set_s(s);
  st_point.set_t(t);

  EXPECT_NEAR(st_point.s(), s, kEps);
  EXPECT_NEAR(st_point.t(), t, kEps);

  EXPECT_EQ(st_point.DebugString(), "{s:1.000000, t:2.000000}");

  const StPoint st_point2(s, t);
  EXPECT_NEAR(st_point2.s(), s, kEps);
  EXPECT_NEAR(st_point2.t(), t, kEps);

  EXPECT_THAT(ToVec2d(st_point2), XYAre(2.0, 1.0));
}

}  // namespace
}  // namespace qcraft::planner
