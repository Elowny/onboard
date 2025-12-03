#include "onboard/planner/speed/svt_point.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(SvtPointTest, Ctor) {
  const double s = 1.0;
  const double v = 2.0;
  const double t = 3.0;
  constexpr double kEps = 1e-6;
  const SvtPoint svt_point(s, v, t);

  EXPECT_NEAR(svt_point.s(), s, kEps);
  EXPECT_NEAR(svt_point.v(), v, kEps);
  EXPECT_NEAR(svt_point.t(), t, kEps);

  EXPECT_EQ(svt_point.DebugString(),
            "{ t : 3.000000, v : 2.000000, s : 1.000000 }");
}

}  // namespace
}  // namespace qcraft::planner
