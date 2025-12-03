#include "onboard/planner/selector/selector_util.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {
TEST(SelectorUtilTest, LinearInterpolateTest) {
  {
    double result = LinearInterpolate(0.0, 1.0, 2.0, 4.0, 0.5);
    double expected = -0.75;
    EXPECT_NEAR(expected, result, 1e-6);
  }

  {
    double result = LinearInterpolate(-10.0, 10.0, -5.0, 5.0, 2.5);
    double expected = 5.0;
    EXPECT_NEAR(expected, result, 1e-6);
  }

  {
    double result = LinearInterpolate(0.0, 100.0, 10.0, 20.0, 15.0);
    double expected = 50.0;
    EXPECT_NEAR(expected, result, 1e-6);
  }

  {
    double result = LinearInterpolate(0.0, 0.0, 10.0, 10.0, 5.0);
    double expected = 0.0;
    EXPECT_NEAR(expected, result, 1e-6);
  }

  {
    double result = LinearInterpolate(1.5, 2.5, 0.0, 1.0, 0.75);
    double expected = 2.25;
    EXPECT_NEAR(expected, result, 1e-6);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
