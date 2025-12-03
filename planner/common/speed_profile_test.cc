#include "onboard/planner/common/speed_profile.h"

#include <vector>

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {

TEST(SpeedProfileTest, Test) {
  PiecewiseLinearFunction<double, double> st({0.0, 1.0, 2.0}, {3.0, 4.0, 6.0});
  SpeedProfile speed_profile(std::move(st));
  constexpr double kEps = 1.0e-6;

  EXPECT_NEAR(speed_profile.GetSAtTime(-1.0), 3.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.5), 3.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.5), 5.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.5), 6.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(-1.0), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.5), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.0), 2.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.5), 2.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.0), 2.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.5), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetTimeAtS(2.0), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetTimeAtS(3.0), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetTimeAtS(3.5), 0.5, kEps);
  EXPECT_NEAR(speed_profile.GetTimeAtS(4.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetTimeAtS(5.0), 1.5, kEps);
  EXPECT_NEAR(speed_profile.GetTimeAtS(6.0), 2.0, kEps);
}

TEST(PiecewiseAccelSpeedProfile, Test_1) {
  PiecewiseLinearFunction<double, double> vt({0.0, 1.0, 2.0}, {1.0, 2.0, 4.0});
  double init_s = 1.0;
  PiecewiseAccelSpeedProfile speed_profile(std::move(vt), init_s);
  constexpr double kEps = 1.0e-6;

  EXPECT_NEAR(speed_profile.GetSAtTime(-1.0), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.5), 1.625, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.0), 2.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.5), 3.75, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.0), 5.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.5), 7.5, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(-1.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.5), 1.5, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.0), 2.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.5), 3.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.0), 4.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.5), 4.0, kEps);
}

TEST(PiecewiseAccelSpeedProfile, Test_2) {
  PiecewiseAccelSpeedProfile speed_profile({1.0, 2.5, 5.5}, {0.0, 1.0, 2.0},
                                           1.0);
  constexpr double kEps = 1.0e-6;

  EXPECT_NEAR(speed_profile.GetSAtTime(-1.0), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.5), 1.625, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.0), 2.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.5), 3.75, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.0), 5.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.5), 7.5, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(-1.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.5), 1.5, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.0), 2.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.5), 3.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.0), 4.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.5), 4.0, kEps);
}

TEST(PiecewiseAccelSpeedProfile, Test_3) {
  PiecewiseLinearFunction<double, double> vt({0.0, 1.0, 2.0}, {1.0, 2.0, 4.0});
  std::vector<double> s = {1.0, 2.5, 5.5};
  PiecewiseAccelSpeedProfile speed_profile(std::move(vt), std::move(s));
  constexpr double kEps = 1.0e-6;

  EXPECT_NEAR(speed_profile.GetSAtTime(-1.0), 0.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(0.5), 1.625, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.0), 2.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(1.5), 3.75, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.0), 5.5, kEps);
  EXPECT_NEAR(speed_profile.GetSAtTime(2.5), 7.5, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(-1.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.0), 1.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(0.5), 1.5, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.0), 2.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(1.5), 3.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.0), 4.0, kEps);
  EXPECT_NEAR(speed_profile.GetVAtTime(2.5), 4.0, kEps);
}

TEST(PiecewiseAccelSpeedProfile, Test_4) {
  PiecewiseLinearFunction<double, double> vt({0.0, 1.0, 2.0}, {1.0, 2.0, 4.0});
  std::vector<double> s = {1.0, 2.5, 5.5};
  PiecewiseAccelSpeedProfile speed_profile(std::move(vt), std::move(s));
  constexpr double kEps = 1.0e-6;

  double ss, v, a;
  speed_profile.GetSVAAtTime(-1.0, &ss, &v, &a);
  EXPECT_NEAR(ss, 0.0, kEps);
  EXPECT_NEAR(v, 1.0, kEps);
  EXPECT_NEAR(a, 0.0, kEps);
  speed_profile.GetSVAAtTime(0.0, &ss, &v, &a);
  EXPECT_NEAR(ss, 1.0, kEps);
  EXPECT_NEAR(v, 1.0, kEps);
  EXPECT_NEAR(a, 1.0, kEps);
  speed_profile.GetSVAAtTime(0.5, &ss, &v, &a);
  EXPECT_NEAR(ss, 1.625, kEps);
  EXPECT_NEAR(v, 1.5, kEps);
  EXPECT_NEAR(a, 1.0, kEps);
  speed_profile.GetSVAAtTime(1.0, &ss, &v, &a);
  EXPECT_NEAR(ss, 2.5, kEps);
  EXPECT_NEAR(v, 2.0, kEps);
  EXPECT_NEAR(a, 2.0, kEps);
  speed_profile.GetSVAAtTime(1.5, &ss, &v, &a);
  EXPECT_NEAR(ss, 3.75, kEps);
  EXPECT_NEAR(v, 3.0, kEps);
  EXPECT_NEAR(a, 2.0, kEps);
  speed_profile.GetSVAAtTime(2.0, &ss, &v, &a);
  EXPECT_NEAR(ss, 5.5, kEps);
  EXPECT_NEAR(v, 4.0, kEps);
  EXPECT_NEAR(a, 0.0, kEps);
  speed_profile.GetSVAAtTime(2.5, &ss, &v, &a);
  EXPECT_NEAR(ss, 7.5, kEps);
  EXPECT_NEAR(v, 4.0, kEps);
  EXPECT_NEAR(a, 0.0, kEps);
}

}  // namespace planner
}  // namespace qcraft
