#include "onboard/planner/test_util/predicted_trajectory_builder.h"

#include <cmath>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(PredictedTrajectoryBuilder, Build) {
  const Vec2d start(0.0, 0.0);
  const Vec2d end(10.0, 10.0);
  const double init_v = 10.0;  // m/s
  const double last_v = 20.0;  // m/s

  PredictedTrajectoryBuilder builder;
  const auto traj = builder.set_annotation("123")
                        .set_type(PredictionType::PT_PED_KINEMATIC)
                        .set_probability(0.5)
                        .set_straight_line(start, end, init_v, last_v)
                        .Build();

  EXPECT_EQ(traj.annotation(), "123");
  EXPECT_EQ(traj.probability(), 0.5);
  EXPECT_EQ(traj.type(), PredictionType::PT_PED_KINEMATIC);

  const double theta = M_PI * 0.25;
  const double length = 10.0 * std::sqrt(2.0);
  const double time = length / 15.0;
  const double a = (20.0 - 10.0) / time;
  double t = 0.0;
  double s = 0.0;
  for (const auto& pt : traj.points()) {
    constexpr double kEpsilon = 1e-3;
    EXPECT_NEAR(pt.theta(), theta, kEpsilon);
    EXPECT_NEAR(pt.t(), t, kEpsilon);
    t += 0.1;
    EXPECT_NEAR(pt.a(), a, kEpsilon);
    EXPECT_GE(pt.s(), s);
    s = pt.s();
    EXPECT_NEAR(pt.pos().x(), pt.pos().y(), kEpsilon);
    EXPECT_EQ(pt.kappa(), 0.0);
  }
}

}  // namespace
}  // namespace qcraft::planner
