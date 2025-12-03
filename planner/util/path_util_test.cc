#include "onboard/planner/util/path_util.h"

#include <vector>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
constexpr double kEpsilon = 1e-7;
constexpr double kEps = 1e-4;
PathPoint MakeDefaultPathPoint() {
  PathPoint path_point;
  path_point.set_x(0.0);
  path_point.set_y(0.0);
  path_point.set_z(1.0);
  path_point.set_theta(0.0);
  path_point.set_kappa(1.0);
  path_point.set_lambda(0.0);
  path_point.set_s(0.0);
  return path_point;
}

TEST(PlannerPathUtilTest, PathPointUtilTest) {
  qcraft::PathPoint path_point;
  path_point.set_x(0.0);
  path_point.set_y(0.0);
  path_point.set_z(1.0);
  path_point.set_theta(0.0);
  path_point.set_kappa(0.0);
  path_point.set_lambda(0.0);
  path_point.set_s(0.0);
  EXPECT_EQ(ToVec2d(path_point), qcraft::Vec2d(0.0, 0.0));

  qcraft::PathPoint from;
  from.set_x(0.0);
  from.set_y(0.0);
  from.set_z(1.0);
  from.set_theta(0.0);
  from.set_kappa(0.0);
  from.set_lambda(0.0);
  from.set_s(0.0);

  qcraft::PathPoint to;
  to.set_x(0.0);
  to.set_y(0.0);
  to.set_z(1.0);
  to.set_theta(0.0);
  to.set_kappa(0.0);
  to.set_lambda(0.0);
  to.set_s(0.0);
  EXPECT_EQ(DistanceTo(from, to), 0.0);
  EXPECT_EQ(Heading(from, to), qcraft::Vec2d(0.0, 0.0));
  EXPECT_NE(Heading(from, to), qcraft::Vec2d(0.0, 5e-5));

  to.set_y(5e-5);
  EXPECT_NE(Heading(from, to), qcraft::Vec2d(0.0, 0.0));
  EXPECT_EQ(Heading(from, to), qcraft::Vec2d(0.0, 1.0));

  to.set_y(-1.0);
  EXPECT_EQ(DistanceTo(from, to), 1.0);
  EXPECT_EQ(Heading(from, to), qcraft::Vec2d(0.0, -1.0));
}

TEST(GetPathPointAlongCircle, PositiveInputFabsSLessThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  PathPoint to = GetPathPointAlongCircle(from, /*s=*/5e-7);
  EXPECT_EQ(from.x(), to.x());
  EXPECT_EQ(from.y(), to.y());
  EXPECT_EQ(from.z(), to.z());
  EXPECT_EQ(from.theta(), to.theta());
  EXPECT_EQ(from.kappa(), to.kappa());
  EXPECT_EQ(from.lambda(), to.lambda());
  EXPECT_EQ(from.s(), to.s());
}

TEST(GetPathPointAlongCircle, ZeroInputFabsSLessThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  PathPoint to = GetPathPointAlongCircle(from, /*s=*/0.0);
  EXPECT_EQ(from.x(), to.x());
  EXPECT_EQ(from.y(), to.y());
  EXPECT_EQ(from.z(), to.z());
  EXPECT_EQ(from.theta(), to.theta());
  EXPECT_EQ(from.kappa(), to.kappa());
  EXPECT_EQ(from.lambda(), to.lambda());
  EXPECT_EQ(from.s(), to.s());
}

TEST(GetPathPointAlongCircle, NegativeInputFabsSLessThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  PathPoint to = GetPathPointAlongCircle(from, /*s=*/-4e-7);
  EXPECT_EQ(from.x(), to.x());
  EXPECT_EQ(from.y(), to.y());
  EXPECT_EQ(from.z(), to.z());
  EXPECT_EQ(from.theta(), to.theta());
  EXPECT_EQ(from.kappa(), to.kappa());
  EXPECT_EQ(from.lambda(), to.lambda());
  EXPECT_EQ(from.s(), to.s());
}

TEST(GetPathPointAlongCircle, PositiveInputFabsPointKappaLessThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  from.set_kappa(1e-8);
  PathPoint end_point = GetPathPointAlongCircle(from, /*s=*/10.0 * M_PI);
  EXPECT_NEAR(end_point.x(), 31.415925074, kEpsilon);
  EXPECT_NEAR(end_point.y(), -1.46203e-06, kEpsilon);
  EXPECT_NEAR(end_point.z(), 0.0, kEpsilon);
  EXPECT_NEAR(end_point.theta(), 3.14159e-07, kEpsilon / 10.0);
  EXPECT_NEAR(end_point.kappa(), 1e-08, kEpsilon / 10.0);
  EXPECT_NEAR(end_point.lambda(), 0.0, kEpsilon);
  EXPECT_NEAR(end_point.s(), 31.415926536, kEpsilon);
}

TEST(GetPathPointAlongCircle, ZeroInputFabsPointKappaLessThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  from.set_x(10.0);
  from.set_y(10.0);
  from.set_kappa(0.0);
  PathPoint end_point = GetPathPointAlongCircle(from, /*s=*/10.0 * M_PI);

  EXPECT_NEAR(end_point.x(), 41.415925074, kEps);
  EXPECT_NEAR(end_point.y(), 10, kEps);
  EXPECT_NEAR(end_point.z(), 0.0, kEps);
  EXPECT_NEAR(end_point.theta(), 0.0, kEps);
  EXPECT_NEAR(end_point.kappa(), 0.0, kEps);
  EXPECT_NEAR(end_point.lambda(), 0.0, kEps);
  EXPECT_NEAR(end_point.s(), 31.415926536, kEps);
}

TEST(GetPathPointAlongCircle, NegativeInputFabsPointKappaLessThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  from.set_x(10.0);
  from.set_y(10.0);
  from.set_theta(0.5 * M_PI);
  from.set_kappa(-4e-7);
  PathPoint end_point = GetPathPointAlongCircle(from, /*s=*/10.0 * M_PI);

  EXPECT_NEAR(end_point.x(), 10, kEps);
  EXPECT_NEAR(end_point.y(), 41.4159, kEps);
  EXPECT_NEAR(end_point.z(), 0.0, kEps);
  EXPECT_NEAR(end_point.theta(), 1.57078, kEps);
  EXPECT_NEAR(end_point.kappa(), 0.0, kEps);
  EXPECT_NEAR(end_point.lambda(), 0.0, kEps);
  EXPECT_NEAR(end_point.s(), 31.4159, kEps);
}

TEST(GetPathPointAlongCircle, PositiveInputFabsPointKappaMoreThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  from.set_kappa(0.01);
  PathPoint end_point = GetPathPointAlongCircle(from, /*s=*/10.0 * M_PI);

  EXPECT_NEAR(end_point.x(), 30.9017, kEps);
  EXPECT_NEAR(end_point.y(), 4.89435, kEps);
  EXPECT_NEAR(end_point.z(), 0.0, kEps);
  EXPECT_NEAR(end_point.theta(), 0.314159, kEps);
  EXPECT_NEAR(end_point.kappa(), 0.01, kEps);
  EXPECT_NEAR(end_point.lambda(), 0.0, kEps);
  EXPECT_NEAR(end_point.s(), 31.4159, kEps);
}

TEST(GetPathPointAlongCircle, InputFabsPointKappaEqualKEps) {
  PathPoint from = MakeDefaultPathPoint();
  from.set_x(10.0);
  from.set_y(10.0);
  from.set_kappa(1e-6);
  PathPoint end_point = GetPathPointAlongCircle(from, /*s=*/10.0 * M_PI);

  EXPECT_NEAR(end_point.x(), 41.4159, kEps);
  EXPECT_NEAR(end_point.y(), 10.0005, kEps);
  EXPECT_NEAR(end_point.z(), 0.0, kEps);
  EXPECT_NEAR(end_point.theta(), 3.14159e-05, kEpsilon);
  EXPECT_NEAR(end_point.kappa(), 0.0, kEps);
  EXPECT_NEAR(end_point.lambda(), 0.0, kEps);
  EXPECT_NEAR(end_point.s(), 31.4159, kEps);
}

TEST(GetPathPointAlongCircle, NegativeInputFabsPointKappaMoreThanKEps) {
  PathPoint from = MakeDefaultPathPoint();
  from.set_x(10.0);
  from.set_y(10.0);
  from.set_theta(0.5 * M_PI);
  from.set_kappa(-0.02);
  PathPoint end_point = GetPathPointAlongCircle(from, /*s=*/10.0 * M_PI);

  EXPECT_NEAR(end_point.x(), 19.5492, kEps);
  EXPECT_NEAR(end_point.y(), 39.3893, kEps);
  EXPECT_NEAR(end_point.z(), 0.0, kEps);
  EXPECT_NEAR(end_point.theta(), 0.942478, kEps);
  EXPECT_NEAR(end_point.kappa(), -0.02, kEps);
  EXPECT_NEAR(end_point.lambda(), 0.0, kEps);
  EXPECT_NEAR(end_point.s(), 31.4159, kEps);
}

}  // namespace
}  // namespace qcraft::planner
