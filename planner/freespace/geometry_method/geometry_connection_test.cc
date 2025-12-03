#include "onboard/planner/freespace/geometry_method/geometry_connection.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kMaxKappa = 0.2;
constexpr double kEpsilon = 1e-4;

TEST(GeometryConnectionTest, CircleCircleConectionTest) {
  GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                               Vec2d::FastUnitFromAngle(0.0)};
  GeometryMethodPoint goal = {Vec2d(7.0, 2.0), 0.0,
                              Vec2d::FastUnitFromAngle(0.0)};

  LineCirclePath result;
  ASSERT_TRUE(CircleCircleConection(start, goal, kMaxKappa, &result));
  ASSERT_TRUE(result.ends.back().pos.DistanceTo(goal.pos) < kEpsilon);
  ASSERT_TRUE(std::abs(NormalizeAngle(result.ends.back().theta - goal.theta)) <
              kEpsilon);
}

TEST(GeometryConnectionTest, CircleLineConectionTest) {
  GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0 * M_PI,
                               Vec2d::FastUnitFromAngle(0.0 * M_PI)};
  GeometryMethodPoint goal = {Vec2d(7.0, 3.0), 0.3 * M_PI,
                              Vec2d::FastUnitFromAngle(0.3 * M_PI)};

  LineCirclePath result;
  ASSERT_TRUE(CircleLineConection(start, goal, kMaxKappa, &result));
  ASSERT_TRUE(result.ends.back().pos.DistanceTo(goal.pos) < kEpsilon);
  ASSERT_TRUE(std::abs(NormalizeAngle(result.ends.back().theta - goal.theta)) <
              kEpsilon);
}

TEST(GeometryConnectionTest, LineCircleLineConectionTest) {
  GeometryMethodPoint start = {Vec2d(-3.0, 6.0), M_PI,
                               Vec2d::FastUnitFromAngle(M_PI)};
  GeometryMethodPoint goal = {Vec2d(0.0, 0.0), 0.5 * M_PI,
                              Vec2d::FastUnitFromAngle(0.5 * M_PI)};

  LineCirclePath result;
  ASSERT_TRUE(LineCircleLineConection(start, goal, kMaxKappa, &result));
  ASSERT_TRUE(result.ends.back().pos.DistanceTo(goal.pos) < kEpsilon);
  ASSERT_TRUE(std::abs(NormalizeAngle(result.ends.back().theta - goal.theta)) <
              kEpsilon);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
