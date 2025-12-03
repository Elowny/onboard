#include "onboard/planner/initializer/geometry/geometry_form.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/test_util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft::planner {
namespace {
TEST(StraightLineGeometry, SamplePoints) {
  const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(10.0, 0.0));
  const auto state = line.State(/*s=*/1.0);
  EXPECT_THAT(state.xy, Vec2dNearXY(1.0, 0.0, 1e-6));
  const auto states = line.Sample(/*delta_s=*/1.0);
  EXPECT_EQ(state.h, 0.0);
  EXPECT_EQ(state.k, 0.0);

  EXPECT_EQ(states.size(), 11);
  EXPECT_THAT(states.front().xy, Vec2dEqXY(0.0, 0.0));
  EXPECT_EQ(states.front().h, 0.0);
  EXPECT_EQ(states.front().k, 0.0);

  EXPECT_THAT(states[5].xy, Vec2dNearXY(5.0, 0.0, 1e-6));
  EXPECT_EQ(states[5].h, 0.0);
  EXPECT_EQ(states[5].k, 0.0);

  EXPECT_THAT(states.back().xy, Vec2dEqXY(10.0, 0.0));
  EXPECT_EQ(states.back().h, 0.0);
  EXPECT_EQ(states.front().k, 0.0);

  EXPECT_EQ(line.states().size(), 101);
}

TEST(PiecewiseLinearGeometry, SamplePoints) {
  const GeometryState state0({.xy = Vec2d(0.0, 1.0), .h = 3.0, .k = 4.0});
  const GeometryState state1({.xy = Vec2d(0.0, 3.0), .h = 4.0, .k = 4.0});

  const PiecewiseLinearGeometry polyline({state0, state1});
  EXPECT_THAT(polyline.State(0.0).xy, Vec2dEqXY(0.0, 1.0));
  EXPECT_EQ(polyline.State(0.0).h, 3.0);
  EXPECT_EQ(polyline.State(0.0).k, 4.0);

  auto states = polyline.Sample(0.5);
  EXPECT_EQ(states.size(), 5);
  EXPECT_THAT(states[0].xy, Vec2dEqXY(0.0, 1.0));
  EXPECT_EQ(states[0].h, 3.0);
  EXPECT_EQ(states[0].k, 4.0);
  EXPECT_THAT(states[2].xy, Vec2dEqXY(0.0, 2.0));
  EXPECT_EQ(states[2].h, NormalizeAngle(3.5));
  EXPECT_EQ(states[2].k, 4.0);
  EXPECT_THAT(states[4].xy, Vec2dEqXY(0.0, 3.0));
  EXPECT_EQ(states[4].h, NormalizeAngle(4.0));
  EXPECT_EQ(states[4].k, 4.0);
}

TEST(StationaryGeometry, SamplePoints) {
  const GeometryState state({.xy = Vec2d(1.0, 2.0), .h = 3.0, .k = 4.0});
  const StationaryGeometry point(state);

  EXPECT_NEAR(point.length(), 0.0, 1e-3);

  EXPECT_THAT(point.State(0.0).xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_EQ(point.State(0.0).h, 3.0);
  EXPECT_EQ(point.State(0.0).k, 4.0);

  EXPECT_THAT(point.FastState(0.0).xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_EQ(point.FastState(0.0).h, 3.0);
  EXPECT_EQ(point.FastState(0.0).k, 4.0);

  auto states = point.Sample(0.1);
  EXPECT_EQ(states.size(), 1);
  EXPECT_THAT(states[0].xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_EQ(states[0].h, 3.0);
  EXPECT_EQ(states[0].k, 4.0);

  EXPECT_EQ(point.states().size(), 1);
  EXPECT_THAT(point.states()[0].xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_EQ(point.states()[0].h, 3.0);
  EXPECT_EQ(point.states()[0].k, 4.0);
}

}  // namespace
}  // namespace qcraft::planner
