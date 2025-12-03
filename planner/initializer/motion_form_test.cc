#include "onboard/planner/initializer/motion_form.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/test_util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"

namespace qcraft::planner {
namespace {

constexpr double kEpsilon = 1e-1;

TEST(MotionForm, ConstAccelMotion) {
  const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                /*init_a=*/1.0, &line);
  EXPECT_NEAR(motion.duration(), 2.0, kEpsilon);
  EXPECT_EQ(motion.type(), MotionFormType::CONST_ACCEL_MOTION);

  const auto all_states = motion.SampleStates();
  const auto& states = all_states.const_interval_states;
  EXPECT_EQ(states.size(), 11);

  EXPECT_THAT(states.front().xy, Vec2dEqXY(0.0, 0.0));
  EXPECT_EQ(states.front().v, 1.0);
  EXPECT_EQ(states.front().a, 1.0);
  EXPECT_EQ(states.front().t, 0.0);

  EXPECT_THAT(states[5].xy, Vec2dNearXY(1.5, 0.0, kEpsilon));
  EXPECT_THAT(states[5].t, 1.0);
  EXPECT_THAT(states[5].a, 1.0);
  EXPECT_NEAR(states[5].v, 2.0, kEpsilon);

  EXPECT_THAT(states.back().xy, Vec2dEqXY(4.0, 0.0));
  EXPECT_NEAR(states.back().v, 3.0, kEpsilon);
  EXPECT_EQ(states.back().a, 1.0);
  EXPECT_EQ(states.back().t, 2.0);
}

TEST(MotionForm, ConstAccelMotionSampleDl) {
  const GeometryState state0({.xy = Vec2d(0.0, 0.0),
                              .h = 3.0,
                              .k = 4.0,
                              .ref_k = 0.0,
                              .accumulated_s = 0.0,
                              .l = 0.0});
  const GeometryState state1({.xy = Vec2d(0.0, 10.0),
                              .h = 4.0,
                              .k = 4.0,
                              .ref_k = 0.0,
                              .accumulated_s = 10.0,
                              .l = 1.0});
  const PiecewiseLinearGeometry geom({state0, state1});

  const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                /*init_a=*/0.0, &geom);
  EXPECT_NEAR(motion.duration(), 10.0, kEpsilon);
  const auto all_states = motion.SampleStates();
  const auto& const_time_interval_states = all_states.const_interval_states;
  const auto& equal_time_interval_states = all_states.equal_interval_states;
  EXPECT_EQ(const_time_interval_states.size(), 51);

  EXPECT_EQ(equal_time_interval_states.size(), 11);
  EXPECT_NEAR(equal_time_interval_states[5].l, 0.5, 1e-3);
  EXPECT_TRUE(equal_time_interval_states[5].dl.has_value());
  EXPECT_NEAR(*equal_time_interval_states[5].dl, 0.1, 1e-3);
  EXPECT_TRUE(equal_time_interval_states[5].ddl.has_value());
  EXPECT_NEAR(*equal_time_interval_states[5].ddl, 0.0, 1e-3);
}

TEST(MotionForm, ConstAccelMotionSampleDDDl) {
  std::vector<GeometryState> states;
  for (int i = 0; i <= 100; ++i) {
    states.push_back(GeometryState{.xy = Vec2d(0.0, i * 0.1),
                                   .h = 3.0,
                                   .k = 4.0,
                                   .ref_k = 0.0,
                                   .accumulated_s = i * 0.1,
                                   .l = 0.01 * std::pow(0.1 * i, 3)});
  }
  const PiecewiseLinearGeometry geom(states);

  const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                /*init_a=*/0.0, &geom);
  EXPECT_NEAR(motion.duration(), 10.0, kEpsilon);
  const auto all_states = motion.SampleStates();
  const auto& const_time_interval_states = all_states.const_interval_states;
  const auto& equal_time_interval_states = all_states.equal_interval_states;
  EXPECT_EQ(const_time_interval_states.size(), 51);
  for (const auto& state : const_time_interval_states) {
    LOG(INFO) << state.DebugString();
  }
  EXPECT_EQ(equal_time_interval_states.size(), 11);
  EXPECT_NEAR(equal_time_interval_states[5].l, 1.25, 1e-3);
  EXPECT_TRUE(equal_time_interval_states[5].dl.has_value());
  EXPECT_NEAR(*equal_time_interval_states[5].dl, 0.03 * 5 * 5, 1e-3);
  EXPECT_TRUE(equal_time_interval_states[5].ddl.has_value());
  EXPECT_NEAR(*equal_time_interval_states[5].ddl, 0.06 * 5, 1e-3);
  EXPECT_TRUE(equal_time_interval_states[5].dddl.has_value());
  EXPECT_NEAR(*equal_time_interval_states[5].dddl, 0.06, 1e-3);
}

TEST(MotionForm, ConstAccelStopMotion) {
  const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
  const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                /*init_a=*/-1.0, &line);
  EXPECT_NEAR(motion.duration(), 10.0, kEpsilon);
  const auto all_states = motion.SampleStates();
  const auto& states = all_states.const_interval_states;
  EXPECT_EQ(states.size(), 50);

  EXPECT_THAT(states.front().xy, Vec2dEqXY(0.0, 0.0));
  EXPECT_EQ(states.front().v, 1.0);
  EXPECT_EQ(states.front().a, -1.0);
  EXPECT_EQ(states.front().t, 0.0);

  EXPECT_THAT(states[1].xy, Vec2dNearXY(0.18, 0.0, kEpsilon));
  EXPECT_THAT(states[1].t, 0.2);
  EXPECT_THAT(states[1].a, -1.0);
  EXPECT_NEAR(states[1].v, 0.8, kEpsilon);

  EXPECT_THAT(states[49].xy, Vec2dNearXY(0.5, 0.0, kEpsilon));
  EXPECT_THAT(states[49].t, 9.9);
  EXPECT_THAT(states[49].a, 0.0);
  EXPECT_NEAR(states[49].v, 0.0, kEpsilon);

  auto stop_state = motion.State(/*t=*/8.0);
  EXPECT_THAT(stop_state.xy, Vec2dNearXY(0.5, 0.0, kEpsilon));
  EXPECT_THAT(stop_state.t, 8.0);
  EXPECT_THAT(stop_state.a, 0.0);
  EXPECT_NEAR(stop_state.v, 0.0, kEpsilon);

  stop_state = motion.State(/*t=*/0.2);
  EXPECT_THAT(stop_state.xy, Vec2dNearXY(0.18, 0.0, kEpsilon));
  EXPECT_THAT(stop_state.t, 0.2);
  EXPECT_THAT(stop_state.a, -1.0);
  EXPECT_NEAR(stop_state.v, 0.8, kEpsilon);
}

TEST(MotionForm, StationaryMotion) {
  const GeometryState state({.xy = Vec2d(1.0, 2.0), .h = 3.0, .k = 4.0});
  const StationaryGeometry geometry(state);
  const StationaryMotion motion(4.0, &geometry);
  EXPECT_EQ(motion.geometry(), &geometry);
  EXPECT_EQ(motion.duration(), 4.0);
  EXPECT_EQ(motion.type(), MotionFormType::STATIONARY_MOTION);

  {
    const auto all_states = motion.SampleStates();
    const auto& states = all_states.const_interval_states;
    EXPECT_EQ(states.size(), 21);

    EXPECT_THAT(states.front().xy, Vec2dEqXY(1.0, 2.0));
    EXPECT_EQ(states.front().v, 0.0);
    EXPECT_EQ(states.front().a, 0.0);
    EXPECT_EQ(states.front().t, 0.0);

    EXPECT_THAT(states.back().xy, Vec2dEqXY(1.0, 2.0));
    EXPECT_NEAR(states.back().v, 0.0, kEpsilon);
    EXPECT_EQ(states.back().a, 0.0);
    EXPECT_EQ(states.back().t, 4.0);
  }
  {
    const auto states = motion.SampleEqualIntervalStates();
    EXPECT_EQ(states.size(), 9);

    EXPECT_THAT(states.front().xy, Vec2dEqXY(1.0, 2.0));
    EXPECT_EQ(states.front().v, 0.0);
    EXPECT_EQ(states.front().a, 0.0);
    EXPECT_EQ(states.front().t, 0.0);

    EXPECT_THAT(states.back().xy, Vec2dEqXY(1.0, 2.0));
    EXPECT_NEAR(states.back().v, 0.0, kEpsilon);
    EXPECT_EQ(states.back().a, 0.0);
    EXPECT_EQ(states.back().t, 4.0);
  }

  const auto start_state = motion.GetStartMotionState();
  EXPECT_THAT(start_state.xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_NEAR(start_state.v, 0.0, kEpsilon);
  EXPECT_EQ(start_state.a, 0.0);
  EXPECT_EQ(start_state.t, 0.0);

  const auto end_state = motion.GetEndMotionState();
  EXPECT_THAT(end_state.xy, Vec2dEqXY(1.0, 2.0));
  EXPECT_NEAR(end_state.v, 0.0, kEpsilon);
  EXPECT_EQ(end_state.a, 0.0);
  EXPECT_EQ(end_state.t, 4.0);
}

}  // namespace
}  // namespace qcraft::planner
