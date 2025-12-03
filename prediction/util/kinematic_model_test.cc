#include "onboard/prediction/util/kinematic_model.h"

#include <cmath>

#include "gtest/gtest.h"

namespace qcraft::prediction {
namespace {

constexpr double kEpsilon = 1e-3;

TEST(kinematicmodelTest, SimulateUniCycleModelTest1) {
  const UniCycleState prev_state{.x = 0.0,
                                 .y = 0.0,
                                 .v = 0.0,
                                 .heading = 0.0,
                                 .yaw_rate = 0.0,
                                 .acc = 0.0};

  const auto cur_state = SimulateUniCycleModel(prev_state, /*dt=*/1.0);

  EXPECT_NEAR(cur_state.x, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.y, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.v, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.heading, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.yaw_rate, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.acc, 0.0, kEpsilon);
}

TEST(kinematicmodelTest, SimulateUniCycleModelTest2) {
  const UniCycleState prev_state{.x = 1.0,
                                 .y = 1.0,
                                 .v = 1.0,
                                 .heading = acos(-1) / 2,
                                 .yaw_rate = 0.0,
                                 .acc = 0.0};

  const auto cur_state = SimulateUniCycleModel(prev_state, /*dt=*/1.0);

  EXPECT_NEAR(cur_state.x, 1.0, kEpsilon);
  EXPECT_NEAR(cur_state.y, 2.0, kEpsilon);
  EXPECT_NEAR(cur_state.v, 1.0, kEpsilon);
  EXPECT_NEAR(cur_state.heading, acos(-1) / 2, kEpsilon);
  EXPECT_NEAR(cur_state.yaw_rate, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.acc, 0.0, kEpsilon);
}

TEST(kinematicmodelTest, SimulateBicycleModelTest1) {
  const BicycleModelState prev_state{.x = 0.0,
                                     .y = 0.0,
                                     .v = 0.0,
                                     .heading = 0.0,
                                     .acc = 0.0,
                                     .front_wheel_angle = 0.0};

  const auto cur_state =
      SimulateBicycleModel(prev_state, /*lf=*/1.0, /*lr=*/1.0, /*dt=*/1.0);

  EXPECT_NEAR(cur_state.x, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.y, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.v, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.heading, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.acc, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.front_wheel_angle, 0.0, kEpsilon);
}

TEST(kinematicmodelTest, SimulateBicycleModelTest2) {
  const BicycleModelState prev_state{.x = 1.0,
                                     .y = 1.0,
                                     .v = 1.0,
                                     .heading = acos(-1) / 2,
                                     .acc = 0.0,
                                     .front_wheel_angle = 0.0};

  const auto cur_state =
      SimulateBicycleModel(prev_state, /*lf=*/1.0, /*lr=*/1.0, /*dt=*/1.0);

  EXPECT_NEAR(cur_state.x, 1.0, kEpsilon);
  EXPECT_NEAR(cur_state.y, 2.0, kEpsilon);
  EXPECT_NEAR(cur_state.v, 1.0, kEpsilon);
  EXPECT_NEAR(cur_state.heading, acos(-1) / 2, kEpsilon);
  EXPECT_NEAR(cur_state.acc, 0.0, kEpsilon);
  EXPECT_NEAR(cur_state.front_wheel_angle, 0.0, kEpsilon);
}

}  // namespace
}  // namespace qcraft::prediction
