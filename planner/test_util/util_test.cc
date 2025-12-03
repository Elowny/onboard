#include "onboard/planner/test_util/util.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(CreatePose, Works) {
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(5.0, 1.0), M_PI_4, Vec2d(4.0, 0.5));
  EXPECT_EQ(pose.timestamp(), 10.0);
  EXPECT_EQ(pose.pos_smooth().x(), 5.0);
  EXPECT_EQ(pose.pos_smooth().y(), 1.0);
  EXPECT_EQ(pose.vel_body().x(), 4.0);
  EXPECT_EQ(pose.vel_body().y(), 0.5);

  const double speed = Hypot(4.0, 0.5),
               speed_heading = M_PI_4 + std::atan2(0.5, 4.0);
  EXPECT_NEAR(pose.speed(), speed, 1e-6);
  EXPECT_NEAR(pose.vel_smooth().x(), speed * std::cos(speed_heading), 1e-6);
  EXPECT_NEAR(pose.vel_smooth().y(), speed * std::sin(speed_heading), 1e-6);
}

TEST(DefaultVehicleGeometry, Works) {
  const auto params = DefaultVehicleGeometry();
  EXPECT_EQ(params.front_edge_to_center(), 4.0);
  EXPECT_EQ(params.back_edge_to_center(), 1.0);
  EXPECT_EQ(params.left_edge_to_center(), 1.0);
  EXPECT_EQ(params.right_edge_to_center(), 1.0);
  EXPECT_EQ(params.length(), 5.0);
  EXPECT_EQ(params.width(), 2.0);
  EXPECT_EQ(params.height(), 2.2);
  EXPECT_EQ(params.min_turn_radius(), 6.0);
  EXPECT_EQ(params.wheel_base(), 3.0);
  EXPECT_EQ(params.wheel_rolling_radius(), 0.3);
}

TEST(DefaultPlannerParams, NormalTest) {
  const auto default_planner_params = DefaultPlannerParams();
  EXPECT_TRUE(default_planner_params.noa_params().has_noa_req_params());
  EXPECT_TRUE(default_planner_params.acc_params().has_speed_finder_params());
  EXPECT_TRUE(default_planner_params.acc_params().has_acc_req_params());
  EXPECT_TRUE(
      default_planner_params.acc_params().has_motion_constraint_params());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
