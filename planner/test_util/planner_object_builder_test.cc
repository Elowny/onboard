#include "onboard/planner/test_util/planner_object_builder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/test_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

TEST(PlannerObjectBuilder, Build) {
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(2.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d::Zero())
                                  .Build();
  PlannerObjectBuilder builder;

  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/1.0, /*last_v=*/1.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(Vec2d::Zero(), Vec2d(20.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);

  const PlannerObject object = builder.Build();

  EXPECT_EQ(object.id(), "abc");
  // NOTE(lidong): `PlannerObject` did not copy `ObjectProto`'s time in
  // `PlannerObject::FromObjectProto`.
  EXPECT_EQ(object.pose().t(), 0.0);
  EXPECT_EQ(object.proto_timestamp(), 1.0);
  EXPECT_EQ(object.prediction().id(), "abc");
  EXPECT_EQ(object.prediction().trajectories().size(), 2);
  EXPECT_THAT(object.prediction().trajectories().front().points().front().pos(),
              Vec2dNearXY(0.0, 0.0, 1e-8));
  EXPECT_THAT(object.prediction().trajectories().front().points().back().pos(),
              Vec2dNearXY(10.0, 0.0, 1e-8));
  EXPECT_EQ(object.prediction().trajectories().front().probability(), 0.5);

  EXPECT_THAT(object.prediction().trajectories().back().points().front().pos(),
              Vec2dNearXY(0.0, 0.0, 1e-8));
  EXPECT_THAT(object.prediction().trajectories().back().points().back().pos(),
              Vec2dNearXY(20.0, 0.0, 1e-8));
  EXPECT_EQ(object.prediction().trajectories().back().probability(), 0.3);
}
}  // namespace planner
}  // namespace qcraft
