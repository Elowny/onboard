#include "onboard/planner/test_util/object_prediction_builder.h"

#include "gtest/gtest.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/perception_object_builder.h"

namespace qcraft {
namespace planner {

TEST(ObjectPredictionBuilder, Build) {
  Polygon2d contour(Box2d(Vec2d::Zero(), 0.0, 4.0, 2.0));
  const auto perception = PerceptionObjectBuilder()
                              .set_id("abc")
                              .set_length_width(4.0, 2.0)
                              .set_pos(Vec2d::Zero())
                              .Build();
  ObjectPredictionBuilder builder;
  builder.set_object(perception)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  builder.add_predicted_trajectory()->set_probability(0.5).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/5.0);

  const auto obj_prediction = builder.Build();

  EXPECT_EQ(obj_prediction.id(), "abc");
  EXPECT_EQ(obj_prediction.trajectory_max_prob(), 0.5);
  EXPECT_EQ(obj_prediction.trajectory_min_prob(), 0.3);

  EXPECT_EQ(obj_prediction.trajectories().size(), 2);
  EXPECT_EQ(obj_prediction.trajectories().front().annotation(), "abc");
  EXPECT_EQ(obj_prediction.trajectories().front().probability(), 0.3);
  EXPECT_EQ(obj_prediction.trajectories().back().probability(), 0.5);
}
}  // namespace planner
}  // namespace qcraft
