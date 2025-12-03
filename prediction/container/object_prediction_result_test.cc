#include "onboard/prediction/container/object_prediction_result.h"

#include "gtest/gtest.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(ObjectPredictionResultTest, CheckObjectPredictionResult) {
  Polygon2d contour(Box2d(Vec2d::Zero(), 0.0, 4.0, 2.0));
  const auto perception = planner::PerceptionObjectBuilder()
                              .set_id("abc")
                              .set_length_width(4.0, 2.0)
                              .set_pos(Vec2d::Zero())
                              .Build();
  planner::ObjectPredictionBuilder builder;
  builder.set_object(perception)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  builder.add_predicted_trajectory()->set_probability(0.5).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/5.0);
  const auto obj_prediction = builder.Build();
  ObjectPredictionProto object_prediction_proto;
  obj_prediction.ToProto(&object_prediction_proto);
  ObjectPredictionResult object_prediction_result;
  object_prediction_result.id = obj_prediction.id();
  object_prediction_result.priority = ObjectPredictionPriority::OPP_P1;
  object_prediction_result.perception_object = perception;
  object_prediction_result.trajectories = obj_prediction.trajectories();
  object_prediction_result.startup_trajs = obj_prediction.trajectories();
  EXPECT_EQ(object_prediction_result.id, "abc");
  EXPECT_EQ(object_prediction_result.trajectories.size(), 2);
  EXPECT_EQ(object_prediction_result.mutable_trajectories()->size(), 2);
  EXPECT_EQ(object_prediction_result.trajectories.front().probability(), 0.3);
  EXPECT_EQ(object_prediction_result.trajectories.back().probability(), 0.5);
  EXPECT_EQ(object_prediction_result.startup_trajs.size(), 2);
  ObjectPredictionProto compressed_object_prediction_proto;
  object_prediction_result.ToCompressedProto(
      &compressed_object_prediction_proto);
  EXPECT_EQ(compressed_object_prediction_proto.trajectories().size(), 2);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
