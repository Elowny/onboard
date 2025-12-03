#include "onboard/prediction/object_prediction.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace prediction {
namespace {
ObjectPrediction BuildObjectPrediction(const ObjectProto& perception) {
  planner::ObjectPredictionBuilder builder;
  builder.set_object(perception)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  builder.add_predicted_trajectory()->set_probability(0.5).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(12.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/7.0);
  builder.add_predicted_trajectory()->set_probability(0.2).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(8.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/3.0);

  return builder.Build();
}

TEST(ObjectPredictionTest, TestConstructByObjectProto) {
  const auto perception = planner::PerceptionObjectBuilder()
                              .set_id("1")
                              .set_length_width(4.0, 2.0)
                              .set_pos(Vec2d::Zero())
                              .Build();
  ObjectPrediction object_prediction(perception);
  EXPECT_TRUE(object_prediction.trajectories().empty());
  EXPECT_THAT(object_prediction.perception_object(), ProtoEq(perception));
}

TEST(ObjectPredictionTest, TestPublicAPI) {
  const auto perception = planner::PerceptionObjectBuilder()
                              .set_id("abc")
                              .set_length_width(4.0, 2.0)
                              .set_pos(Vec2d::Zero())
                              .Build();
  auto object_prediction = BuildObjectPrediction(perception);
  object_prediction.PrintDebugInfo();
  const auto& poly_contour = object_prediction.contour();
  const auto point_contour = object_prediction.CreateContourForPoint(0, 10);
  EXPECT_FALSE(poly_contour.points().empty());
  EXPECT_EQ(point_contour.points().size(), poly_contour.points().size());

  const auto& stop_time_proto = object_prediction.stop_time();
  EXPECT_FALSE(stop_time_proto.has_time_duration_since_stop());
  const auto& behavior = object_prediction.long_term_behavior();
  EXPECT_EQ(behavior.obs_duration, 0.0);
  EXPECT_EQ(object_prediction.road_status(), ObjectRoadStatus::ORS_NONE);
  EXPECT_EQ(object_prediction.intersection_status(),
            ObjectIntersectionStatus::OIS_NONE);
  EXPECT_EQ(object_prediction.trajectories().size(), 3);
  EXPECT_THAT(object_prediction.perception_object(), ProtoEq(perception));
  EXPECT_EQ(object_prediction.timestamp(), perception.timestamp());
  EXPECT_EQ(object_prediction.id(), perception.id());
  EXPECT_NEAR(object_prediction.trajectory_prob_sum(), 1.0, 1e-4);
  EXPECT_NEAR(object_prediction.trajectory_max_prob(), 0.5, 1e-4);
  EXPECT_NEAR(object_prediction.trajectory_min_prob(), 0.2, 1e-4);

  ObjectPredictionProto obj_pred_proto, obj_pred_compressed_proto;
  object_prediction.ToProto(&obj_pred_proto);
  object_prediction.ToCompressedProto(&obj_pred_compressed_proto);
  EXPECT_THAT(obj_pred_proto.perception_object(),
              ProtoEq(obj_pred_compressed_proto.perception_object()));
  EXPECT_EQ(obj_pred_proto.trajectories().size(),
            obj_pred_compressed_proto.trajectories().size());
  EXPECT_GT(obj_pred_proto.trajectories(0).points().size(),
            obj_pred_compressed_proto.trajectories(0).points().size());
  object_prediction.FromProto(obj_pred_proto);
}

TEST(ObjectPredictionTest, ShiftTimeWithObjectProto) {
  const std::string id = "object";
  const Vec2d pos = Vec2d(0.0, 0.0);

  // Build perception object.
  planner::PerceptionObjectBuilder percep_builder;
  const auto perception = percep_builder.set_id(id)
                              .set_type(OT_VEHICLE)
                              .set_pos(pos)
                              .set_timestamp(0.0)
                              .set_box_center(pos)
                              .set_length_width(5.2, 1.7)
                              .set_yaw(0.0)
                              .set_velocity(10.0)
                              .Build();
  // Build latest perception object.
  const Vec2d new_pos = Vec2d(48.0, 0.0);
  planner::PerceptionObjectBuilder percep_builder_2;
  const auto perception_new = percep_builder_2.set_id(id)
                                  .set_type(OT_VEHICLE)
                                  .set_pos(new_pos)
                                  .set_timestamp(5.0)
                                  .set_box_center(new_pos)
                                  .set_length_width(5.2, 1.7)
                                  .set_yaw(0.0)
                                  .set_velocity(10.0)
                                  .Build();

  // Build ObjectPrediction.
  planner::ObjectPredictionBuilder builder;
  builder.set_object(perception)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(80.0, 0.0),
                         /*init_v=*/10.0, /*last_v=*/10.0);
  builder.add_predicted_trajectory()->set_probability(0.5).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(120.0, 0.0),
      /*init_v=*/10.0, /*last_v=*/15.0);
  builder.add_predicted_trajectory()->set_probability(0.2).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(30.0, 0.0),
      /*init_v=*/10.0, /*last_v=*/6.0);
  auto object_prediction = builder.Build();
  object_prediction.PrintDebugInfo();
  // Should be true, one of the trajectories has been deleted. Two remains with
  // decreased number of points.
  const double shift_time1 =
      perception_new.timestamp() - perception.timestamp();
  EXPECT_TRUE(object_prediction.ShiftTimeWithObjectProto(
      /*prediction_shift_time=*/shift_time1,
      /*object_shift_time=*/shift_time1, perception_new));
  EXPECT_EQ(object_prediction.trajectories().size(), 2);
  object_prediction.PrintDebugInfo();

  // Build next latest perception object.
  const Vec2d new_pos_2 = Vec2d(100.0, 0.0);
  planner::PerceptionObjectBuilder percep_builder_3;
  const auto perception_new_2 = percep_builder_3.set_id(id)
                                    .set_type(OT_VEHICLE)
                                    .set_pos(new_pos_2)
                                    .set_timestamp(20.0)
                                    .set_box_center(new_pos_2)
                                    .set_length_width(5.2, 1.7)
                                    .set_yaw(0.0)
                                    .set_velocity(10.0)
                                    .Build();
  // Further shift. None of the trajs should left.
  const double shift_time2 =
      perception_new_2.timestamp() - perception_new.timestamp();
  EXPECT_FALSE(object_prediction.ShiftTimeWithObjectProto(
      /*prediction_shift_time=*/shift_time2,
      /*object_shift_time=*/shift_time2, perception_new_2));
  EXPECT_TRUE(object_prediction.trajectories().empty());
  object_prediction.PrintDebugInfo();
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
