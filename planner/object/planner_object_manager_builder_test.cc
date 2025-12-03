#include "onboard/planner/object/planner_object_manager_builder.h"

#include <string>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEps = 1e-3;

TEST(PlannerObjectManagerBuilder, MergePerceptionAndPrediction) {
  ObjectsProto perception_objects;
  const auto object1 = PerceptionObjectBuilder()
                           .set_type(OT_FOD)
                           .set_id("1")
                           .set_timestamp(1.0)
                           .Build();
  *perception_objects.add_objects() = object1;
  const auto object2 = PerceptionObjectBuilder()
                           .set_id("2")
                           .set_type(OT_VEHICLE)
                           .set_timestamp(1.1)
                           .Build();
  *perception_objects.add_objects() = object2;

  ObjectPredictionBuilder pred_builder;
  auto object3 = object2;
  const double pred_ts = 0.9;
  object3.set_timestamp(pred_ts);
  pred_builder.set_object(object3)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  const auto object_pred = pred_builder.Build();
  ObjectsPredictionProto objects_pred;
  object_pred.ToProto(objects_pred.add_objects());

  const double align_time = 1.2;
  auto planner_objects =
      BuildPlannerObjects(&perception_objects, &objects_pred, align_time);
  for (const auto& planner_object : planner_objects) {
    if (planner_object.id() == object_pred.id()) {
      EXPECT_EQ(planner_object.num_trajs(), 1);
      EXPECT_EQ(object_pred.trajectories().size(), 1);
      const double time_shift = align_time - pred_ts;
      const double original_prediction_time =
          planner_object.traj(0).points().back().t();
      const double shifted_prediction_time =
          object_pred.trajectories()[0].points().back().t();
      EXPECT_NEAR(original_prediction_time,
                  shifted_prediction_time - time_shift, kEps);
    }
  }
  const auto mgr = PlannerObjectManagerBuilder()
                       .set_planner_objects(std::move(planner_objects))
                       .Build();
  EXPECT_OK(mgr) << mgr.status().message();
  EXPECT_EQ(mgr->size(), 2);
  EXPECT_EQ(mgr->stationary_objects().size(), 1);
  for (const auto& planner_object : mgr->planner_objects()) {
    ASSERT_EQ(planner_object.object_proto().timestamp(), 1.2);
    ASSERT_EQ(planner_object.prediction().perception_object().timestamp(), 1.2);
  }
}

TEST(PlannerObjectManagerBuilder, BuildPlannerObjectsFromObjectPrediction) {
  ObjectsProto perception_objects;
  const auto object1 = PerceptionObjectBuilder()
                           .set_type(OT_FOD)
                           .set_id("1")
                           .set_timestamp(1.0)
                           .Build();
  *perception_objects.add_objects() = object1;
  const auto object2 = PerceptionObjectBuilder()
                           .set_id("2")
                           .set_type(OT_VEHICLE)
                           .set_timestamp(1.1)
                           .Build();
  *perception_objects.add_objects() = object2;

  ObjectPredictionBuilder pred_builder;
  auto object3 = object2;
  const double pred_ts = 0.9;
  object3.set_timestamp(pred_ts);
  pred_builder.set_object(object3)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(10.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/5.0);
  auto object_pred = pred_builder.Build();
  object_pred.PrintDebugInfo();

  std::vector<prediction::ObjectPrediction> prediction_objects = {object_pred};
  const double align_time = 1.2;
  auto planner_objects = BuildPlannerObjectsFromObjectPrediction(
      &perception_objects, &prediction_objects, align_time);
  for (const auto& planner_object : planner_objects) {
    if (planner_object.id() == object_pred.id()) {
      EXPECT_EQ(planner_object.num_trajs(), 1);
      EXPECT_EQ(object_pred.trajectories().size(), 1);
      const double time_shift = align_time - pred_ts;
      const double original_prediction_time =
          planner_object.traj(0).points().back().t();
      const double shifted_prediction_time =
          object_pred.trajectories()[0].points().back().t();
      EXPECT_NEAR(original_prediction_time,
                  shifted_prediction_time - time_shift, kEps);
    }
  }

  const auto mgr = PlannerObjectManagerBuilder()
                       .set_planner_objects(std::move(planner_objects))
                       .Build();
  EXPECT_OK(mgr) << mgr.status().message();
  EXPECT_EQ(mgr->size(), 2);
  EXPECT_EQ(mgr->stationary_objects().size(), 1);
  for (const auto& planner_object : mgr->planner_objects()) {
    ASSERT_EQ(planner_object.object_proto().timestamp(), 1.2);
    ASSERT_EQ(planner_object.prediction().perception_object().timestamp(), 1.2);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
