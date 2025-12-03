#include "onboard/planner/ml/context_feature_extractor/object_history_sampler.h"

#include "gtest/gtest.h"

#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"

namespace qcraft {
namespace planner {
namespace ml {
namespace {
TEST(ResampleObjectMotionHistoryFromTrackerHistoryWithLookAhead,
     CorrectSyncResult) {
  const Vec2d pos(0.0, 0.0);
  const double perception_ts = 10.0;
  const auto obj = PerceptionObjectBuilder()
                       .set_id("o1")
                       .set_timestamp(perception_ts)
                       .set_accel(0.0)
                       .set_length_width(4.0, 2.0)
                       .set_yaw(0.0)
                       .set_velocity(5.0)
                       .set_trajectory(20)
                       .set_pos(pos)
                       .Build();
  ObjectPredictionBuilder obj_pred_builder;
  obj_pred_builder.set_object(obj)
      .add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/pos, /*end=*/Vec2d(pos.x() + 50.0, pos.y()),
                         /*init_v=*/5.0,
                         /*last_v=*/5.0);
  const auto obj_pred = obj_pred_builder.Build();

  const auto plan_time = perception_ts + 0.4;
  const auto resampled_hist_or =
      ResampleObjectMotionHistoryFromTrackerHistoryWithLookAhead(
          obj, obj_pred.trajectories().front(), perception_ts, plan_time,
          /*time_step=*/0.1, /*max_steps=*/10);
  EXPECT_OK(resampled_hist_or.status());
  EXPECT_EQ(resampled_hist_or->id, "o1");
  const auto& resampled_states = resampled_hist_or->states;
  EXPECT_EQ(resampled_states.back().timestamp, 10.4);
}
}  // namespace
}  // namespace ml
}  // namespace planner
}  // namespace qcraft
