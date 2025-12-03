#include "onboard/prediction/post_process/trajectory_modifier.h"

#include "gtest/gtest.h"

#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(TrajectoryModifierTest, VehicleCutoffTrajByCurb) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const auto obj_1 = planner::PerceptionObjectBuilder()
                         .set_pos(Vec2d(120.0, 70.0))
                         .set_speed(Vec2d(0.0, 1.0))
                         .set_accel(Vec2d::Zero())
                         .set_yaw(M_PI_2)
                         .set_id("veh_1")
                         .Build();
  planner::ObjectPredictionBuilder prediction_builder_1;
  prediction_builder_1.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/Vec2d(120.0, 70.0),
                         /*end=*/Vec2d(120.0, 80.0),
                         /*init_v=*/1.0, /*last_v=*/1.0);
  auto obj_prediction_1 = prediction_builder_1.Build();
  const int raw_points_num = obj_prediction_1.trajectories()[0].points().size();
  ConflictResolverParams params;
  params.LoadParams();
  const auto& object_config = params.GetConfigByObjectType(obj_1.type());
  CutoffTrajByCurb(psmm, obj_1, object_config,
                   &obj_prediction_1.mutable_trajectories()->front());
  const int cutoff_points_num =
      obj_prediction_1.trajectories()[0].points().size();

  EXPECT_EQ(obj_prediction_1.trajectories().size(), 1);
  EXPECT_GT(raw_points_num, cutoff_points_num);
}

TEST(TrajectoryModifierTest, PedestrianNoCutoffTrajByCurb) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const auto obj_1 = planner::PerceptionObjectBuilder()
                         .set_type(OT_PEDESTRIAN)
                         .set_pos(Vec2d(0.0, -10.0))
                         .set_speed(Vec2d(0.0, 2.0))
                         .set_accel(Vec2d::Zero())
                         .set_yaw(M_PI_2)
                         .set_id("ped_1")
                         .Build();
  planner::ObjectPredictionBuilder prediction_builder_1;
  prediction_builder_1.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/Vec2d(0.0, -10.0), /*end=*/Vec2d(0.0, 10.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);
  auto obj_prediction_1 = prediction_builder_1.Build();
  const int raw_points_num = obj_prediction_1.trajectories()[0].points().size();
  ConflictResolverParams params;
  params.LoadParams();
  const auto& object_config = params.GetConfigByObjectType(obj_1.type());
  CutoffTrajByCurb(psmm, obj_1, object_config,
                   &obj_prediction_1.mutable_trajectories()->front());
  const int cutoff_points_num =
      obj_prediction_1.trajectories()[0].points().size();

  EXPECT_EQ(obj_prediction_1.trajectories().size(), 1);
  EXPECT_EQ(raw_points_num, cutoff_points_num);
}

TEST(TrajectoryModifierTest, PedestrianCutoffTrajByCurb) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const auto obj_1 = planner::PerceptionObjectBuilder()
                         .set_type(OT_PEDESTRIAN)
                         .set_pos(Vec2d(0.0, -10.0))
                         .set_speed(Vec2d(0.0, 1.0))
                         .set_accel(Vec2d::Zero())
                         .set_yaw(M_PI_2)
                         .set_id("ped_1")
                         .Build();
  planner::ObjectPredictionBuilder prediction_builder_1;
  prediction_builder_1.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/Vec2d(0.0, -10.0), /*end=*/Vec2d(0.0, 0.0),
                         /*init_v=*/1.0, /*last_v=*/1.0);
  auto obj_prediction_1 = prediction_builder_1.Build();
  const int raw_points_num = obj_prediction_1.trajectories()[0].points().size();
  ConflictResolverParams params;
  params.LoadParams();
  const auto& object_config = params.GetConfigByObjectType(obj_1.type());
  CutoffTrajByCurb(psmm, obj_1, object_config,
                   &obj_prediction_1.mutable_trajectories()->front());
  const int cutoff_points_num =
      obj_prediction_1.trajectories()[0].points().size();

  EXPECT_EQ(obj_prediction_1.trajectories().size(), 1);
  EXPECT_GT(raw_points_num, cutoff_points_num);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
