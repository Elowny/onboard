#include "onboard/planner/object/spacetime_object_trajectory.h"

#include "absl/strings/str_cat.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/test_util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {
namespace {

TEST(SpacetimeTrajectoryManagerTest, CreateSpacetimeObject) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d(1.0, 0.0))
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
      .set_straight_line(Vec2d(1.0, 0.0), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(Vec2d::Zero(), Vec2d(0.0, 10.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);

  const PlannerObject object = builder.Build();

  ASSERT_EQ(object.prediction().trajectories().size(), 2);

  constexpr double kEpsilon = 1e-8;
  for (int traj_ix = 0; traj_ix < object.num_trajs(); ++traj_ix) {
    const auto& traj = object.traj(traj_ix);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    EXPECT_EQ(traj.points().size(), states.size());
    for (int i = 0; i < states.size(); ++i) {
      ASSERT_EQ(states[i].box.width(), 2.0);
      ASSERT_EQ(states[i].box.length(), 4.0);
      ASSERT_EQ(states[i].contour.points().size(),
                object.contour().points().size());
      ASSERT_THAT(
          states[i].box.center(),
          Vec2dNear(traj.points()[i].pos() - Vec2d(1.0, 0.0), kEpsilon));
      ASSERT_NEAR(states[i].box.heading(), traj.points()[i].theta(), kEpsilon);
    }
    const auto st_traj = SpacetimeObjectTrajectory(
        object, states, traj_ix, /*required_lateral_gap=*/0.5);
    EXPECT_EQ(st_traj.states().size(), states.size());
    EXPECT_EQ(st_traj.traj_index(), traj_ix);
    EXPECT_EQ(st_traj.traj_id(), absl::StrCat("abc-idx", traj_ix));
    EXPECT_EQ(st_traj.required_lateral_gap(), 0.5);
  }
}

TEST(SpacetimeTrajectoryManagerTest, ModifySpacetimeObjectTrajectory) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d(1.0, 0.0))
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
      .set_straight_line(Vec2d(1.0, 0.0), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  const PlannerObject object = builder.Build();

  ASSERT_EQ(object.prediction().trajectories().size(), 1);

  constexpr double kEpsilon = 1e-8;
  for (int traj_ix = 0; traj_ix < object.num_trajs(); ++traj_ix) {
    const auto& traj = object.traj(traj_ix);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    auto st_traj = SpacetimeObjectTrajectory(object, states, traj_ix,
                                             /*required_lateral_gap=*/0.5);
    // Modify trajectory.
    PredictedTrajectoryBuilder pred_traj_builder;
    pred_traj_builder.set_probability(0.3).set_straight_line(
        Vec2d(2.0, 0.0), Vec2d(8.0, 0.0), /*init_v=*/4.0, /*last_v=*/8.0);
    auto new_traj = pred_traj_builder.Build();
    const auto new_st_traj = st_traj.CreateTrajectoryMutatedInstance(new_traj);
    EXPECT_EQ(new_st_traj.states().size(), new_traj.points().size());
    EXPECT_EQ(new_st_traj.traj_index(), traj_ix);
    EXPECT_EQ(new_st_traj.traj_id(), absl::StrCat("abc-idx", traj_ix));
    EXPECT_EQ(new_st_traj.required_lateral_gap(), 0.5);
    EXPECT_EQ(new_st_traj.trajectory().probability(), 0.3);
    for (int i = 0; i < new_st_traj.states().size(); ++i) {
      ASSERT_EQ(new_st_traj.states()[i].box.width(), 2.0);
      ASSERT_EQ(new_st_traj.states()[i].box.length(), 4.0);
      ASSERT_EQ(new_st_traj.states()[i].contour.points().size(),
                object.contour().points().size());
      ASSERT_THAT(new_st_traj.trajectory().points()[i].pos(),
                  Vec2dNear(new_traj.points()[i].pos(), kEpsilon));
      ASSERT_THAT(
          new_st_traj.states()[i].box.center(),
          Vec2dNear(new_traj.points()[i].pos() - Vec2d(1.0, 0.0), kEpsilon));
      ASSERT_NEAR(new_st_traj.states()[i].box.heading(),
                  new_traj.points()[i].theta(), kEpsilon);
    }
  }
}

}  // namespace
}  // namespace qcraft::planner
