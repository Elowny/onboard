#include "onboard/planner/object/spacetime_trajectory_manager.h"

#include <algorithm>
#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/test_util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft::planner {
namespace {

PlannerObject BuildPlannerObjectHelper(
    const Vec2d& percep_pos, const std::string& object_id, double heading,
    double length, double width, double timestamp, const Vec2d& init_pos,
    const Vec2d& end_pos, double init_v, double end_v) {
  PerceptionObjectBuilder percep_builder;
  auto perception_obj = percep_builder.set_id(object_id)
                            .set_type(OT_VEHICLE)
                            .set_timestamp(timestamp)
                            .set_velocity(init_v)
                            .set_yaw(heading)
                            .set_length_width(length, width)
                            .set_box_center(percep_pos)
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(init_pos, end_pos, init_v, end_v);
  return builder.Build();
}

TEST(SpacetimeTrajectoryManagerTest, CreateSpacetimeObject) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("abc")
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
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(Vec2d::Zero(), Vec2d(0.0, 10.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);

  objects.push_back(builder.Build());

  perception_obj.set_id("def");
  objects.push_back(PlannerObjectBuilder()
                        .set_object(perception_obj)
                        .set_stationary(true)
                        .Build());

  SpacetimeTrajectoryManager mgr(objects);

  // TODO(lidong): How to draw the planner object and spacetime objects in unit
  // test?

  const auto& st_trajs = mgr.trajectories();
  ASSERT_EQ(st_trajs.size(), 3);

  EXPECT_EQ(mgr.FindTrajectoriesByObjectId("abc").size(), 2);
  EXPECT_EQ(mgr.FindTrajectoriesByObjectId("def").size(), 1);
  EXPECT_EQ(mgr.FindTrajectoriesByObjectId("hig").size(), 0);
  EXPECT_EQ(mgr.FindTrajectoryById("abc-idx1")->traj_id(), "abc-idx1");
  EXPECT_EQ(mgr.FindTrajectoryById("abc-def"), nullptr);
  EXPECT_EQ(mgr.FindObjectByObjectId("abc")->id(), "abc");
  EXPECT_EQ(mgr.FindObjectByObjectId("xyz"), nullptr);

  const auto& first_states = st_trajs[0].states();
  for (int i = 0; i < first_states.size(); ++i) {
    const auto& state = first_states[i];
    ASSERT_NEAR(state.traj_point->theta(), 0.0, 1e-8);
    ASSERT_NEAR(state.box.heading(), 0.0, 1e-8);
    ASSERT_NEAR(state.box.length(), 4.0, 1e-8);
    ASSERT_NEAR(state.box.width(), 2.0, 1e-8);
    ASSERT_TRUE(state.contour.IsPointIn(state.box.center()));
  }

  const auto& second_states = st_trajs[1].states();
  for (int i = 0; i < second_states.size(); ++i) {
    const auto& state = second_states[i];
    ASSERT_NEAR(state.traj_point->theta(), M_PI_2, 1e-8);
    ASSERT_NEAR(state.box.heading(), M_PI_2, 1e-8);
    ASSERT_NEAR(state.box.length(), 4.0, 1e-8);
    ASSERT_NEAR(state.box.width(), 2.0, 1e-8);
    ASSERT_TRUE(state.contour.IsPointIn(state.box.center()));
  }

  // This is a stationary object.
  const auto& third_states = st_trajs[2].states();
  ASSERT_EQ(third_states.size(), 100);
  const auto& state = third_states[0];
  ASSERT_EQ(state.box.heading(), 0.0);
  ASSERT_EQ(state.box.length(), 4.0);
  ASSERT_EQ(state.box.width(), 2.0);
  ASSERT_THAT(state.box.center(), Vec2dEqXY(0.0, 0.0));
  ASSERT_TRUE(state.contour.IsPointIn(state.box.center()));
}

TEST(SpacetimeTrajectoryManagerTest, FilterMoreSpacetimeObjectTrajectories) {
  std::vector<PlannerObject> planner_objects;
  planner_objects.push_back(
      BuildPlannerObjectHelper(Vec2d(0.0, 0.0), "obj_1", /*heading=*/0.0,
                               /*length=*/4.0, /*width=*/2.0, /*timestamp=*/0.1,
                               Vec2d(0.0, 0.0), Vec2d(10.0, 0.0),
                               /*init_v=*/2.0, /*end_v=*/2.0));
  planner_objects.push_back(
      BuildPlannerObjectHelper(Vec2d(5.0, 0.0), "obj_2", /*heading=*/0.0,
                               /*length=*/4.0, /*width=*/2.0, /*timestamp=*/0.1,
                               Vec2d(5.0, 0.0), Vec2d(5.0, 5.0),
                               /*init_v=*/10.0, /*end_v=*/2.0));
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  EXPECT_NE(st_traj_mgr.FindTrajectoryById("obj_1-idx0"), nullptr);
  EXPECT_NE(st_traj_mgr.FindTrajectoryById("obj_2-idx0"), nullptr);
  EXPECT_EQ(st_traj_mgr.trajectories().size(), 2);
  EXPECT_EQ(st_traj_mgr.moving_object_trajs().size(), 2);
  EXPECT_EQ(st_traj_mgr.stationary_object_trajs().size(), 0);
  EXPECT_EQ(st_traj_mgr.ignored_trajectories().size(), 0);

  std::map<std::string, FilterReason::Type> id_reason_map;
  id_reason_map.insert(std::make_pair(
      "obj_1-idx0", FilterReason::OBJECT_NOT_ON_OR_NEAR_PATH_BOUNDARY));

  st_traj_mgr.FilterTrajectoriesWithReason(id_reason_map);
  EXPECT_EQ(st_traj_mgr.ignored_trajectories().size(), 1);
  EXPECT_EQ(st_traj_mgr.FindTrajectoryById("obj_1-idx0"), nullptr);
  EXPECT_NE(st_traj_mgr.FindTrajectoryById("obj_2-idx0"), nullptr);
  EXPECT_EQ(st_traj_mgr.trajectories().size(), 1);
  const auto& ignored_traj = st_traj_mgr.ignored_trajectories().front();
  EXPECT_EQ(ignored_traj.object_id, "obj_1");
  EXPECT_EQ(ignored_traj.reason,
            FilterReason::OBJECT_NOT_ON_OR_NEAR_PATH_BOUNDARY);
}
}  // namespace
}  // namespace qcraft::planner
