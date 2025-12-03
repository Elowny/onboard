#include "onboard/planner/initializer/collision_checker.h"

#include "absl/algorithm/container.h"
#include "absl/strings/string_view.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/brute_force_collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {

// operator== Must in the same namespace with the class definition to make
// EXPECT_EQ work.
bool operator==(const CollisionInfo::ObjectCollision& a,
                const CollisionInfo::ObjectCollision& b) {
  return a.traj->traj_id() == b.traj->traj_id() && a.time == b.time;
}
bool operator!=(const CollisionInfo::ObjectCollision& a,
                const CollisionInfo::ObjectCollision& b) {
  return !(a == b);
}

bool operator==(const CollisionInfo& lhs, const CollisionInfo& rhs) {
  if (lhs.collision_objects.size() != rhs.collision_objects.size()) {
    return false;
  }
  const auto sorter = [](const CollisionInfo::ObjectCollision& a,
                         const CollisionInfo::ObjectCollision& b) {
    if (a.traj->traj_id() < b.traj->traj_id()) return true;
    if (a.traj->traj_id() > b.traj->traj_id()) return false;
    return a.time < b.time;
  };
  auto lhs_collisions = lhs.collision_objects;
  auto rhs_collisions = rhs.collision_objects;
  absl::c_sort(lhs_collisions, sorter);
  absl::c_sort(rhs_collisions, sorter);
  for (int i = 0; i < lhs_collisions.size(); ++i) {
    if (lhs_collisions[i] != rhs_collisions[i]) {
      return false;
    }
  }
  return true;
}

std::vector<GeometryState> MotionStatesToGeomStates(
    absl::Span<const MotionState> states) {
  std::vector<GeometryState> geom_states;
  double accum_s = 0.0;
  for (const auto& s : states) {
    if (!geom_states.empty()) {
      accum_s += geom_states.back().xy.DistanceTo(s.xy);
    }
    geom_states.push_back(GeometryState{
        .xy = s.xy, .h = s.h, .accumulated_s = accum_s, .l = 0.0});
  }
  return geom_states;
}

namespace {
constexpr double kBuffer = 0.25;  // m.

using testing::UnorderedElementsAre;

void CheckResult(const BoxGroupCollisionChecker& checker,
                 const BruteForceCollisionChecker& gt_checker, double init_t,
                 const MotionForm& motion, const IgnoreTrajMap& ignored_trajs,
                 qcraft::SourceLocation loc) {
  ASSERT_EQ(checker.sample_step(), gt_checker.sample_step());
  CollisionInfo info;
  const auto& all_states = motion.SampleStates();
  const auto& states = all_states.const_interval_states;
  checker.CheckCollisionWithTrajectories(init_t, states, ignored_trajs, &info);
  for (const auto& obj : info.collision_objects) {
    EXPECT_FALSE(ignored_trajs.contains(obj.traj->traj_id()));
  }
  CollisionInfo bf_info;
  gt_checker.CheckCollisionWithTrajectories(init_t, states, ignored_trajs,
                                            &bf_info);
  for (const auto& obj : bf_info.collision_objects) {
    EXPECT_FALSE(ignored_trajs.contains(obj.traj->traj_id()));
  }
  EXPECT_EQ(info, bf_info) << loc.ToString();

  const auto geom_states = MotionStatesToGeomStates(states);
  info.Clear();
  checker.CheckCollisionWithStationaryObjects(geom_states, &info);
  bf_info.Clear();
  gt_checker.CheckCollisionWithStationaryObjects(geom_states, &bf_info);
  EXPECT_EQ(info, bf_info) << loc.ToString();
}
TEST(CollisionChecker, StraightLineStatic) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  const Vec2d obj_pos(12.0, 0.0);
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj =
      perception_builder.set_id("abc")
          .set_type(OT_VEHICLE)
          .set_timestamp(1.0)
          .set_velocity(0.0)
          .set_yaw(0.0)
          .set_length_width(4.0, 2.0)
          .set_contour(Polygon2d(Box2d(obj_pos, 0.0, 4.0, 2.0)))
          .set_box_center(obj_pos)
          .Build();
  PlannerObjectBuilder builder;

  builder.set_type(OT_VEHICLE).set_object(perception_obj).set_stationary(true);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_stationary_traj(obj_pos, 0.0);
  const PlannerObject object = builder.Build();

  SpacetimeTrajectoryManager mgr(absl::MakeSpan(&object, 1));
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::STATIONARY);
  }

  const auto vehicle_geom = DefaultVehicleGeometry();

  BoxGroupCollisionChecker checker(&st_planner_object_traj, &vehicle_geom,
                                   MotionForm::kConstTimeIntervalSampleStep,
                                   /*stationary_obj_buffer=*/kBuffer,
                                   /*moving_obj_buffer=*/2.0 * kBuffer);
  BruteForceCollisionChecker brute_force(
      mgr.trajectories(), &vehicle_geom,
      MotionForm::kConstTimeIntervalSampleStep,
      /*stationary_buffer=*/kBuffer, /*moving_buffer=*/2.0 * kBuffer);

  const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(5.0, 0.0));
  const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                /*init_a=*/1.0, &line);
  CheckResult(checker, brute_force, /*init_t=*/0.0, motion,
              /*ignored_trajs=*/{}, QCRAFT_LOC);

  checker.UpdateStationaryObjectBuffer(10.0 * kBuffer);
  brute_force.UpdateStationaryObjectBuffer(10.0 * kBuffer);
  CheckResult(checker, brute_force, /*init_t=*/0.0, motion,
              /*ignored_trajs=*/{}, QCRAFT_LOC);
}

TEST(CollisionChecker, ConfigurationRight) {
  const Box2d box1({0, 0}, M_PI_2, 4, 2);
  const Box2d box2({5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box1, box2), CollisionConfiguration::RIGHT);

  const Box2d box3({0, 0}, -M_PI_2, 4, 2);
  const Box2d box4({-5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box3, box4), CollisionConfiguration::RIGHT);
}

TEST(CollisionChecker, ConfigurationLeft) {
  const Box2d box1({0, 0}, M_PI_2, 4, 2);
  const Box2d box2({-5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box1, box2), CollisionConfiguration::LEFT);

  const Box2d box3({0, 0}, -M_PI_2, 4, 2);
  const Box2d box4({5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box3, box4), CollisionConfiguration::LEFT);
}

TEST(CollisionChecker, ConfigurationFront) {
  const Box2d box1({0, 0}, 0, 4, 2);
  const Box2d box2({5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box1, box2), CollisionConfiguration::FRONT);

  const Box2d box3({0, 0}, M_PI, 4, 2);
  const Box2d box4({-5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box3, box4), CollisionConfiguration::FRONT);

  const Box2d box5({1, 1}, M_PI / 6.0, 4, 2);
  const Box2d box6({10, 20}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box5, box6), CollisionConfiguration::FRONT);
}

TEST(CollisionChecker, ConfigurationBack) {
  const Box2d box1({0, 0}, 0, 4, 2);
  const Box2d box2({-5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box1, box2), CollisionConfiguration::BACK);

  const Box2d box3({0, 0}, M_PI, 4, 2);
  const Box2d box4({5, 2}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box3, box4), CollisionConfiguration::BACK);

  const Box2d box5({1, 1}, M_PI / 6.0, 4, 2);
  const Box2d box6({-10, -20}, 0, 3, 1);
  EXPECT_EQ(DetermineConfiguration(box5, box6), CollisionConfiguration::BACK);
}

TEST(CollisionChecker, StraightLine) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

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
                         /*init_v=*/2.0, /*last_v=*/10.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(Vec2d::Zero(), Vec2d(0.0, 10.0),
                         /*init_v=*/2.0, /*last_v=*/2.0);

  const PlannerObject object = builder.Build();

  SpacetimeTrajectoryManager mgr(absl::MakeSpan(&object, 1));
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  for (const auto& trajectory : mgr.trajectories()) {
    st_planner_object_traj.AddSpacetimePlannerObjectTrajectory(
        trajectory, SpacetimePlannerObjectTrajectoryReason::FRONT);
  }

  const auto vehicle_geom = DefaultVehicleGeometry();

  BoxGroupCollisionChecker checker(&st_planner_object_traj, &vehicle_geom,
                                   MotionForm::kConstTimeIntervalSampleStep,
                                   /*stationary_obj_buffer=*/kBuffer,
                                   /*moving_obj_buffer=*/2.0 * kBuffer);
  BruteForceCollisionChecker brute_force(
      mgr.trajectories(), &vehicle_geom,
      MotionForm::kConstTimeIntervalSampleStep,
      /*stationary_buffer=*/kBuffer, /*moving_buffer=*/2.0 * kBuffer);
  const IgnoreTrajMap empty_ignored_trajs = {};

  {
    // Collision at the initial positions.
    const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
    const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                  /*init_a=*/1.0, &line);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion,
                empty_ignored_trajs, QCRAFT_LOC);
  }

  {
    // No collision when no time overlap.
    const double last_time =
        mgr.trajectories()[1].states().back().traj_point->t();
    const StraightLineGeometry line(Vec2d(0.0, 0.0), Vec2d(4.0, 0.0));
    const ConstAccelMotion motion(/*traj_horizon=*/9.9, /*init_v=*/1.0,
                                  /*init_a=*/1.0, &line);
    CheckResult(checker, brute_force, /*init_t=*/last_time + 1.0, motion,
                empty_ignored_trajs, QCRAFT_LOC);
  }

  {
    // Collision with one prediction trajectory.
    GeometryState state;
    state.xy = Vec2d(5.0, 0.0);
    state.h = 0.0;
    const StationaryGeometry geom(state);
    const StationaryMotion motion(/*duration=*/10.0, &geom);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion,
                empty_ignored_trajs, QCRAFT_LOC);
  }

  {
    IgnoreTrajMap ignored_trajs;
    ignored_trajs["abc-idx0"] = CollisionConfigurationInfo{
        .time_idx = 2,
        .collision_config = CollisionConfiguration::FRONT,
    };

    // Collision with one prediction trajectory but ignored.
    GeometryState state;
    state.xy = Vec2d(5.0, 0.0);
    state.h = 0.0;
    const StationaryGeometry geom(state);
    const StationaryMotion motion(/*duration=*/10.0, &geom);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion, ignored_trajs,
                QCRAFT_LOC);
  }

  {
    // No collision as SDC disappears before object arrives.
    GeometryState state;
    state.xy = Vec2d(5.0, 0.0);
    state.h = 0.0;
    const StationaryGeometry geom(state);
    const StationaryMotion motion(/*duration=*/0.5, &geom);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion,
                empty_ignored_trajs, QCRAFT_LOC);
  }

  {
    // Collision due to large buffer.
    checker.UpdateMovingObjectBuffer(20.0 * kBuffer);
    brute_force.UpdateMovingObjectBuffer(20.0 * kBuffer);
    GeometryState state;
    state.xy = Vec2d(5.0, 0.0);
    state.h = 0.0;
    const StationaryGeometry geom(state);
    const StationaryMotion motion(/*duration=*/0.5, &geom);
    CheckResult(checker, brute_force, /*init_t=*/0.0, motion,
                empty_ignored_trajs, QCRAFT_LOC);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
