#include "onboard/planner/assist/tja_internal.h"

#include <algorithm>
#include <cmath>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/time_util.h"

// #include "offboard/mapping/utils/map_plot_util.h"
namespace qcraft::planner {

namespace {
PlannerObjectManager BuildPhantomVehicle(const Vec2d& obj_pos,
                                         double heading = 0.0) {
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(heading)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(obj_pos, heading);

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

PlannerObjectManager BuildMovingVehicle(const Vec2d& obj_pos,
                                        const Vec2d& speed,
                                        double heading = 0.0) {
  ObjectVector<PlannerObject> objects;
  const double perception_ts = 10.0;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("MovingPhantom")
                            .set_timestamp(perception_ts)
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_speed(speed)
                            .set_length_width(4.5, 2.2)
                            .set_yaw(heading)
                            .set_trajectory(80)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(1.0)
      .set_straight_line(/*start=*/obj_pos,
                         /*end=*/Vec2d(obj_pos.x() + 50.0, obj_pos.y()),
                         /*init_v=*/speed.norm(),
                         /*last_v=*/speed.norm());

  objects.push_back(builder.Build());
  return PlannerObjectManager(objects);
}

// void SendPathToCanvas(absl::Span<const Vec2d> path, const std::string& topic,
//                       const vis::Color& color) {
//   auto& canvas = qcraft::vantage_client_man::GetCanvas(topic);
//   for (int i = 0; i + 1 < path.size(); ++i) {
//     canvas.DrawLine(Vec3d(path[i].x(), path[i].y(), 0.0),
//                     Vec3d(path[i + 1].x(), path[i + 1].y(), 0.0),
//                     color, 1.0);
//   }
// }

std::vector<Vec2d> SamplePath(const Vec2d& start, const Vec2d& end,
                              double interval) {
  std::vector<Vec2d> path;
  const double dist = start.DistanceTo(end);
  const int num = std::ceil(dist / interval);
  path.reserve(num);
  const Vec2d step = (end - start) / dist * interval;
  for (int i = 0; i < num; ++i) {
    path.push_back(start + i * step);
  }
  return path;
}

TEST(TjaUtil, FillPlannerCenterLine) {
  google::protobuf::RepeatedPtrField<Vec2dProto> input;
  for (int i = 0; i < 3; ++i) {
    Vec2dProto* p = input.Add();
    p->set_x(i);
    p->set_y(i + 1);
  }

  std::vector<Vec2d> output;
  FillPlannerCenterLine(input, &output);

  EXPECT_EQ(output.size(), input.size());
  for (int i = 0; i < input.size(); ++i) {
    EXPECT_EQ(output[i].x(), input.Get(i).x());
    EXPECT_EQ(output[i].y(), input.Get(i).y());
  }
}

TEST(TjaUtil, ShouldUseTjaOnlineSemanticMap) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  absl::Time plan_time = absl::Now();
  const auto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time),
                                   Vec2d(216.0, 63.0), 0.0, Vec2d(6.0, 0.0));

  {
    // When input psmm is ready, refuse to use tja.
    TjaState tja_state;
    const bool should_use_tja =
        ShouldUseTjaOnlineSemanticMap(sdc_pose, psmm, &tja_state);
    EXPECT_TRUE(!should_use_tja);
    EXPECT_EQ(tja_state.exit_counter, 0);
  }

  {
    // When input psmm is ready. delay to exit tja.
    TjaState tja_state;
    tja_state.planner_use_tja_map = true;
    tja_state.exit_counter = 3;
    const bool should_use_tja =
        ShouldUseTjaOnlineSemanticMap(sdc_pose, psmm, &tja_state);
    EXPECT_TRUE(should_use_tja);
    EXPECT_EQ(tja_state.exit_counter, 2);
  }

  {
    // When input psmm is ready. Exit tja.
    TjaState tja_state;
    tja_state.planner_use_tja_map = true;
    tja_state.exit_counter = 1;
    const bool should_use_tja =
        ShouldUseTjaOnlineSemanticMap(sdc_pose, psmm, &tja_state);
    EXPECT_TRUE(!should_use_tja);
    EXPECT_EQ(tja_state.exit_counter, 0);
  }
  {
    // When input psmm is ready. Exit tja.
    TjaState tja_state;
    const auto sdc_pose_no_map =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(153.0, -38.0), 0.0,
                   Vec2d(6.0, 0.0));
    const bool should_use_tja =
        ShouldUseTjaOnlineSemanticMap(sdc_pose_no_map, psmm, &tja_state);
    EXPECT_TRUE(should_use_tja);
    EXPECT_GT(tja_state.exit_counter, 0);
  }

  {
    // When ego pos in middle of two lanes(lane lateral distance > 4.5).
    TjaState tja_state;
    const auto sdc_pose_middle =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(1036.3, -468.5), -1.57,
                   Vec2d(6.0, 0.0));
    const bool should_use_tja =
        ShouldUseTjaOnlineSemanticMap(sdc_pose_middle, psmm, &tja_state);
    EXPECT_FALSE(should_use_tja);
    EXPECT_EQ(tja_state.exit_counter, 0);
  }
}

TEST(TjaUtil, ActivateOnlineSemanticMapStaticLeader) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  mapping::OnlineSemanticMapProto online_semantic_map;
  const auto vehicle_geom = DefaultVehicleGeometry();

  absl::Time plan_time = absl::Now();
  const auto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time),
                                   Vec2d(168.0, -50.0), 0.0, Vec2d(6.0, 0.0));
  {
    // Build one object.
    const auto object_mgr = BuildPhantomVehicle(Vec2d(188.0, -50.0));
    const auto st_traj_mgr =
        SpacetimeTrajectoryManager(object_mgr.planner_objects());
    TjaState tja_state;
    tja_state.center_line =
        SamplePath(Vec2d(168.0, -50.0), Vec2d(188.0, -50.0), 2.0);
    const auto new_online_map =
        ActivateOnlineSemanticMap(sdc_pose, st_traj_mgr, vehicle_geom, psmm,
                                  online_semantic_map, &tja_state);
    EXPECT_OK(new_online_map);
  }

  {
    // Build one out of angle object.
    const auto object_mgr =
        BuildPhantomVehicle(Vec2d(188.0, -50.0), M_PI * 30.0 / 180.0);
    const auto st_traj_mgr =
        SpacetimeTrajectoryManager(object_mgr.planner_objects());
    TjaState tja_state;
    const auto new_online_map =
        ActivateOnlineSemanticMap(sdc_pose, st_traj_mgr, vehicle_geom, psmm,
                                  online_semantic_map, &tja_state);
    EXPECT_NOT_OK(new_online_map);
  }
}

TEST(TjaUtil, ActivateOnlineSemanticMapMovingLeader) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  mapping::OnlineSemanticMapProto online_semantic_map;
  const auto vehicle_geom = DefaultVehicleGeometry();

  absl::Time plan_time = absl::Now();
  const auto sdc_pose = CreatePose(ToUnixDoubleSeconds(plan_time),
                                   Vec2d(168.0, -50.0), 0.0, Vec2d(6.0, 0.0));

  // Build one object.
  const auto object_mgr =
      BuildMovingVehicle(Vec2d(188.0, -50.0), Vec2d(5.0, 0.0));
  const auto st_traj_mgr =
      SpacetimeTrajectoryManager(object_mgr.planner_objects());
  TjaState tja_state;
  tja_state.center_line =
      SamplePath(Vec2d(168.0, -50.0), Vec2d(188.0, -50.0), 2.0);

  const auto new_online_map =
      ActivateOnlineSemanticMap(sdc_pose, st_traj_mgr, vehicle_geom, psmm,
                                online_semantic_map, &tja_state);
  EXPECT_OK(new_online_map);
  EXPECT_GT(tja_state.center_line.size(), 10);
  EXPECT_EQ(tja_state.target_obs_ids.size(), 1);
  EXPECT_EQ(tja_state.update_id, 1);

  EXPECT_EQ((*new_online_map)->lanes_size(), 1);
}
}  // namespace

}  // namespace qcraft::planner
