#include "onboard/planner/decision/traffic_gap_v2/traffic_gap_finder.h"

#include <algorithm>
#include <optional>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {
namespace {

PlannerObject BuildPlannerObject(const std::string& obj_id,
                                 const Vec2d& obj_pos, double vel,
                                 double heading = 0.0) {
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id(obj_id)
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(obj_pos)
                            .set_length_width(4.2, 2.1)
                            .set_yaw(heading)
                            .set_velocity(vel)
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(false)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.8)
      .set_straight_line(obj_pos,
                         obj_pos + Vec2d::FastUnitFromAngle(heading) * vel *
                                       prediction::kPredictionDuration,
                         /*init_v=*/vel, /*last_v=*/vel);
  return builder.Build();
}

TEST(TrafficGapFinder, FindBestTrafficGapOnRouteTarget) {
  const auto& psmm = CreateDojoTestPSMM();

  const double ego_v = 6.0;  // m/s.
  const Box2d ego_box(/*half_length=*/2.1, /*half_width=*/1.05,
                      Vec2d(136.7, 70.0), /*heading=*/0.0);
  SendBox2dToCanvas(ego_box, "gap_finder/candidates/ego_vehicle",
                    vis::Color::kOrange);
  const double speed_limit = 11.11;  // m/s.

  const mapping::LanePath cur_lane_path = *BuildLanePathFromData(
      mapping::LanePathData(
          /*start_fraction=*/0.0, /*end_fraction=*/1.0,
          {mapping::ElementId(2475), mapping::ElementId(2490),
           mapping::ElementId(2485)}),
      psmm);
  const auto cur_frenet_frame =
      *BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, cur_lane_path),
                              /*down_sample_raw_points=*/true);
  const auto cur_ego_fbox = *cur_frenet_frame.QueryFrenetBoxAt(ego_box);
  const double max_reach_length =
      cur_lane_path.length() - cur_ego_fbox.s_min - 20.0;

  EXPECT_NEAR(max_reach_length, 80.0, 0.1);

  const mapping::LanePath target_lane_path = *BuildLanePathFromData(
      mapping::LanePathData(
          /*start_fraction=*/0.0, /*end_fraction=*/1.0,
          {mapping::ElementId(2476), mapping::ElementId(2491),
           mapping::ElementId(2484)}),
      psmm);
  DrawLanePathToCanvas(psmm, target_lane_path,
                       "gap_finder/candidates/target_lane_path",
                       vis::Color::kGreen);
  const auto target_frenet_frame =
      *BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path),
                              /*down_sample_raw_points=*/true);
  const auto target_ego_fbox = *target_frenet_frame.QueryFrenetBoxAt(ego_box);

  ObjectVector<PlannerObject> objects;
  objects.push_back(
      BuildPlannerObject("First", Vec2d(160.0, 66.5), /*vel=*/7.0));
  objects.push_back(
      BuildPlannerObject("Second", Vec2d(145.0, 66.0), /*vel=*/5.0));
  objects.push_back(
      BuildPlannerObject("Third", Vec2d(120.0, 67.0), /*vel=*/6.0));
  const PlannerObjectManager obj_mgr(objects);
  DrawPlannerObjectManagerToCanvas(obj_mgr, "gap_finder/candidates/objects",
                                   vis::Color::kLightGreen);
  const SpacetimeTrajectoryManager target_st_traj_mgr(
      obj_mgr.planner_objects());

  {
    TrafficGapDebugProto debug_info;
    const auto gap_res = FindBestTrafficGapOnRouteTarget(
        cur_frenet_frame, cur_ego_fbox,
        /*cur_st_traj_mgr=*/SpacetimeTrajectoryManager(), target_frenet_frame,
        target_ego_fbox, target_st_traj_mgr, ego_v, /*ego_a=*/0.0,
        max_reach_length, speed_limit, /*prev_leader=*/"", /*prev_follower=*/"",
        &debug_info);
    EXPECT_EQ(debug_info.gap_candidates().size(), 3);
    ASSERT_TRUE(gap_res.leader_id.has_value());
    EXPECT_EQ(*gap_res.leader_id, "Second");
    ASSERT_TRUE(gap_res.follower_id.has_value());
    EXPECT_EQ(*gap_res.follower_id, "Third");
  }
  {
    ObjectVector<PlannerObject> objects;
    objects.push_back(
        BuildPlannerObject("Cap", Vec2d(150.0, 70.0), /*vel=*/5.0));
    const PlannerObjectManager cur_obj_mgr(objects);
    DrawPlannerObjectManagerToCanvas(cur_obj_mgr, "gap_finder/candidates/cap",
                                     vis::Color::kLightBlue);
    const SpacetimeTrajectoryManager cur_st_traj_mgr(
        cur_obj_mgr.planner_objects());

    TrafficGapDebugProto debug_info;
    const auto gap_res = FindBestTrafficGapOnRouteTarget(
        cur_frenet_frame, cur_ego_fbox, cur_st_traj_mgr, target_frenet_frame,
        target_ego_fbox, target_st_traj_mgr, ego_v, /*ego_a=*/0.0,
        max_reach_length, speed_limit, /*prev_leader=*/"", /*prev_follower=*/"",
        &debug_info);
    EXPECT_EQ(debug_info.gap_candidates().size(), 2);
    ASSERT_TRUE(gap_res.leader_id.has_value());
    EXPECT_EQ(*gap_res.leader_id, "Second");
    ASSERT_TRUE(gap_res.follower_id.has_value());
    EXPECT_EQ(*gap_res.follower_id, "Third");
  }
}

}  // namespace

}  // namespace qcraft::planner
