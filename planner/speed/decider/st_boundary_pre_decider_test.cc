#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/maps/lane_path.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace {

std::vector<StBoundaryWithDecision> BuildStBoundariesWithDecision(
    double low_s, double high_s, double total_plan_time) {
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {{low_s, 0.0}, {low_s, total_plan_time}};
  st_boundary_points.upper_points = {{high_s, 0.0}, {high_s, total_plan_time}};
  st_boundary_points.speed_points = {{0.5, 0.0}, {0.5, total_plan_time}};
  st_boundary_points.overlap_infos = {OverlapInfo{
      .time = 0.0, .obj_idx = 0, .av_start_idx = 1, .av_end_idx = 1}};

  std::vector<StBoundaryRef> st_boundaries;

  StBoundaryRef st_boundary0 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "Agent0-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  st_boundary0->set_source_type(StBoundarySourceTypeProto::ST_OBJECT);
  st_boundaries.push_back(std::move(st_boundary0));

  StBoundaryRef st_boundary1 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "Agent1-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  st_boundary1->set_source_type(StBoundarySourceTypeProto::ST_OBJECT);
  st_boundaries.push_back(std::move(st_boundary1));

  StBoundaryRef st_boundary2 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "Agent2-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::LANE_CHANGE_GAP,
      /*is_large_vehicle=*/false);
  st_boundary2->set_source_type(StBoundarySourceTypeProto::ST_OBJECT);
  st_boundaries.push_back(std::move(st_boundary2));

  StBoundaryRef st_boundary3 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "Agent3-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::LANE_CHANGE_GAP,
      /*is_large_vehicle=*/false);
  st_boundary3->set_source_type(StBoundarySourceTypeProto::ST_OBJECT);
  st_boundaries.push_back(std::move(st_boundary3));

  StBoundaryRef st_boundary4 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "Agent4-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT,
      /*is_large_vehicle=*/false);
  st_boundary4->set_source_type(StBoundarySourceTypeProto::ST_OBJECT);
  st_boundaries.push_back(std::move(st_boundary4));

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.reserve(st_boundaries.size());

  for (auto& st_boundary : st_boundaries) {
    // Initialize with unknown-decision.
    st_boundaries_with_decision.emplace_back(std::move(st_boundary));
    st_boundaries_with_decision.back().set_decision_type(
        StBoundaryProto::UNKNOWN);
  }

  return st_boundaries_with_decision;
}

TEST(StBoundaryPreDeciderTest, MakeFreespacePreDecisionForStBoundaries) {
  const double low_s = 0.0, high_s = 4.0, total_plan_time = 10.0;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision =
      BuildStBoundariesWithDecision(low_s, high_s, total_plan_time);
  MakeFreespacePreDecisionForStBoundaries(&st_boundaries_with_decision);

  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    EXPECT_EQ(st_boundary_with_decision.decision_type(),
              StBoundaryProto::IGNORE);
    EXPECT_EQ(st_boundary_with_decision.decision_reason(),
              StBoundaryProto::IGNORE_TRACKER_DECIDER);
  }
}

TEST(StBoundaryPreDeciderTest, MakePreDecisionForStBoundaries) {
  const double low_s = 0.0, high_s = 4.0, total_plan_time = 10.0;
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision =
      BuildStBoundariesWithDecision(low_s, high_s, total_plan_time);
  std::optional<VtSpeedLimit> parallel_cut_in_speed_limit = std::nullopt;

  SpeedFinderParamsProto::StBoundaryPreDeciderParamsProto
      vehicle_geometry_params;
  vehicle_geometry_params.set_enable_parallel_cut_in_pre_brake(true);

  ConstraintProto::LeadingObjectProto leading_obj;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs{
      {"Agent1-idx0", leading_obj}};

  absl::flat_hash_set<std::string> follower_set;
  follower_set.insert("Agent0");

  TrafficGapResult lane_change_gap;
  lane_change_gap.leader_id = "Agent2";
  lane_change_gap.follower_id = "Agent3";

  PathPoint point1, point2;
  point1.set_x(1.0);
  point1.set_y(1.0);
  point1.set_theta(0.2);
  point2.set_x(1.5);
  point2.set_y(1.5);
  point2.set_theta(0.2);
  DiscretizedPath path;
  path.push_back(point1);
  path.push_back(point2);

  VehicleGeometryParamsProto vehicle_geo_params;
  vehicle_geo_params.set_width(1.8);
  vehicle_geo_params.set_front_edge_to_center(1.2);

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(5.0);
  av_pose.mutable_pos_smooth()->set_y(0.0);
  av_pose.set_yaw(0.15);
  av_pose.mutable_vel_body()->set_x(Kph2Mps(60.0));  // 60 kph.
  const auto& psmm = CreateDojoTestPSMM();
  const double credible_length = 100.0;
  const auto curr_lane_path_or =
      FindNearestLanePathFromEgoPose(av_pose, psmm, credible_length);
  EXPECT_TRUE(curr_lane_path_or.ok());
  absl::StatusOr<DrivePassage> drive_passages = BuildDrivePassageFromLanePath(
      psmm, *curr_lane_path_or, /*step_s=*/2.0,
      /*avoid_loop=*/true, /*backward_extend_len=*/10.0,
      /*required_planning_horizon=*/0.0, /*required_backward_len=*/0.0,
      std::nullopt);
  EXPECT_TRUE(drive_passages.ok());
  std::vector<PlannerObject> planner_objects;
  planner_objects.reserve(5);
  for (int i = 0; i < 4; ++i) {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj =
        perception_builder.set_id(absl::StrFormat("Agent%d", i))
            .set_type(OT_VEHICLE)
            .set_timestamp(1.0)
            .set_velocity(0.0)
            .set_yaw(0.0)
            .set_length_width(4.5, 2.0)
            .set_pos(Vec2d(160.6 + i * 10.0, 66.6))
            .set_box_center(Vec2d(160.6 + i * 10.0, 66.6))
            .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
  }
  PerceptionObjectBuilder perception_builder4;
  const auto perception_obj4 =
      perception_builder4.set_id("Agent4")
          .set_type(OT_VEHICLE)
          .set_timestamp(1.0)
          .set_velocity(0.0)
          .set_yaw(-0.02)
          .set_length_width(10.0, 3.0)
          .set_pos(Vec2d(5.0, 2.0))
          .set_box_center(Vec2d(5.0, 2.0))
          .set_contour(Polygon2d(Box2d(Vec2d(5.0, 2.0), -0.02, 10.0, 3.0)))
          .Build();

  PlannerObjectBuilder builder4;
  builder4.set_type(OT_VEHICLE)
      .set_object(perception_obj4)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2dFromProto(perception_obj4.pos()),
                            perception_obj4.yaw())
      .set_probability(0.5);

  PlannerObject object4 = builder4.Build();
  planner_objects.push_back(std::move(object4));
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  PreDeciderInput pre_decider_input = PreDeciderInput({
      .params = &vehicle_geometry_params,
      .leading_trajs = &leading_trajs,
      .follower_set = &follower_set,
      .lane_change_gap = &lane_change_gap,
      .st_traj_mgr = &st_traj_mgr,
      .path = &path,
      .vehicle_geo_params = &vehicle_geo_params,
      .drive_passage = &drive_passages.value(),
      .current_v = 5.0,
      .max_v = 10.0,
      .time_step = 0.1,
      .trajectory_steps = 5,
  });

  MakePreDecisionForStBoundaries(pre_decider_input,
                                 &st_boundaries_with_decision,
                                 &parallel_cut_in_speed_limit);

  EXPECT_EQ(st_boundaries_with_decision[0].decision_type(),
            StBoundaryProto::IGNORE);
  EXPECT_EQ(st_boundaries_with_decision[0].decision_reason(),
            StBoundaryProto::PRE_DECIDER);
  EXPECT_EQ(st_boundaries_with_decision[1].decision_type(),
            StBoundaryProto::FOLLOW);
  EXPECT_EQ(st_boundaries_with_decision[1].decision_reason(),
            StBoundaryProto::PRE_DECIDER);
  EXPECT_EQ(st_boundaries_with_decision[2].decision_type(),
            StBoundaryProto::FOLLOW);
  EXPECT_EQ(st_boundaries_with_decision[2].decision_reason(),
            StBoundaryProto::PRE_DECIDER);
  EXPECT_EQ(st_boundaries_with_decision[3].decision_type(),
            StBoundaryProto::LEAD);
  EXPECT_EQ(st_boundaries_with_decision[3].decision_reason(),
            StBoundaryProto::PRE_DECIDER);
  EXPECT_EQ(st_boundaries_with_decision[4].decision_type(),
            StBoundaryProto::FOLLOW);
  EXPECT_EQ(st_boundaries_with_decision[4].decision_reason(),
            StBoundaryProto::PRE_DECIDER);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
