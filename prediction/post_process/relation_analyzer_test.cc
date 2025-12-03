#include "onboard/prediction/post_process/relation_analyzer.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
namespace {

// Only filling trajectory points.
TrajectoryProto PredictedTrajectoryPointsToTrajectoryProto(
    absl::Span<const PredictedTrajectoryPoint> pred_points) {
  auto apollo_points = planner::ToApolloTrajectoryPointProto(pred_points);
  TrajectoryProto traj_proto;
  for (auto& apollo_point : apollo_points) {
    *traj_proto.add_trajectory_point() = std::move(apollo_point);
  }
  return traj_proto;
}

ApolloTrajectoryPointProto GenerateTrajectoryPoint(const Vec2d& xy, double s,
                                                   double theta, double v,
                                                   double relative_time) {
  ApolloTrajectoryPointProto apollo_traj_point;
  apollo_traj_point.mutable_path_point()->set_x(xy.x());
  apollo_traj_point.mutable_path_point()->set_y(xy.y());
  apollo_traj_point.mutable_path_point()->set_s(s);
  apollo_traj_point.mutable_path_point()->set_theta(theta);
  apollo_traj_point.set_v(v);
  apollo_traj_point.set_relative_time(relative_time);

  return apollo_traj_point;
}

TrajectoryProto GenerateTrajectory(double trajectory_start_timestamp,
                                   const Vec2d& start_point, double theta,
                                   double v) {
  TrajectoryProto trajectory_proto;
  trajectory_proto.set_trajectory_start_timestamp(trajectory_start_timestamp);

  constexpr int kNumOfTrajPoint = 100;
  constexpr double kTrajPointInterval = 0.1;  // s

  *trajectory_proto.add_trajectory_point() = GenerateTrajectoryPoint(
      start_point, /* s = */ 0.0, theta, v, /* relative_time = */ 0.0);

  for (int i = 1; i < kNumOfTrajPoint; ++i) {
    const double s = v * kTrajPointInterval * i;
    Vec2d traj_point_xy = start_point + Vec2d::UnitFromAngle(theta) * s;
    const double relative_time = i * kTrajPointInterval;

    *trajectory_proto.add_trajectory_point() =
        GenerateTrajectoryPoint(traj_point_xy, s, theta, v, relative_time);
  }

  trajectory_proto.set_low_speed_freespace(false);
  trajectory_proto.set_aeb_triggered(false);

  return trajectory_proto;
}

TEST(RelationAnalyzerTest, FindAvObjMeetingPoints) {
  const auto av_traj =
      GenerateTrajectory(/*trajectory_start_timestamp*/ 1.0,
                         /*start_point*/ Vec2d(-48.0, 3.5), /*theta*/ 0.0,
                         /*v*/ 5.0);
  const auto av_segment_matcher_ptr =
      TrajectoryProtoToSegmentMatcherKdtree(av_traj);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_pos(Vec2d(-14.0, -22.0))
                       .set_velocity(4.0)
                       .set_yaw(M_PI_2)
                       .Build();
  const auto obj_pred_traj =
      planner::PredictedTrajectoryBuilder()
          .set_probability(1.0)
          .set_straight_line(
              /*start=*/Vec2d(-14.0, -22.0), /*end=*/Vec2d(-14.0, 18.0),
              /*init_v=*/4.0, /*last_v=*/4.0)
          .Build();
  std::vector<Box2dProto> box2d_protos;
  std::vector<Box2dProto*> obj_shapes;
  box2d_protos.reserve(obj_pred_traj.points().size());
  obj_shapes.reserve(obj_pred_traj.points().size());

  for (const auto& traj_point : obj_pred_traj.points()) {
    auto& box2d_proto = box2d_protos.emplace_back();
    box2d_proto.set_x(traj_point.pos().x());
    box2d_proto.set_y(traj_point.pos().y());
    box2d_proto.set_heading(traj_point.theta());
    box2d_proto.set_width(2.0);
    box2d_proto.set_length(4.0);
    obj_shapes.push_back(&box2d_proto);
  }

  const auto meeting_points_or = FindAvObjMeetingPoints(
      av_traj, *av_segment_matcher_ptr.get(), obj, obj_pred_traj, obj_shapes);
  EXPECT_TRUE(meeting_points_or.has_value());
}

TEST(RelationAnalyzerTest, AnalyzeYieldObjectRelation) {
  const auto av_traj =
      GenerateTrajectory(/*trajectory_start_timestamp*/ 1.0,
                         /*start_point*/ Vec2d(-48.0, 3.5), /*theta*/ 0.0,
                         /*v*/ 5.0);
  const auto av_segment_matcher_ptr =
      TrajectoryProtoToSegmentMatcherKdtree(av_traj);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_pos(Vec2d(-14.0, -22.0))
                       .set_velocity(4.0)
                       .set_yaw(M_PI_2)
                       .Build();
  const auto obj_pred_traj =
      planner::PredictedTrajectoryBuilder()
          .set_probability(1.0)
          .set_straight_line(
              /*start=*/Vec2d(-14.0, -22.0), /*end=*/Vec2d(-14.0, 18.0),
              /*init_v=*/4.0, /*last_v=*/4.0)
          .Build();
  std::vector<Box2dProto> box2d_protos;
  std::vector<Box2dProto*> obj_shapes;
  box2d_protos.reserve(obj_pred_traj.points().size());
  obj_shapes.reserve(obj_pred_traj.points().size());

  for (const auto& traj_point : obj_pred_traj.points()) {
    auto& box2d_proto = box2d_protos.emplace_back();
    box2d_proto.set_x(traj_point.pos().x());
    box2d_proto.set_y(traj_point.pos().y());
    box2d_proto.set_heading(traj_point.theta());
    box2d_proto.set_width(2.0);
    box2d_proto.set_length(4.0);
    obj_shapes.push_back(&box2d_proto);
  }

  const auto relation_proto =
      AnalyzeObjectRelation(av_traj, *av_segment_matcher_ptr.get(), obj,
                            obj_pred_traj, obj_shapes, /*av_box=*/std::nullopt);
  EXPECT_EQ(relation_proto.av_relation(), ObjectRelation::OR_YIELD);
}

TEST(RelationAnalyzerTest, AnalyzeStationhyTrajectoryRelation) {
  PredictedTrajectory pred_traj_1, pred_traj_2;
  pred_traj_1.set_type(PT_STATIONARY);
  pred_traj_2.set_type(PT_STATIONARY);
  const auto input_1 =
      AgentRelationAnalyzerInput{.traj_id = "1_0", .trajectory = &pred_traj_1};
  const auto input_2 =
      AgentRelationAnalyzerInput{.traj_id = "2_0", .trajectory = &pred_traj_2};
  const auto relation_type = AnalyzeTrajectoryRelation(input_1, input_2);
  EXPECT_EQ(relation_type, AgentRelationType::ART_NONE);
}

TEST(RelationAnalyzerTest, AnalyzeVoidObjectRelation) {
  const auto av_traj =
      GenerateTrajectory(/*trajectory_start_timestamp*/ 1.0,
                         /*start_point*/ Vec2d(-48.0, 3.5), /*theta*/ 0.0,
                         /*v*/ 5.0);
  const auto av_segment_matcher_ptr =
      TrajectoryProtoToSegmentMatcherKdtree(av_traj);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id("1")
                       .set_type(OT_UNKNOWN_STATIC)
                       .set_pos(Vec2d(-14.0, -22.0))
                       .set_velocity(0.0)
                       .set_yaw(M_PI_2)
                       .Build();
  const prediction::PredictedTrajectory obj_pred_traj;
  std::vector<Box2dProto*> obj_shapes;
  const auto relation_proto =
      AnalyzeObjectRelation(av_traj, *av_segment_matcher_ptr.get(), obj,
                            obj_pred_traj, obj_shapes, /*av_box=*/std::nullopt);
  EXPECT_EQ(relation_proto.av_relation(), ObjectRelation::OR_VOID);
}

TEST(BuildPriorityGraph, BuildPriorityGraphTest) {
  // 0 -> 2
  // 0 -> 3
  // 2 -> 3
  // 1 -> 3
  // Output should be: 0,1; 2; 3.
  std::vector<TrajectoryNode> nodes;
  nodes.push_back(TrajectoryNode({
      .index = TrajectoryNodeIndex(0),
      .traj_id = "node_0",
      .influencers = {},
      .reactors = {TrajectoryNodeIndex(2), TrajectoryNodeIndex(3)},
  }));
  nodes.push_back(TrajectoryNode({
      .index = TrajectoryNodeIndex(1),
      .traj_id = "node_1",
      .influencers = {},
      .reactors = {TrajectoryNodeIndex(3)},
  }));

  nodes.push_back(TrajectoryNode({
      .index = TrajectoryNodeIndex(2),
      .traj_id = "node_2",
      .influencers = {TrajectoryNodeIndex(0)},
      .reactors = {TrajectoryNodeIndex(3)},
  }));
  nodes.push_back(TrajectoryNode({
      .index = TrajectoryNodeIndex(3),
      .traj_id = "node_3",
      .influencers = {TrajectoryNodeIndex(0), TrajectoryNodeIndex(1),
                      TrajectoryNodeIndex(2)},
      .reactors = {},
  }));

  const auto result = BuildPriorityGraph(&nodes);
}

TEST(FindAvObjMeetingPointsWithShape, Works) {
  // AV going straight.
  UniCycleState av_start_state{
      .x = 0.0,
      .y = 0.0,
      .v = 30.0,  // m/s.
      .heading = 0.0,
      .yaw_rate = 0.0,
      .acc = 0.0,
  };
  const auto av_pts = DevelopCYCVTrajectory(
      av_start_state, /*dt=*/0.1, /*horizon=*/10.0, /*is_reversed=*/false);
  const auto av_traj = PredictedTrajectoryPointsToTrajectoryProto(av_pts);
  const auto av_box =
      Box2d(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/5.0, /*width=*/2.0);
  const auto av_segment_matcher =
      TrajectoryProtoToSegmentMatcherKdtree(av_traj);

  // Object merging.
  UniCycleState obj_start_state{
      .x = 0.0,
      .y = -5.0,
      .v = 20.0,
      .heading = M_PI / 6.0,            // 30 deg.
      .yaw_rate = (M_PI / 6.0) / 12.0,  // Back to 0 deg heading within 12.0 s.
      .acc = 0.0,
  };
  auto obj_pts = DevelopCYCVTrajectory(obj_start_state, /*dt=*/0.1,
                                       /*horizon=*/10.0, /*is_reversed=*/false);
  const auto obj_pred_traj = PredictedTrajectory(
      /*probability=*/1.0, "obj_mering_traj", PredictionType::PT_CYCV,
      /*index=*/0, std::move(obj_pts),
      /*is_reversed=*/false);
  const auto obj_box =
      Box2d(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/4.0, /*width=*/1.8);
  Box2dProto obj_box_proto;
  obj_box.ToProto(&obj_box_proto);
  std::vector<const Box2dProto*> obj_shapes(obj_pred_traj.points().size(),
                                            &obj_box_proto);

  planner::PerceptionObjectBuilder obj_builder;
  const auto obj_proto = obj_builder.set_id("obj")
                             .set_pos(Vec2d(0.0, -5.0))
                             .set_velocity(20.0)
                             .Build();

  const auto meeting_pts_opt =
      FindAvObjMeetingPointsWithShape(av_traj, *av_segment_matcher, av_box,
                                      obj_proto, obj_pred_traj, obj_shapes);
  EXPECT_TRUE(meeting_pts_opt.has_value());
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
