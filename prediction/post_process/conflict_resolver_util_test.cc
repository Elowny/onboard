#include "onboard/prediction/post_process/conflict_resolver_util.h"

#include "gtest/gtest.h"

#include "onboard/planner/test_util/predicted_trajectory_builder.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(ConflictResolverUtilTest, PredictedTrajectoryToPurePathAndSpeedVector) {
  const auto pred_traj =
      planner::PredictedTrajectoryBuilder()
          .set_probability(1.0)
          .set_straight_line(
              /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(50.0, 0.0),
              /*init_v=*/5.0, /*last_v=*/5.0)
          .Build();
  const auto path_speed =
      PredictedTrajectoryToPurePathAndSpeedVector(pred_traj);
  EXPECT_EQ(path_speed.first.size(), pred_traj.points().size());
  EXPECT_NEAR(path_speed.first.length(), 50.0, 1e-3);
  EXPECT_EQ(path_speed.second.size(), pred_traj.points().size());
}
TEST(ConflictResolverUtilTest,
     PredictedTrajectoryPointProtoToPurePathAndSpeedVector) {
  const auto pred_traj =
      planner::PredictedTrajectoryBuilder()
          .set_probability(1.0)
          .set_straight_line(
              /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(50.0, 0.0),
              /*init_v=*/5.0, /*last_v=*/5.0)
          .Build();
  std::vector<PredictedTrajectoryPointProto> point_protos;
  point_protos.reserve(pred_traj.points().size());
  for (const auto& pt : pred_traj.points()) {
    point_protos.emplace_back();
    pt.ToProto(&point_protos.back());
  }
  const auto path_speed =
      PredictedTrajectoryPointProtoToPurePathAndSpeedVector(point_protos);
  EXPECT_EQ(path_speed.first.size(), point_protos.size());
  EXPECT_NEAR(path_speed.first.length(), 50.0, 1e-3);
  EXPECT_EQ(path_speed.second.size(), point_protos.size());
}

TEST(ConflictResolverUtilTest, EdgeConnectionToSimpleSpeedProfile) {
  std::vector<std::string> cost_names = {"collision_cost"};
  SvtEdgeCost edge_cost = SvtEdgeCost{
      .feature_costs = {10.0},
      .sum_cost = 10.0,
  };
  std::vector<SvtState> states = {SvtState{.s = 2.0, .v = 2.0, .t = 1.0}};
  std::vector<SvtEdge> edges = {SvtEdge{
      .start_index = SvtNodeIndex(0),
      .end_index = SvtNodeIndex(1),
      .a = 0.0,
      .index = SvtEdgeIndex(0),
      .final_t = 1.0,
      .final_v = 2.0,
      .states = &states,
      .edge_cost = &edge_cost,
  }};
  SvtEdgeVector<SvtEdgeCost> search_costs;
  search_costs.push_back(edge_cost);
  std::vector<SvtEdgeIndex> edge_idxes = {SvtEdgeIndex(0)};
  const auto speed_profile = EdgeConnectionToSimpleSpeedProfile(
      absl::MakeSpan(cost_names), edges, search_costs, edge_idxes);
  EXPECT_EQ(speed_profile.final_t(), 1.0);
  EXPECT_EQ(speed_profile.final_v(), 2.0);
  EXPECT_EQ(speed_profile.total_cost(), 10.0);
  EXPECT_EQ(speed_profile.eval_cost(), 10.0);
  EXPECT_EQ(speed_profile.edges().size(), 1);
}

TEST(ConflictResolverUtilTest, SpeedVectorToSpeedProfile) {
  std::vector<SvtEdgeIndex> edge_idxes{SvtEdgeIndex(0)};
  std::vector<planner::SpeedPoint> points;
  points.reserve(4);
  for (int i = 0; i < 4; ++i) {
    planner::SpeedPoint speed_pt(0.1 * i, 1.0 * i, 1.0, 0.0, 0.0);
    points.push_back(speed_pt);
  }
  const auto speed_profile =
      SpeedVectorToSpeedProfile(planner::SpeedVector(points), edge_idxes);
  EXPECT_EQ(speed_profile.points().size(), 4);
  EXPECT_EQ(speed_profile.edge_idxes().size(), 1);
}
TEST(ConflictResolverUtilTest, SvtStateToSpeedPoint) {
  const auto speed_pt =
      SvtStateToSpeedPoint(SvtState{.s = 10.0, .v = 5.0, .t = 1.0}, /*a=*/0.5);
  EXPECT_EQ(speed_pt.s(), 10.0);
  EXPECT_EQ(speed_pt.v(), 5.0);
  EXPECT_EQ(speed_pt.t(), 1.0);
  EXPECT_EQ(speed_pt.a(), 0.5);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
