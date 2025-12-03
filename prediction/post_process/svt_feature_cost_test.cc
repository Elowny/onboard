#include "onboard/prediction/post_process/svt_feature_cost.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/object_svt_sample.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

namespace prediction {

namespace {

using StBoundaryRef = planner::StBoundaryRef;
using StBoundaryPoints = planner::StBoundaryPoints;

TEST(SvtFeatureCost, ConstV) {
  // Load params.
  ConflictResolverParams params;
  params.LoadParams();

  // Const V sample. 6m/s 15m.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 6.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 15.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);

  // Object Collision.
  std::vector<double> stationary_objects;
  stationary_objects.push_back(10.0);

  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);

  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_collision_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, object_config.stationary_follow_distance());

  // Reference speed 5m/s and progress feature.
  std::vector<double> v;
  std::vector<double> t;
  std::vector<double> s;
  const int point_size = 16;
  for (int i = 0; i < point_size; ++i) {
    v.push_back(5.0);
    t.push_back(i);
    s.push_back(i);
  }
  std::vector<planner::SpeedPoint> pts;
  pts.reserve(v.size());
  for (int i = 0; i < v.size(); ++i) {
    pts.push_back(planner::SpeedPoint(t[i], s[i], v[i], /*a=*/0.0, /*j=*/0.0));
  }
  planner::SpeedVector ref_speed(std::move(pts));
  std::unique_ptr<SvtReferenceSpeedFeatureCost> svt_ref_speed_cost =
      std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed);

  std::vector<double> collision_costs;
  collision_costs.resize(1);
  std::vector<double> reference_speed_costs;
  reference_speed_costs.resize(2);

  svt_collision_cost->ComputeFeatureCost(states,
                                         absl::MakeSpan(collision_costs));
  svt_ref_speed_cost->ComputeFeatureCost(states,
                                         absl::MakeSpan(reference_speed_costs));
  QCHECK_GT(collision_costs[0], 0.0);
  QCHECK_GT(reference_speed_costs[0], 0.0);
  QCHECK_EQ(reference_speed_costs[1], 0.0);
}

TEST(SvtFeatureCost, SvtObjectCollisionFeatureCost_ConstV) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  stationary_objects.push_back(2.0);
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, object_config.stationary_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 5.0;
  const double ds = 0.1;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(1);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost,
     SvtObjectCollisionFeatureCost_DecelerateBeforeStationaryObject) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  stationary_objects.push_back(2.0);
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, object_config.stationary_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 5.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(1);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  svt_feature_cost->ComputeFeatureCost({states.front()}, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost,
     SvtObjectCollisionFeatureCost_ConstVelocityAcrossStopline) {
  ConflictResolverParams params;
  params.LoadParams();

  std::vector<double> stationary_objects;
  stationary_objects.push_back(2.0);
  const auto& object_config =
      params.GetConfigByObjectType(ObjectType::OT_VEHICLE);
  std::unique_ptr<SvtObjectCollisionFeatureCost> svt_feature_cost =
      std::make_unique<SvtObjectCollisionFeatureCost>(
          stationary_objects, object_config.stationary_follow_distance());

  // Sample const velocity edge.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 1.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 5.0;
  const double ds = 0.2;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(1);
  svt_feature_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost, SvtReferenceSpeedFeatureCost_ConstVelocity) {
  std::vector<double> v = {1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> s = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<planner::SpeedPoint> pts;
  pts.reserve(v.size());
  for (int i = 0; i < v.size(); ++i) {
    pts.push_back(planner::SpeedPoint(t[i], s[i], v[i], /*a=*/0.0, /*j=*/0.0));
  }
  planner::SpeedVector ref_speed(std::move(pts));
  std::unique_ptr<SvtReferenceSpeedFeatureCost> svt_ref_speed_cost =
      std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed);

  // Get svt states samples.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 2.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 3.0;
  const double ds = 0.1;
  const double a = 0.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_ref_speed_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
}

TEST(SvtFeatureCost, SvtReferenceSpeedFeatureCost_Deceleration) {
  std::vector<double> v = {1.0, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> s = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<planner::SpeedPoint> pts;
  pts.reserve(v.size());
  for (int i = 0; i < v.size(); ++i) {
    pts.push_back(planner::SpeedPoint(t[i], s[i], v[i], /*a=*/0.0, /*j=*/0.0));
  }
  planner::SpeedVector ref_speed(std::move(pts));
  std::unique_ptr<SvtReferenceSpeedFeatureCost> svt_ref_speed_cost =
      std::make_unique<SvtReferenceSpeedFeatureCost>(&ref_speed);

  // Get svt states samples.
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 2.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 3.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> costs;
  costs.resize(2);
  svt_ref_speed_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  // It completely stops, which is very different from original speed vector.
}
TEST(SvtFeatureCostTest, StopLineCost) {
  std::vector<SvtNode> nodes;
  const SvtNode start_node({
      .s = 0.0,
      .v = 2.0,
      .t = 0.0,
      .index = SvtNodeIndex(0),
      .s_index = 0,
  });
  nodes.push_back(start_node);
  const double length = 3.0;
  const double ds = 0.1;
  const double a = -1.0;
  DpEdgeInfo edge_info({
      .start_index = SvtNodeIndex(0),
      .a = a,
      .length = length,
  });
  const auto states = SampleDp(nodes, edge_info, ds);
  std::vector<double> stop_lines = {1.0};
  std::unique_ptr<SvtStoplineFeatureCost> svt_stopline_cost =
      std::make_unique<SvtStoplineFeatureCost>(stop_lines);
  std::vector<double> costs = {0.0};
  svt_stopline_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  EXPECT_GT(costs[0], 0.0);
  std::vector<double> empty_stop_line;
  std::unique_ptr<SvtStoplineFeatureCost> empty_stop_line_cost =
      std::make_unique<SvtStoplineFeatureCost>(empty_stop_line);
  empty_stop_line_cost->ComputeFeatureCost(states, absl::MakeSpan(costs));
  EXPECT_EQ(costs[0], 0.0);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
