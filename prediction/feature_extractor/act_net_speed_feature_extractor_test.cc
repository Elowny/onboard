#include "onboard/prediction/feature_extractor/act_net_speed_feature_extractor.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace prediction {
namespace {

const int kNumHistory = 10;

const Vec2d kAgentInitPos = {0.0, 0.0};
constexpr double kAgentVel = 5.0;  // m/s.
constexpr double kAgentHeading = 0.0;
const Vec2d kAVInitPos = {3.0, 0.0};
constexpr double kAVHeading = 0.0;
constexpr double kAVVel = 5.0;
const Vec2d kObject1InitPos = {6.0, 1.0};
constexpr double kObject1Heading = 0.0;
constexpr double kObject1Vel = 5.0;

constexpr double kUpdateTimeStep = 0.1;
constexpr double kStartTs = 0.0;
constexpr double kDt = 0.1;
constexpr double kWidth = 2.0;
constexpr double kLength = 3.4;

constexpr char kObject1Id[] = "object1";
constexpr char kAgentId[] = "agent";

ObjectHistory ObjectMotionHistoryToObjectHistory(
    const ObjectMotionHistory& motion_history) {
  CoordinateConverter coord_converter;
  ObjectHistory obj_history(motion_history.states.size() * 2);
  planner::PerceptionObjectBuilder obj_builder;
  for (const auto& state : motion_history.states) {
    auto percep_obj = obj_builder.set_id(motion_history.id)
                          .set_pos(state.pos)
                          .set_box_center(state.pos)
                          .set_velocity(state.vel.norm())
                          .set_yaw(state.heading)
                          .set_accel(Vec2d::Zero())
                          .Build();
    obj_history.Push(state.timestamp,
                     PredictionObject(std::move(percep_obj), coord_converter));
  }
  return obj_history;
}

ObjectHistorySampler MakeObjectHistorySampler(const int history_num) {
  // Agent.
  auto agent_states = BuildRawTrackerHistoryCTRA(
      kAgentInitPos, kAgentVel, /*acc=*/0.0, kAgentHeading, /*yaw_rate=*/0.0,
      kLength, kWidth, history_num, kStartTs, kDt);
  ObjectMotionHistory agent_motion_hist;
  agent_motion_hist.id = kAgentId;
  agent_motion_hist.type = OT_VEHICLE;
  agent_motion_hist.states = std::move(agent_states);
  auto agent_hist = ObjectMotionHistoryToObjectHistory(agent_motion_hist);

  // AV.
  auto av_states = BuildRawTrackerHistoryCTRA(
      kAVInitPos, kAVVel, /*acc=*/0.0, kAVHeading, /*yaw_rate=*/0.0, kLength,
      kWidth, history_num, kStartTs, kDt);
  ObjectMotionHistory av_motion_hist;
  av_motion_hist.id = kAvObjectId;
  av_motion_hist.type = OT_VEHICLE;
  av_motion_hist.states = std::move(av_states);
  auto av_hist = ObjectMotionHistoryToObjectHistory(av_motion_hist);

  // Object1.
  auto obj1_states = BuildRawTrackerHistoryCTRA(
      kObject1InitPos, kObject1Vel, /*acc=*/0.0, kObject1Heading,
      /*yaw_rate=*/0.0, kLength, kWidth, history_num, kStartTs, kDt);
  ObjectMotionHistory obj1_motion_hist;
  obj1_motion_hist.id = kObject1Id;
  obj1_motion_hist.type = OT_VEHICLE;
  obj1_motion_hist.states = std::move(obj1_states);
  auto obj1_hist = ObjectMotionHistoryToObjectHistory(obj1_motion_hist);

  return ObjectHistorySampler(
      {&agent_hist, &obj1_hist}, av_hist,
      /*current_time=*/kUpdateTimeStep * (history_num - 1) - 0.01,
      kUpdateTimeStep, history_num,
      /*enable_smoothing=*/false,
      /*use_tracker_history=*/false);
}

TEST(ActNetSpeedFeatureExtractorTest, ExtractActNetFeatureWithAVOnly) {
  const auto obj_sampler = MakeObjectHistorySampler(kNumHistory);
  const auto* av_hist =
      obj_sampler.GetResampledMotionHistoryPtrById(kAvObjectId);
  const auto* agent_hist =
      obj_sampler.GetResampledMotionHistoryPtrById(kAgentId);
  EXPECT_NE(av_hist, nullptr);
  EXPECT_NE(agent_hist, nullptr);
  auto agent_act_net_feature =
      ExtractActNetFeatureWithAVOnly(kAgentId, *agent_hist, *av_hist);
  EXPECT_EQ(agent_act_net_feature.context_obj_features.size(), 1);
  EXPECT_TRUE(agent_act_net_feature.lc_features.empty());
  EXPECT_TRUE(agent_act_net_feature.solid_lb_features.empty());
  EXPECT_TRUE(agent_act_net_feature.cw_features.empty());
  EXPECT_EQ(agent_act_net_feature.agent_id, kAgentId);
}

TEST(ActNetSpeedFeatureExtractorTest,
     ExtractActNetSpeedFeaturesForPlannerTarget) {
  // Build ActNetSpeedPlannerTarget.
  const Vec2d start(0.0, 0.0);
  const Vec2d end(100.0, 0.0);
  const double init_v = 10.0;
  const double last_v = 20.0;
  // Perception ObjectProto.
  planner::PerceptionObjectBuilder percep_builder;
  const auto object_proto = percep_builder.set_id(kObject1Id)
                                .set_type(ObjectType::OT_VEHICLE)
                                .set_pos(start)
                                .set_timestamp(0.0)
                                .set_length_width(3.4, 1.6)
                                .set_yaw(0.0)
                                .set_velocity(init_v)
                                .set_trajectory(20)
                                .Build();

  // Build PlannerObject.
  planner::PlannerObjectBuilder planner_obj_builder;
  planner_obj_builder.set_type(ObjectType::OT_VEHICLE)
      .set_object(object_proto)
      .set_pos(start)
      .set_v(init_v)
      .set_theta(0.0)
      .set_stationary(/*stationary=*/false);
  auto* obj_pred_builder = planner_obj_builder.get_object_prediction_builder();
  obj_pred_builder->set_object(object_proto);
  obj_pred_builder->add_predicted_trajectory()
      ->set_annotation("object_traj")
      .set_index(0)
      .set_probability(1.0)
      .set_straight_line(start, end, init_v, last_v);
  const auto planner_object = planner_obj_builder.Build();

  // Build SpacetimeObjectTrajectory.
  const auto st_traj = planner::SpacetimeObjectTrajectory(
      planner_object, /*traj_index=*/0, /*required_lateral_gap=*/0.2);

  // ActNetSpeedPlannerTarget.
  ActNetSpeedPlannerTarget target{
      .object_id = kObject1Id,
      .st_traj = &st_traj,
      .av_s = 49.0,
      .object_s = 48.0,
  };

  // Build AV discretized path.
  planner::PredictedTrajectoryBuilder av_traj_builder;
  const Vec2d av_start(50.0, -50.0);
  const Vec2d av_end(50.0, 50.0);
  const double av_init_v = 10.0;
  const double av_last_v = 20.0;
  const auto av_pred_traj =
      av_traj_builder.set_annotation("av_traj")
          .set_index(0)
          .set_straight_line(av_start, av_end, av_init_v, av_last_v)
          .set_probability(1.0)
          .Build();
  const auto av_discretized_path = PredictedTrajectoryPointsToDiscretizedPath(
      absl::MakeSpan(av_pred_traj.points()));

  // Build object act net feature.
  const auto obj_sampler = MakeObjectHistorySampler(kNumHistory);
  const auto* av_hist =
      obj_sampler.GetResampledMotionHistoryPtrById(kAvObjectId);
  const auto* agent_hist =
      obj_sampler.GetResampledMotionHistoryPtrById(kAgentId);
  const auto agent_act_net_feature =
      ExtractActNetFeatureWithAVOnly(kAgentId, *agent_hist, *av_hist);

  // Extract act net speed feature.
  const auto act_net_speed_feature = ExtractActNetSpeedFeaturesForPlannerTarget(
      agent_act_net_feature, av_discretized_path, target);
  const auto& agent_path = act_net_speed_feature.agent_path;
  const auto& av_path = act_net_speed_feature.av_path;
  EXPECT_NEAR(agent_path.path_s_preint.back(), 48.0, 1.0);
  EXPECT_NEAR(av_path.path_s_preint.back(), 49.0, 1.0);
  EXPECT_EQ(agent_path.path_s_preint.size(),
            kActNetSpeedConfig.path_point_num_pre_intersection);
  EXPECT_EQ(av_path.path_s_preint.size(),
            kActNetSpeedConfig.path_point_num_pre_intersection);

  LOG(INFO) << act_net_speed_feature.DebugString();
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
