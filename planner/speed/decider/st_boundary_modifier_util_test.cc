#include "onboard/planner/speed/decider/st_boundary_modifier_util.h"

#include <memory>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEps = 1e-3;

TEST(StBoundaryModifierUtil, AccelPointCtor) {
  constexpr double kTime = 1.0;  // s.
  constexpr double kAcc = 2.0;   // m/ss.
  const auto acc_point = AccelPoint(kTime, kAcc);

  EXPECT_NEAR(acc_point.t, kTime, kEps);
  EXPECT_NEAR(acc_point.a, kAcc, kEps);
}

TEST(StBoundaryModifierUtil, GetPathPointFromPredictedTrajectoryPoint) {
  constexpr double kS = 10.0;  // m.
  constexpr double kTheta = M_PI;
  constexpr double kKappa = 0.1;  // 1/m.
  const Vec2d pos = Vec2d(20.0, 30.0);

  prediction::PredictedTrajectoryPoint pred_traj_point;
  pred_traj_point.set_s(kS);
  pred_traj_point.set_pos(pos);
  pred_traj_point.set_theta(kTheta);
  pred_traj_point.set_kappa(kKappa);

  const auto path_point =
      GetPathPointFromPredictedTrajectoryPoint(pred_traj_point);

  EXPECT_NEAR(path_point.x(), pos.x(), kEps);
  EXPECT_NEAR(path_point.y(), pos.y(), kEps);
  EXPECT_NEAR(path_point.theta(), kTheta, kEps);
  EXPECT_NEAR(path_point.kappa(), kKappa, kEps);
  EXPECT_NEAR(path_point.s(), kS, kEps);
}

TEST(StBoundaryModifierUtil,
     SetPredictedTrajectoryPointSpatialInfoFromPathPoint) {
  constexpr double kS = 10.0;  // m.
  constexpr double kTheta = M_PI;
  constexpr double kKappa = 0.1;  // 1/m.
  const Vec2d pos = Vec2d(20.0, 30.0);

  PathPoint path_point;
  path_point.set_x(pos.x());
  path_point.set_y(pos.y());
  path_point.set_theta(kTheta);
  path_point.set_kappa(kKappa);
  path_point.set_s(kS);

  prediction::PredictedTrajectoryPoint pred_traj_point;
  SetPredictedTrajectoryPointSpatialInfoFromPathPoint(path_point,
                                                      &pred_traj_point);

  EXPECT_NEAR(pred_traj_point.pos().x(), pos.x(), kEps);
  EXPECT_NEAR(pred_traj_point.pos().y(), pos.y(), kEps);
  EXPECT_NEAR(pred_traj_point.theta(), kTheta, kEps);
  EXPECT_NEAR(pred_traj_point.kappa(), kKappa, kEps);
  EXPECT_NEAR(pred_traj_point.s(), kS, kEps);
}

TEST(StBoundaryModifierUtil, ComputeOncomingObjectDesiredAccel) {
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {{12.0, 0.0}, {10.0, 2.0}};
  st_boundary_points.upper_points = {{22.0, 0.0}, {20.0, 2.0}};
  st_boundary_points.speed_points = {{-1.0, 0.0}, {-1.0, 2.0}};
  st_boundary_points.overlap_infos = {OverlapInfo{
      .time = 0.0, .obj_idx = 0, .av_start_idx = 1, .av_end_idx = 1}};
  std::vector<StBoundaryRef> st_boundaries;
  StBoundaryRef st_boundary = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "Agent0-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  st_boundaries.push_back(std::move(st_boundary));

  constexpr double kVEgo = 10.0;  // m/s.
  const double desired_acc = ComputeOncomingObjectDesiredAccel(
      st_boundaries, kVEgo, /*yield_time_buffer=*/0.0);

  EXPECT_NEAR(desired_acc, -5.0, kEps);
}

TEST(StBoundaryModifierUtil, ComputeSafeStopDecelWithDelayedAction) {
  double s = 50.0;
  double v = 10.0;
  double a = -1.0;
  constexpr double kDelay = 1.0;

  auto res = ComputeSafeStopDecelWithDelayedAction(s, v, a, kDelay);
  EXPECT_TRUE(res.has_value());
  EXPECT_NEAR(*res, -1.0, kEps);

  a = -10.0;
  res = ComputeSafeStopDecelWithDelayedAction(s, v, a, kDelay);
  EXPECT_TRUE(res.has_value());
  EXPECT_NEAR(*res, 0.0, kEps);

  a = -1.0;
  v = -10.0;
  res = ComputeSafeStopDecelWithDelayedAction(s, v, a, kDelay);
  EXPECT_FALSE(res.has_value());

  v = 10.0;
  s = 1.0;
  res = ComputeSafeStopDecelWithDelayedAction(s, v, a, kDelay);
  EXPECT_FALSE(res.has_value());
}

TEST(StBoundaryModifierUtil, CreateSpacetimeTrajectoryByDecelAfterDelay) {
  constexpr double kObjVel = 10.0;  // m/s
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("obj1")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d(0.0, 0.0))
                                  .set_timestamp(1.0)
                                  .set_velocity(kObjVel)
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
      .set_straight_line(Vec2d(0.0, 0.0), Vec2d(100.0, 0.0),
                         /*init_v=*/kObjVel, /*last_v=*/kObjVel);

  const PlannerObject object = builder.Build();

  constexpr int kTrajIdx = 0;
  const auto& traj = object.traj(kTrajIdx);
  const auto states = SampleTrajectoryStates(
      traj, object.pose().pos(), object.contour(), object.bounding_box());
  const auto st_traj = SpacetimeObjectTrajectory(object, states, kTrajIdx,
                                                 /*required_lateral_gap=*/0.2);

  constexpr double kDelay = 1.0;
  constexpr double kAcc = -1.0;
  const auto new_st_traj =
      CreateSpacetimeTrajectoryByDecelAfterDelay(st_traj, kDelay, kAcc);
  const auto& new_states = new_st_traj.states();
  for (int i = 1; i < new_states.size(); ++i) {
    const auto& traj_point = *new_states[i].traj_point;
    const auto& last_traj_point = *new_states[i - 1].traj_point;
    if (traj_point.t() >= kDelay) {
      EXPECT_NEAR(last_traj_point.a(), kAcc, kEps);
      if (traj_point.v() > 0.0) {
        EXPECT_NEAR((traj_point.v() - last_traj_point.v()) /
                        (traj_point.t() - last_traj_point.t()),
                    kAcc, kEps);
      }
    } else {
      EXPECT_NEAR(last_traj_point.v(), kObjVel, kEps);
      EXPECT_NEAR(last_traj_point.a(), 0.0, kEps);
    }
  }
}

TEST(StBoundaryModifierUtil, CreateNudgeSpacetimeTrajectory) {
  constexpr double kObjVel = 10.0;  // m/s
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("obj1")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d(0.0, 0.0))
                                  .set_timestamp(1.0)
                                  .set_velocity(kObjVel)
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
      .set_straight_line(Vec2d(0.0, 0.0), Vec2d(100.0, 0.0),
                         /*init_v=*/kObjVel, /*last_v=*/kObjVel);

  const PlannerObject object = builder.Build();

  constexpr int kTrajIdx = 0;
  const auto& traj = object.traj(kTrajIdx);
  const auto states = SampleTrajectoryStates(
      traj, object.pose().pos(), object.contour(), object.bounding_box());
  const auto st_traj = SpacetimeObjectTrajectory(object, states, kTrajIdx,
                                                 /*required_lateral_gap=*/0.2);

  constexpr double kLambda = 0.005;  // 1/mm.
  constexpr double kMaxKappa = 0.1;  // 1/m.
  const auto new_st_traj =
      CreateNudgeSpacetimeTrajectory(st_traj, kLambda, kMaxKappa);
  const auto& new_states = new_st_traj.states();
  for (int i = 1; i < new_states.size(); ++i) {
    const auto& traj_point = *new_states[i].traj_point;
    EXPECT_LE(std::abs(traj_point.kappa()), kMaxKappa);

    const auto& last_traj_point = *new_states[i - 1].traj_point;
    if (std::abs(traj_point.kappa() - last_traj_point.kappa()) > kEps) {
      EXPECT_NEAR((traj_point.kappa() - last_traj_point.kappa()) /
                      (traj_point.s() - last_traj_point.s()),
                  kLambda, kEps);
    }
  }
}

TEST(StBoundaryModifierUtil, ModifyAndUpdateStBoundaries) {
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {{10.0, 0.0}, {10.0, 10.0}};
  st_boundary_points.upper_points = {{20.0, 0.0}, {20.0, 10.0}};
  st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, 10.0}};
  st_boundary_points.overlap_infos = {OverlapInfo{
      .time = 0.0, .obj_idx = 0, .av_start_idx = 1, .av_end_idx = 1}};
  StBoundaryRef st_boundary1 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "object1-idx0|0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  StBoundaryRef st_boundary2 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "object2-idx0",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  StBoundaryRef st_boundary3 = StBoundary::CreateInstance(
      st_boundary_points, ToStBoundaryObjectType(OT_VEHICLE), "object1-idx0|1",
      /*st_traj->trajectory().probability()=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);

  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
  st_boundaries_with_decision.emplace_back(std::move(st_boundary1));
  st_boundaries_with_decision.emplace_back(std::move(st_boundary2));
  st_boundaries_with_decision.emplace_back(std::move(st_boundary3));

  constexpr double kObjVel = 10.0;  // m/s
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("object1")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d(0.0, 0.0))
                                  .set_timestamp(1.0)
                                  .set_velocity(kObjVel)
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
      .set_straight_line(Vec2d(0.0, 0.0), Vec2d(100.0, 0.0),
                         /*init_v=*/kObjVel, /*last_v=*/kObjVel);

  const PlannerObject object = builder.Build();

  constexpr int kTrajIdx = 0;
  const auto& traj = object.traj(kTrajIdx);
  const auto states = SampleTrajectoryStates(
      traj, object.pose().pos(), object.contour(), object.bounding_box());
  auto st_traj = SpacetimeObjectTrajectory(object, states, kTrajIdx,
                                           /*required_lateral_gap=*/0.2);

  std::unordered_map<std::string, SpacetimeObjectTrajectory> input;
  input.emplace("object1-idx0|0", std::move(st_traj));

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;

  const std::function<std::optional<StBoundaryModificationResult>(
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>&,
      const StBoundaryWithDecision&)>
      test_modifier =
          [](const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
                 input,
             const StBoundaryWithDecision& st_boudary_wd)
      -> std::optional<StBoundaryModificationResult> {
    const auto* st_traj = FindOrNull(input, st_boudary_wd.id());
    if (nullptr == st_traj) {
      return std::nullopt;
    }
    std::vector<StBoundaryWithDecision> new_st_boundaries_wd;
    auto new_st_boundary =
        StBoundary::CopyInstance(*st_boudary_wd.raw_st_boundary());
    new_st_boundary->set_id(absl::StrCat(st_boudary_wd.id(), "|m"));
    new_st_boundaries_wd.push_back(
        StBoundaryWithDecision(std::move(new_st_boundary)));
    return StBoundaryModificationResult(
        {.newly_generated_st_boundaries_wd = std::move(new_st_boundaries_wd),
         .processed_st_traj = *st_traj,
         .modifier_type = StBoundaryModifierProto::PRE_DECISION});
  };
  ModifyAndUpdateStBoundaries(input, test_modifier, &processed_st_objects,
                              &st_boundaries_with_decision);

  EXPECT_TRUE(ContainsKey(processed_st_objects, "object1-idx0"));
  EXPECT_EQ(st_boundaries_with_decision.size(), 4);
  EXPECT_EQ(st_boundaries_with_decision[0].id(), "object1-idx0|0|raw");
  EXPECT_EQ(st_boundaries_with_decision[1].id(), "object2-idx0");
  EXPECT_EQ(st_boundaries_with_decision[2].id(), "object1-idx0|1|raw");
  EXPECT_EQ(st_boundaries_with_decision[3].id(), "object1-idx0|0|m");
  EXPECT_EQ(st_boundaries_with_decision[0].decision_type(),
            StBoundaryProto::IGNORE);
  EXPECT_EQ(st_boundaries_with_decision[0].decision_reason(),
            StBoundaryProto::PRE_DECIDER);
  EXPECT_EQ(st_boundaries_with_decision[0].ignore_reason(),
            StBoundaryProto::ST_BOUNDARY_MODIFIED);
  EXPECT_EQ(st_boundaries_with_decision[2].decision_type(),
            StBoundaryProto::IGNORE);
  EXPECT_EQ(st_boundaries_with_decision[2].decision_reason(),
            StBoundaryProto::ST_BOUNDARY_MODIFIER);
  EXPECT_EQ(st_boundaries_with_decision[2].ignore_reason(),
            StBoundaryProto::ST_BOUNDARY_MODIFIED);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
