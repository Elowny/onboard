#include "onboard/planner/speed/decider/interaction_util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

namespace {

const VehicleGeometryParamsProto kVehGeoParams =
    planner::DefaultVehicleGeometry();

constexpr int kSpeedSize = 60;
constexpr int kTrajIdx = 0;
constexpr double kObjVel = 10.0;  // m/s
constexpr double kEps = 1e-3;

TEST(InteractionUtilTest, IsAvInObjectFov) {
  PathPoint current_path_point;
  current_path_point.set_x(1.0);
  current_path_point.set_y(0.0);
  current_path_point.set_z(0.0);
  current_path_point.set_theta(0.0);
  current_path_point.set_kappa(0.0);
  current_path_point.set_s(1.0);

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("Agent0")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(0.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.5, 2.0)
                                  .set_pos(Vec2d(170.6, 66.6))
                                  .set_box_center(Vec2d(170.6, 66.6))
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

  const PlannerObject planner_object = builder.Build();
  const bool result =
      IsAvInObjectFov(current_path_point, planner_object, kVehGeoParams);
  EXPECT_FALSE(result);

  PerceptionObjectBuilder perception_builder_2;
  const auto perception_obj_2 = perception_builder_2.set_id("Agent0")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(0.0)
                                    .set_yaw(-2.0)
                                    .set_length_width(4.5, 2.0)
                                    .set_pos(Vec2d(170.6, 66.6))
                                    .set_box_center(Vec2d(170.6, 66.6))
                                    .Build();

  PlannerObjectBuilder builder_2;
  builder_2.set_type(OT_VEHICLE)
      .set_object(perception_obj_2)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2dFromProto(perception_obj_2.pos()),
                            perception_obj_2.yaw())
      .set_probability(0.5);

  const PlannerObject planner_object_2 = builder_2.Build();
  const bool result_2 =
      IsAvInObjectFov(current_path_point, planner_object_2, kVehGeoParams);
  EXPECT_TRUE(result_2);
}

TEST(InteractionUtilTest, IsAvCompletelyInObjectFov) {
  PathPoint current_path_point;
  current_path_point.set_x(1.0);
  current_path_point.set_y(0.0);
  current_path_point.set_z(0.0);
  current_path_point.set_theta(0.0);
  current_path_point.set_kappa(0.0);
  current_path_point.set_s(1.0);

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("Agent0")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(0.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.5, 2.0)
                                  .set_pos(Vec2d(170.6, 66.6))
                                  .set_box_center(Vec2d(170.6, 66.6))
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

  const PlannerObject planner_object = builder.Build();
  const bool result = IsAvCompletelyInObjectFov(current_path_point,
                                                planner_object, kVehGeoParams);
  EXPECT_FALSE(result);

  PerceptionObjectBuilder perception_builder_3;
  const auto perception_obj_3 = perception_builder.set_id("Agent0")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(0.0)
                                    .set_yaw(-2.0)
                                    .set_length_width(4.5, 2.0)
                                    .set_pos(Vec2d(170.6, 66.6))
                                    .set_box_center(Vec2d(170.6, 66.6))
                                    .Build();

  PlannerObjectBuilder builder_3;
  builder_3.set_type(OT_VEHICLE)
      .set_object(perception_obj_3)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2dFromProto(perception_obj_3.pos()),
                            perception_obj_3.yaw())
      .set_probability(0.5);

  const PlannerObject planner_object_3 = builder_3.Build();
  const bool result_3 = IsAvCompletelyInObjectFov(
      current_path_point, planner_object_3, kVehGeoParams);
  EXPECT_TRUE(result_3);
}

TEST(InteractionUtilTest, IsObjectInAvFov) {
  PathPoint current_path_point;
  current_path_point.set_x(1.0);
  current_path_point.set_y(0.0);
  current_path_point.set_z(0.0);
  current_path_point.set_theta(0.0);
  current_path_point.set_kappa(0.0);
  current_path_point.set_s(1.0);

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("Agent0")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(0.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.5, 2.0)
                                  .set_pos(Vec2d(170.6, 66.6))
                                  .set_box_center(Vec2d(170.6, 66.6))
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

  const PlannerObject planner_object = builder.Build();
  const bool result = IsObjectInAvFov(current_path_point, planner_object);
  EXPECT_TRUE(result);

  current_path_point.set_theta(5.0);
  const bool result_4 = IsObjectInAvFov(current_path_point, planner_object);
  EXPECT_FALSE(result_4);
}

TEST(InteractionUtilTest, GetAvOverlapTtcInfo) {
  std::vector<OverlapInfo> overlap_infos;

  overlap_infos.push_back(OverlapInfo{.av_start_idx = 1, .av_end_idx = 2});
  StOverlapMetaProto overlap_meta;
  overlap_meta.set_priority(StOverlapMetaProto::HIGH);

  PathPoint p0;
  p0.set_x(0.0);
  p0.set_y(0.0);
  p0.set_z(0.0);
  p0.set_theta(0.0);
  p0.set_kappa(0.0);
  p0.set_lambda(0.0);
  p0.set_s(0.0);

  PathPoint p1;
  p1.set_x(1.0);
  p1.set_y(0.0);
  p1.set_z(0.0);
  p1.set_theta(0.0);
  p1.set_kappa(0.0);
  p0.set_lambda(0.0);
  p1.set_s(1.0);

  PathPoint p2;
  p2.set_x(2.0);
  p2.set_y(0.0);
  p2.set_z(0.0);
  p2.set_theta(0.0);
  p2.set_kappa(0.0);
  p0.set_lambda(0.0);
  p2.set_s(2.0);

  std::vector<PathPoint> path_points = {p0, p1, p2};

  const DiscretizedPath path(std::move(path_points));

  std::vector<SpeedPoint> speed_points;
  speed_points.reserve(kSpeedSize);
  for (int j = 0; j < kSpeedSize; ++j) {
    SpeedPoint temp;
    temp.set_s(0.15 * static_cast<double>(j));
    temp.set_t(static_cast<double>(j));
    temp.set_v(0.1);
    temp.set_a(0.1);
    temp.set_j(0.1);
    speed_points.push_back(std::move(temp));
  }

  const SpeedVector speed_data(std::move(speed_points));

  const TtcInfo ttc_info =
      GetAvOverlapTtcInfo(overlap_infos, overlap_meta, path, speed_data);
  EXPECT_NEAR(ttc_info.ttc_lower_limit, 1.538, kEps);
  EXPECT_NEAR(ttc_info.ttc_upper_limit, 1000.0, kEps);
}

TEST(InteractionUtilTest, GetObjectOverlapTtcInfo) {
  const auto fo_info = OverlapInfo{.obj_idx = 1};

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

  const auto& traj = object.traj(kTrajIdx);
  const auto states = SampleTrajectoryStates(
      traj, object.pose().pos(), object.contour(), object.bounding_box());
  const auto spacetime_obj =
      SpacetimeObjectTrajectory(object, states, kTrajIdx,
                                /*required_lateral_gap=*/0.2);

  const TtcInfo ttc_info = GetObjectOverlapTtcInfo(fo_info, spacetime_obj);
  EXPECT_NEAR(ttc_info.ttc_lower_limit, 0.086, kEps);
  EXPECT_NEAR(ttc_info.ttc_upper_limit, 0.095, kEps);
}

TEST(InteractionUtilTest, CalcObjectYieldingTime) {
  std::vector<OverlapInfo> overlap_infos;
  overlap_infos.push_back(OverlapInfo{.av_start_idx = 1, .av_end_idx = 2});
  StOverlapMetaProto overlap_meta;
  overlap_meta.set_priority(StOverlapMetaProto::HIGH);

  PathPoint p0;
  p0.set_x(0.0);
  p0.set_y(0.0);
  p0.set_z(0.0);
  p0.set_theta(0.0);
  p0.set_kappa(0.0);
  p0.set_lambda(0.0);
  p0.set_s(0.0);

  PathPoint p1;
  p1.set_x(1.0);
  p1.set_y(0.0);
  p1.set_z(0.0);
  p1.set_theta(0.0);
  p1.set_kappa(0.0);
  p0.set_lambda(0.0);
  p1.set_s(1.0);

  PathPoint p2;
  p2.set_x(2.0);
  p2.set_y(0.0);
  p2.set_z(0.0);
  p2.set_theta(0.0);
  p2.set_kappa(0.0);
  p0.set_lambda(0.0);
  p2.set_s(2.0);

  std::vector<PathPoint> path_points = {p0, p1, p2};

  const DiscretizedPath path(std::move(path_points));

  std::vector<SpeedPoint> speed_points;
  speed_points.reserve(kSpeedSize);
  for (int j = 0; j < kSpeedSize; ++j) {
    SpeedPoint temp;
    temp.set_s(0.15 * static_cast<double>(j));
    temp.set_t(static_cast<double>(j));
    temp.set_v(0.1);
    temp.set_a(0.1);
    temp.set_j(0.1);
    speed_points.push_back(std::move(temp));
  }

  const SpeedVector speed_data(std::move(speed_points));

  const double t =
      CalcObjectYieldingTime(overlap_infos, overlap_meta, path, speed_data);
  EXPECT_NEAR(t, 6.666667, kEps);
}

TEST(InteractionUtilTest, IsParallelMerging) {
  StOverlapMetaProto overlap_meta;
  overlap_meta.set_source(StOverlapMetaProto::LANE_MERGE);
  overlap_meta.set_theta_diff(0.01);
  const bool result = IsParallelMerging(overlap_meta);
  EXPECT_TRUE(result);

  StOverlapMetaProto overlap_meta_5;
  overlap_meta_5.set_source(StOverlapMetaProto::AV_CUTIN);
  overlap_meta_5.set_theta_diff(0.01);
  const bool result_5 = IsParallelMerging(overlap_meta_5);
  EXPECT_FALSE(result_5);
}

TEST(InteractionUtilTest, HasYieldingIntentionToFrontAv) {
  PathPoint current_path_point;
  current_path_point.set_x(1.0);
  current_path_point.set_y(0.0);
  current_path_point.set_z(0.0);
  current_path_point.set_theta(0.0);
  current_path_point.set_kappa(0.0);
  current_path_point.set_s(1.0);

  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("Agent0")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(0.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.5, 2.0)
                                  .set_pos(Vec2d(170.6, 66.6))
                                  .set_box_center(Vec2d(170.6, 66.6))
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

  const PlannerObject planner_object = builder.Build();

  StOverlapMetaProto overlap_meta;
  overlap_meta.set_priority(StOverlapMetaProto::HIGH);

  std::vector<SpeedPoint> speed_points;
  speed_points.reserve(kSpeedSize);
  for (int j = 0; j < kSpeedSize; ++j) {
    SpeedPoint temp;
    temp.set_s(0.15 * static_cast<double>(j));
    temp.set_t(static_cast<double>(j));
    temp.set_v(0.1);
    temp.set_a(0.1);
    temp.set_j(0.1);
    speed_points.push_back(std::move(temp));
  }

  const SpeedVector preliminary_speed(std::move(speed_points));

  const bool result = HasYieldingIntentionToFrontAv(
      current_path_point, planner_object, kVehGeoParams, overlap_meta,
      preliminary_speed, /*first_overlap_time=*/0.5);

  EXPECT_FALSE(result);

  PerceptionObjectBuilder perception_builder_6;
  const auto perception_obj_6 = perception_builder.set_id("Agent0")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(0.0)
                                    .set_yaw(-2.0)
                                    .set_length_width(4.5, 2.0)
                                    .set_pos(Vec2d(170.6, 66.6))
                                    .set_box_center(Vec2d(170.6, 66.6))
                                    .Build();

  PlannerObjectBuilder builder_6;
  builder_6.set_type(OT_VEHICLE)
      .set_object(perception_obj_6)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_stationary_traj(Vec2dFromProto(perception_obj_6.pos()),
                            perception_obj_6.yaw())
      .set_probability(0.5);

  const PlannerObject planner_object_6 = builder_6.Build();
  const bool result_6 = HasYieldingIntentionToFrontAv(
      current_path_point, planner_object_6, kVehGeoParams, overlap_meta,
      preliminary_speed, /*first_overlap_time=*/0.5);

  EXPECT_TRUE(result_6);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
