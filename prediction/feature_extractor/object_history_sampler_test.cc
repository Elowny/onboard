#include "onboard/prediction/feature_extractor/object_history_sampler.h"

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move
#include <memory>
#include <ostream>
#include <string>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
TEST(ObjectsHistorySamplerTest, LerpObjectMotionState) {
  const auto obj_state_a = ObjectMotionState{
      .timestamp = 10.0,
      .pos = Vec2d(10.0, 0.0),
      .heading = 0.0,
      .vel = Vec2d(5.0, 0.0),
      .bbox = Box2d(Vec2d(10.0, 0.0), 0.0, 4.0, 2.0),
  };
  const auto obj_state_b = ObjectMotionState{
      .timestamp = 20.0,
      .pos = Vec2d(60.0, 0.0),
      .heading = 0.0,
      .vel = Vec2d(5.0, 0.0),
      .bbox = Box2d(Vec2d(60.0, 0.0), 0.0, 4.0, 2.0),
  };
  const auto obj_state_m = LerpObjectMotionState(obj_state_a, obj_state_b, 0.5);
  LOG(INFO) << "lerped motion state:" << obj_state_m.DebugString();
  EXPECT_EQ(obj_state_m.timestamp, 15.0);
  EXPECT_EQ(obj_state_m.pos, Vec2d(35.0, 0.0));
  EXPECT_EQ(obj_state_m.vel, Vec2d(5.0, 0.0));
  const auto obj_state_2 = LerpObjectMotionState(obj_state_a, obj_state_b, 2.0);
  EXPECT_EQ(obj_state_2.timestamp, 30.0);
  EXPECT_EQ(obj_state_2.pos, Vec2d(110.0, 0.0));
  EXPECT_EQ(obj_state_2.vel, Vec2d(5.0, 0.0));
}

TEST(ObjectsHistorySamplerTest, ResampleObjectHistorySpanToMotionHistory) {
  Vec2d pos(60.0, 80.0);
  const Vec2d vel(10.0, 10.0);
  std::string id("1");
  auto obj_a = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_type(OT_VEHICLE)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(0.0)
                   .set_box_center(pos)
                   .set_length_width(/*length=*/2.0, /*width=*/1.0)
                   .Build();
  pos = pos + vel;
  auto obj_b = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_type(OT_VEHICLE)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(1.0)
                   .set_box_center(pos)
                   .set_length_width(/*length=*/2.0, /*width=*/1.0)
                   .Build();
  pos = pos + vel;
  auto obj_c = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_type(OT_VEHICLE)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(2.0)
                   .set_box_center(pos)
                   .set_length_width(/*length=*/2.0, /*width=*/1.0)
                   .Build();
  ObjectHistory hist(20);
  CoordinateConverter target;
  hist.Push(0.0, PredictionObject(obj_a, target));
  hist.Push(1.0, PredictionObject(obj_b, target));
  hist.Push(2.0, PredictionObject(obj_c, target));
  const auto obj_motion_history = ResampleObjectHistorySpanToMotionHistory(
      hist.GetHistory(), /*current_ts=*/2.0, /*time_step=*/0.1,
      /*max_steps=*/21, /*enable_smoothing=*/false);
  EXPECT_EQ(obj_motion_history.id, id);
  EXPECT_EQ(obj_motion_history.states.size(), 21);
  EXPECT_NEAR(obj_motion_history.states.front().timestamp, 0.0, kEps);
  EXPECT_NEAR(obj_motion_history.states.back().timestamp, 2.0, kEps);
  EXPECT_NEAR(obj_motion_history.states[1].timestamp, 0.1, kEps);
  EXPECT_NEAR(obj_motion_history.states[1].pos.x(), 61.0, kEps);
  EXPECT_NEAR(obj_motion_history.states[1].pos.y(), 81.0, kEps);
}

TEST(ObjectsHistorySamplerTest, ResampleObjectHistorySpanNotEnough) {
  Vec2d pos(60.0, 80.0);
  const Vec2d vel(10.0, 10.0);
  std::string id("1");
  auto obj_a = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_type(OT_VEHICLE)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(0.5)
                   .set_box_center(pos)
                   .set_length_width(/*length=*/2.0, /*width=*/1.0)
                   .Build();
  pos = pos + 0.5 * vel;
  auto obj_b = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_type(OT_VEHICLE)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(1.0)
                   .set_box_center(pos)
                   .set_length_width(/*length=*/2.0, /*width=*/1.0)
                   .Build();
  pos = pos + vel;
  auto obj_c = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_type(OT_VEHICLE)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(2.0)
                   .set_box_center(pos)
                   .set_length_width(/*length=*/2.0, /*width=*/1.0)
                   .Build();
  ObjectHistory hist(20);
  CoordinateConverter target;
  hist.Push(0.5, PredictionObject(obj_a, target));
  hist.Push(1.0, PredictionObject(obj_b, target));
  hist.Push(2.0, PredictionObject(obj_c, target));
  const auto resampled_objs = ResampleObjectHistorySpan(
      hist.GetHistory(), /*current_ts=*/2.0, /*time_step=*/0.1,
      /*max_steps=*/21, /*enable_smoothing=*/true);
  EXPECT_EQ(resampled_objs.size(), 16);
  EXPECT_NEAR(resampled_objs.front().timestamp(), 0.5, kEps);
  EXPECT_NEAR(resampled_objs.back().timestamp(), 2.0, kEps);
  EXPECT_NEAR(resampled_objs[1].timestamp(), 0.6, kEps);
  EXPECT_NEAR(resampled_objs[1].pos().x(), 61.0, kEps);
  EXPECT_NEAR(resampled_objs[1].pos().y(), 81.0, kEps);
}

TEST(ObjectsHistorySamplerTest, ResampleObjectHistorySpanStepCount1) {
  Vec2d pos(60.0, 80.0);
  const Vec2d vel(10.0, 10.0);
  std::string id("1");
  auto obj_a = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(0.55)

                   .Build();
  pos = pos + 0.45 * vel;
  auto obj_b = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(1.0)
                   .Build();
  pos = pos + vel;
  auto obj_c = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(2.0)
                   .Build();
  ObjectHistory hist(20);
  CoordinateConverter target;
  hist.Push(0.55, PredictionObject(obj_a, target));
  hist.Push(1.0, PredictionObject(obj_b, target));
  hist.Push(2.0, PredictionObject(obj_c, target));
  const auto resampled_objs = ResampleObjectHistorySpan(
      hist.GetHistory(), /*current_ts=*/2.0, /*time_step=*/0.1,
      /*max_steps=*/21, /*enable_smoothing=*/false);
  EXPECT_EQ(resampled_objs.size(), 15);
}

TEST(ObjectsHistorySamplerTest, ResampleObjectHistorySpanStepCount2) {
  Vec2d pos(60.0, 80.0);
  const Vec2d vel(10.0, 10.0);
  std::string id("1");
  auto obj_a = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(0.5)
                   .Build();
  pos = pos + 0.5 * vel;
  auto obj_b = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(1.0)
                   .Build();
  pos = pos + vel;
  auto obj_c = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(2.0)
                   .Build();
  ObjectHistory hist(20);
  CoordinateConverter target;
  hist.Push(0.5, PredictionObject(obj_a, target));
  hist.Push(1.0, PredictionObject(obj_b, target));
  hist.Push(2.0, PredictionObject(obj_c, target));
  const auto resampled_objs = ResampleObjectHistorySpan(
      hist.GetHistory(), /*current_ts=*/2.0, /*time_step=*/0.1,
      /*max_steps=*/21, /*enable_smoothing=*/false);
  EXPECT_EQ(resampled_objs.size(), 16);
}

TEST(ObjectsHistorySamplerTest, ResampleObjectHistorySpanToMotionHistoryEmpty) {
  const auto obj = planner::PerceptionObjectBuilder().Build();
  const auto resample_or =
      ResampleObjectMotionHistoryFromTrackerHistory(obj, 1.0, 0.1, 10);
  EXPECT_FALSE(resample_or.ok());
}
TEST(ObjectsHistorySamplerTest,
     ResampleObjectMotionHistoryFromTrackerHistoryOne) {
  const Vec2d pos(60.0, 80.0);
  const Vec2d vel(10.0, 10.0);
  const std::string id("1");
  const auto obj_a = planner::PerceptionObjectBuilder()
                         .set_id(id)
                         .set_pos(pos)
                         .set_yaw(vel.Angle())
                         .set_velocity(vel.norm())
                         .set_speed(vel)
                         .set_timestamp(100.0)
                         .set_trajectory(1)
                         .Build();
  const auto resample_or =
      ResampleObjectMotionHistoryFromTrackerHistory(obj_a, 100.0, 0.1, 10);
  EXPECT_TRUE(resample_or.ok());
  EXPECT_EQ(resample_or->states.size(), 1);
}

TEST(ObjectsHistorySamplerTest, CheckHistoryConsistency) {
  Vec2d pos(60.0, 80.0);
  const Vec2d vel(10.0, 10.0);
  const std::string id("1");
  auto obj_a = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(0.5)
                   .Build();
  pos = pos + 0.5 * vel;
  auto obj_b = planner::PerceptionObjectBuilder()
                   .set_id(id)
                   .set_pos(pos)
                   .set_yaw(vel.Angle())
                   .set_velocity(vel.norm())
                   .set_speed(vel)
                   .set_timestamp(1.0)
                   .Build();
  EXPECT_TRUE(CheckHistoryConsistency(obj_a, obj_b, 0.5));
}

TEST(ObjectsHistorySamplerTest, ObjectHistorySamplerSpan) {
  const auto av_history =
      BuildVehicleHistoryByConstVel(kAvObjectId, 10, Vec2d::Zero(), 5.0);
  const auto obj_history =
      BuildVehicleHistoryByConstVel("1", 10, Vec2d(40.0, 0.0), 6.0);

  const double current_ts = obj_history.timestamp();
  const ObjectHistorySampler obj_sampler({&av_history, &obj_history},
                                         av_history, current_ts, 0.2, 10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  EXPECT_EQ(obj_sampler.current_time(), current_ts);
  EXPECT_EQ(obj_sampler.time_step(), 0.2);
  EXPECT_EQ(obj_sampler.max_steps(), 10);
  EXPECT_EQ(obj_sampler.objects_size(), 2);
  EXPECT_EQ(obj_sampler.GetResampledMotionHistoryById("1").id, "1");
  EXPECT_EQ(obj_sampler.GetResampledAVMotionHistory()->id, kAvObjectId);
  EXPECT_EQ(obj_sampler.GetResampledMotionHistoryPtrById("2"), nullptr);
  const auto empty_objs =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d("1", Box2d(), 0);
  EXPECT_TRUE(empty_objs.empty());
  const auto one_obj_motion_history =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d("1", Box2d(), 1);

  EXPECT_EQ(one_obj_motion_history.front()->id, kAvObjectId);
  const auto two_obj_motion_history =
      obj_sampler.GetResampledMotionHistoryWithAVInBox2d("1", Box2d(), 2);
  EXPECT_EQ(two_obj_motion_history.size(), 1);
  const ObjectHistorySampler obj_sampler_tracker_smooth(
      {&av_history, &obj_history}, av_history, current_ts, 0.2, 10,
      /*enable_smoothing=*/true,
      /*use_tracker_history=*/true);
  EXPECT_EQ(obj_sampler_tracker_smooth.objects_size(), 2);

  const ObjectHistorySampler obj_sampler_no_tracker_no_smooth(
      {&av_history, &obj_history}, av_history, current_ts, 0.2, 10,
      /*enable_smoothing=*/false,
      /*use_tracker_history=*/false);
  EXPECT_EQ(obj_sampler_no_tracker_no_smooth.objects_size(), 2);
  const ObjectHistorySampler obj_sampler_no_tracker_smooth(
      {&av_history, &obj_history}, av_history, current_ts, 0.2, 10,
      /*enable_smoothing=*/true,
      /*use_tracker_history=*/false);
  EXPECT_EQ(obj_sampler_no_tracker_smooth.objects_size(), 2);
}

}  // namespace
}  // namespace qcraft::prediction
