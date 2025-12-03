#include "onboard/prediction/predictor/l2_lane_follow_predictor.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/vehicle.pb.h"

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

namespace qcraft::prediction {
namespace {
const double kTimeStep = 0.1;
const int kHistoryNum = 10;
TEST(L2LaneFollowPredictorTest, OneTrajTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(-42, 3.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  PredictionContext context(input);
  vantage_client_man::CreateVantageClientMan();
  int i = 0;
  for (const auto& dp : context.drive_passages()) {
    SendDrivePassageLineToCanvas(*dp, "dp_" + std::to_string(i));
    i++;
  }
  const auto traj = MakeAssitDrivingLaneFollowPrediction(
      obj_sampler.GetResampledMotionHistoryById(id), context);
  LOG(INFO) << traj.annotation();
  EXPECT_EQ(traj.type(), PredictionType::PT_L2_LANE_FOLLOW);
}
TEST(L2LaneFollowPredictorTest, OneTrajTest2) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(-92, 3.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  PredictionContext context(input);
  vantage_client_man::CreateVantageClientMan();
  int i = 0;
  for (const auto& dp : context.drive_passages()) {
    SendDrivePassageLineToCanvas(*dp, "dp_" + std::to_string(i));
    i++;
  }
  const auto traj = MakeAssitDrivingLaneFollowPrediction(
      obj_sampler.GetResampledMotionHistoryById(id), context);
  LOG(INFO) << traj.annotation();
  EXPECT_EQ(traj.type(), PredictionType::PT_VOID);
}

TEST(L2LaneFollowPredictorTest, OneTrajReverseTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(-40, 3.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(M_PI)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  PredictionContext context(input);
  vantage_client_man::CreateVantageClientMan();
  int i = 0;
  for (const auto& dp : context.drive_passages()) {
    SendDrivePassageLineToCanvas(*dp, "dp_" + std::to_string(i));
    i++;
  }
  const auto traj = MakeAssitDrivingLaneFollowPrediction(
      obj_sampler.GetResampledMotionHistoryById(id), context);
  LOG(INFO) << traj.annotation();
  EXPECT_TRUE(traj.annotation().find("reversed") != std::string::npos);
  EXPECT_EQ(traj.type(), PredictionType::PT_L2_LANE_FOLLOW);
}

TEST(L2LaneFollowPredictorTest, OneTrajCTRATest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d pos(24, 3.2);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(-M_PI_2)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  FLAGS_prediction_use_tracker_history = false;
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  PredictionContext context(input);
  vantage_client_man::CreateVantageClientMan();
  int i = 0;
  for (const auto& dp : context.drive_passages()) {
    SendDrivePassageLineToCanvas(*dp, "dp_" + std::to_string(i));
    i++;
  }
  const auto traj = MakeAssitDrivingLaneFollowPrediction(
      obj_sampler.GetResampledMotionHistoryById(id), context);
  LOG(INFO) << traj.annotation();
  EXPECT_EQ(traj.type(), PredictionType::PT_CTRA);
}

}  // namespace
}  // namespace qcraft::prediction
