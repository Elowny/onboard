#include "onboard/prediction/predictor/bicycle_lane_follow_predictor.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
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
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {
namespace {
const double kTimeStep = 0.1;
const int kHistoryNum = 10;
TEST(BicycleLaneFollowPredictorTest, OneTrajTest) {
  FLAGS_prediction_use_tracker_history = false;
  const auto& psmm = planner::CreateDojoTestPSMM();
  FLAGS_prediction_use_tracker_history = false;
  const Vec2d pos(-6.0, 34.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_CYCLIST)
                 .set_pos(pos)
                 .set_velocity(5.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(-M_PI * 0.5)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
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
  const auto trajs = MakeBicycleLaneFollowPrediction(
      obj_sampler.GetResampledMotionHistoryById(id), context,
      /*ignore_off_road=*/false);
  EXPECT_EQ(trajs.size(), 1);
}

TEST(BicycleLaneFollowPredictorTest, TwoTrajsTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(-6.0, 34.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_CYCLIST)
                 .set_pos(pos)
                 .set_velocity(5.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(-M_PI * 0.5 + 3.5 * M_PI / 12.0)
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
  const auto trajs = MakeBicycleLaneFollowPrediction(
      obj_sampler.GetResampledMotionHistoryById(id), context,
      /*ignore_off_road=*/false);
  EXPECT_EQ(trajs.size(), 2);
}

}  // namespace
}  // namespace qcraft::prediction
