#include "onboard/prediction/predictor/act_net_j5_predictor.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <algorithm>  // for max
#include <memory>     // for make_unique, unique_ptr
#include <string>     // for string
#include <utility>    // for move
#include <vector>     // for vector, allocator

#include "gtest/gtest.h"  // for Test, TestInfo, EXPECT_EQ, Message, TEST

#include "onboard/lite/check.h"    // for QCHECK_OK
#include "onboard/lite/logging.h"  // for BufferedLoggerWrapper
#include "onboard/math/vec.h"
#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/params/param_finder.h"      // for GetProtoParamById
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/av_context.h"       // for AvContext
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/act_net_j5_inferencer.h"  // for ActNeJ5tInferencer
#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"  // for BuildOneObjectPredictionContext
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
namespace {
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr double kObject1InitPosY = 2.0;
constexpr double kObject1Vel = 1.0;
constexpr char kObject1Id[] = "999";
constexpr char kEgoId[] = "9999";

constexpr int kNumHistory = 10;
constexpr double kUpdateTimeStep = 0.1;

TEST(ActNetJ5PredictorTest, CutinTest) {
  FLAGS_print_prediction_time_stats = true;
  FLAGS_prediction_enable_auxiliary_cutin_sl_net_j5 = true;

  NetParam act_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "act_net_j5_param", &act_net_j5_param));

  NetParam cutin_sl_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "cutin_sl_net_j5_param",
                              &cutin_sl_net_j5_param));
  cutin_sl_net_j5::CutinNetJ5Inferencer cutin_sl_net_j5_inferencer(
      cutin_sl_net_j5_param);
  actnetj5::ActNetJ5Inferencer act_net_j5_inferencer(act_net_j5_param);

  const auto& psmm = planner::CreateDojoTestPSMM();

  auto agent_history = BuildVehicleHistoryByConstVel(
      kEgoId, kNumHistory, Vec2d(kEgoInitPosX, kObject1InitPosY), kEgoVel);
  auto obj_history = BuildVehicleHistoryByConstVel(
      kObject1Id, kNumHistory, Vec2d(0.0, kObject1InitPosY), kObject1Vel);
  auto av_history = BuildVehicleHistoryByConstVel(kAvObjectId, kNumHistory,
                                                  Vec2d::Zero(), kEgoVel);
  FLAGS_prediction_use_tracker_history = false;
  const auto obj_sampler = ObjectHistorySampler(
      {&agent_history, &obj_history}, av_history,
      kUpdateTimeStep * (kNumHistory - 1), kUpdateTimeStep, kNumHistory,
      /*enable_smoothing=*/false, FLAGS_prediction_use_tracker_history);

  std::vector<const ObjectHistory*> objects_history = {&agent_history,
                                                       &obj_history};
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  PredictionContext context(input);
  ObjectsProto objects_proto;
  objects_proto.set_scope(ObjectsProto::SCOPE_REAL);
  objects_proto.mutable_objects()->Add(
      ObjectProto(agent_history.object_proto()));
  objects_proto.mutable_objects()->Add(ObjectProto(obj_history.object_proto()));
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  MapSampler map_sampler(
      psmm, context.traffic_light_manager().GetOriginalTlStateMap(),
      kFeatureV2MaxMapSampleLen, kFeatureV2MapSegmentNum,
      MapSampler::SampleType::ADAPTIVE);
  auto threadpool = std::make_unique<ThreadPool>(0);

  const auto pred_results = MakeActNetJ5Prediction(
      context, objects_history, act_net_j5_inferencer,
      &cutin_sl_net_j5_inferencer, obj_sampler, threadpool.get());
  EXPECT_EQ(pred_results.size(), 2);
}

TEST(ActNetJ5PredictorTest, NormalTest) {
  FLAGS_prediction_enable_auxiliary_cutin_sl_net_j5 = true;
  NetParam act_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "act_net_j5_param", &act_net_j5_param));
  actnetj5::ActNetJ5Inferencer act_net_j5_inferencer(act_net_j5_param);

  NetParam cutin_sl_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "cutin_sl_net_j5_param",
                              &cutin_sl_net_j5_param));
  cutin_sl_net_j5::CutinNetJ5Inferencer cutin_sl_net_j5_inferencer(
      cutin_sl_net_j5_param);

  const auto& psmm = planner::CreateDojoTestPSMM();

  std::string id("1");
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto obj =
      planner::PerceptionObjectBuilder().set_id(id).set_trajectory(10).Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  input.av_context->Update(*input.pose, *input.localization_transform,
                           *input.veh_geom_params);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);
  const double current_ts = hist.timestamp();

  std::vector<const ObjectHistory*> objects_history{&hist};
  const ObjectHistorySampler obj_sampler({&hist}, hist, current_ts, 0.1, 10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);

  auto threadpool = std::make_unique<ThreadPool>(0);

  const auto context = BuildOneObjectPredictionContext(id, &input);
  const auto pred_results = MakeActNetJ5Prediction(
      context, objects_history, act_net_j5_inferencer,
      &cutin_sl_net_j5_inferencer, obj_sampler, threadpool.get());
  EXPECT_EQ(pred_results.size(), 1);
}

TEST(ActNetJ5PredictorTest, MultiObjectsTest) {
  FLAGS_prediction_enable_auxiliary_cutin_sl_net_j5 = true;
  NetParam act_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "act_net_j5_param", &act_net_j5_param));

  NetParam cutin_sl_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "cutin_sl_net_j5_param",
                              &cutin_sl_net_j5_param));
  cutin_sl_net_j5::CutinNetJ5Inferencer cutin_sl_net_j5_inferencer(
      cutin_sl_net_j5_param);

  actnetj5::ActNetJ5Inferencer act_net_j5_inferencer(act_net_j5_param);
  const int max_bs = act_net_j5_param.max_batch_size();
  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(max_bs, &input);
  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(max_bs);
  for (int i = 0; i < max_bs; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }
  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  auto threadpool = std::make_unique<ThreadPool>(0);
  // Rebuild context to include av drive passage, need to modify later
  context = BuildMultiObjectsPredictionContext(max_bs, &input);
  const auto pred_results = MakeActNetJ5Prediction(
      context, objects_history, act_net_j5_inferencer,
      &cutin_sl_net_j5_inferencer, obj_sampler, threadpool.get());
  EXPECT_EQ(pred_results.size(), max_bs);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
