#include "onboard/prediction/predictor/cutin_sl_net_predictor.h"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/vec.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {
namespace {
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr double kObject1InitPosY = 2.0;
constexpr double kObject1Vel = 1.0;
constexpr char kObject1Id[] = "999";
constexpr char kEgoId[] = "9999";

constexpr int kNumHistory = 10;
constexpr double kUpdateTimeStep = 0.1;

TEST(CutinSLPredictorTest, NormalTest) {
  NetParam cutin_sl_net_param;
  QCHECK_OK(
      GetProtoParamById("Q0001", "cutin_sl_net_param", &cutin_sl_net_param));
  cutin_sl_net_param.set_device_type(NetParam_DeviceType_CPU);
  cutin_sl_net::CutinSLNetInferencer cutin_sl_net_inferencer(
      cutin_sl_net_param);

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

  std::vector<const ObjectHistory*> objects_history = {
      &agent_history, &obj_history, &av_history};
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  std::string object_id("1");
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  const auto context = BuildOneObjectPredictionContext(object_id, &input);
  MapSampler map_sampler(
      psmm, context.traffic_light_manager().GetOriginalTlStateMap(),
      kFeatureV2MaxMapSampleLen, kFeatureV2MapSegmentNum,
      MapSampler::SampleType::ADAPTIVE);
  auto threadpool = std::make_unique<ThreadPool>(2);

  const auto pred_results_opt = MakeCutinSLNetPrediction(
      context, objects_history, cutin_sl_net_inferencer, obj_sampler,
      &map_sampler, threadpool.get());
  EXPECT_NE(pred_results_opt, std::nullopt);
  EXPECT_EQ(pred_results_opt->size(), 2);
}
}  // namespace
}  // namespace qcraft::prediction
