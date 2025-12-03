#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"

#include <string>  // for basic_string, string

#include "gtest/gtest.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/vec.h"  // for Vec2d
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/traffic_light_manager.h"  // for TrafficLightManager
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/prediction_flags.h"  // for FLAGS_prediction_use_tracker_history
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
namespace {
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr double kObject1InitPosY = 2.0;
constexpr double kObject1Vel = 1.0;
constexpr char kObject1Id[] = "999";
constexpr char kEgoId1[] = "9996";
constexpr char kEgoId2[] = "9997";
constexpr char kEgoId3[] = "9998";
constexpr int kNumHistory = 10;
constexpr double kUpdateTimeStep = 0.1;

TEST(CutinSLNetInferencerTest, PredictForObjectsOnCPU) {
  NetParam cutin_sl_net_param;
  QCHECK_OK(
      GetProtoParamById("Q0001", "cutin_sl_net_j5_param", &cutin_sl_net_param));
  cutin_sl_net_j5::CutinNetJ5Inferencer cutin_sl_net_inferencer(
      cutin_sl_net_param);

  const auto& psmm = planner::CreateDojoTestPSMM();

  auto agent_history_1 = BuildVehicleHistoryByConstVel(
      kEgoId1, kNumHistory, Vec2d(kEgoInitPosX, kObject1InitPosY), kEgoVel);
  auto agent_history_2 = BuildVehicleHistoryByConstVel(
      kEgoId2, kNumHistory, Vec2d(kEgoInitPosX, kObject1InitPosY), kEgoVel);
  auto agent_history_3 = BuildVehicleHistoryByConstVel(
      kEgoId3, kNumHistory, Vec2d(kEgoInitPosX, kObject1InitPosY), kEgoVel);
  auto obj_history = BuildVehicleHistoryByConstVel(
      kObject1Id, kNumHistory, Vec2d(0.0, kObject1InitPosY), kObject1Vel);
  auto av_history = BuildVehicleHistoryByConstVel(kAvObjectId, kNumHistory,
                                                  Vec2d::Zero(), kEgoVel);
  FLAGS_prediction_use_tracker_history = false;
  const auto obj_sampler = ObjectHistorySampler(
      {&agent_history_1, &agent_history_2, &agent_history_3, &obj_history},
      av_history, kUpdateTimeStep * (kNumHistory - 1), kUpdateTimeStep,
      kNumHistory,
      /*enable_smoothing=*/false, FLAGS_prediction_use_tracker_history);

  std::vector<const ObjectHistory*> objects_history = {
      &agent_history_1, &agent_history_2, &agent_history_3, &obj_history,
      &av_history};

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  std::string object_id("1");
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  const auto context = BuildOneObjectPredictionContext(object_id, &input);
  MapSampler map_sampler(
      context.semantic_map_manager(),
      context.traffic_light_manager().GetOriginalTlStateMap(), 10, 5,
      MapSampler::SampleType::ADAPTIVE);
  std::vector<ObjectIDType> obj_ids({kEgoId1, kEgoId2, kEgoId3, kObject1Id});
  const auto inference_results = cutin_sl_net_inferencer.PredictForObjects(
      obj_ids, obj_sampler, &map_sampler, *context.av_drive_passage());
  EXPECT_EQ(inference_results.size(), 4);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
