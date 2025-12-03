#include "onboard/prediction/predictor/act_net_predictor.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <memory>  // for make_unique, unique_ptr
#include <vector>

#include "gtest/gtest.h"  // for Test, TestInfo, EXPECT_EQ, Message, TEST

#include "onboard/lite/check.h"    // for QCHECK_OK
#include "onboard/lite/logging.h"  // for BufferedLoggerWrapper
#include "onboard/math/vec.h"
#include "onboard/params/param_finder.h"  // for GetProtoParamById
#include "onboard/params/param_manager.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/av_context.h"  // for AvContext
#include "onboard/prediction/container/model_pool.h"  // for BuildOneObjectPredictionContext
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"  // for BuildOneObjectPredictionContext
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

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

TEST(ActNetPredictorTest, CutinTest) {
  FLAGS_print_prediction_time_stats = true;
  FLAGS_prediction_enable_auxiliary_cutin_sl_net = true;

  auto param_manager = CreateParamManagerFromCarId("Q8001");
  QCHECK(param_manager != nullptr);
  auto param_finder = CreateParamFinderWithCarId("Q8001");
  QCHECK(param_finder != nullptr);
  ModelPool model_pool(*param_manager, *param_finder, {});

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

  const auto pred_results = MakeActNetPrediction(
      context, objects_history, model_pool.GetActNetInferencer(),
      model_pool.GetCutinSLNetInferencer(), obj_sampler, threadpool.get());
  EXPECT_EQ(pred_results.size(), 2);
}
}  // namespace
}  // namespace qcraft::prediction
