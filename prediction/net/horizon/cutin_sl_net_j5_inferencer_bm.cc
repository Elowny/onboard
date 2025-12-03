#include <string>  // for allocator, basic_string, string
#include <vector>  // for vector

#include "benchmark/benchmark.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"  // for Box2d
#include "onboard/math/vec.h"             // for Vec2d
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/traffic_light_manager.h"  // for TrafficLightManager
#include "onboard/prediction/feature_extractor/map_sampler.h"  // for MapSampler
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/cutin_sl_net_j5_inferencer.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/prediction_defs.h"  // for ObjectMotionState, ObjectIDType, ObjectMot...
#include "onboard/prediction/prediction_flags.h"  // for FLAGS_prediction_use_tracker_history
#include "onboard/prediction/prediction_util.h"
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
constexpr double kAvMapRadius = 210.0;
constexpr int kMaxLaneCenterNum = 200;
constexpr int kMaxLaneBoundaryNum = 200;

static void BM_CutinNetJ5PredictForObjects(benchmark::State& state) {  // NOLINT
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
  auto context = BuildOneObjectPredictionContext(object_id, &input);
  MapSampler map_sampler(
      context.semantic_map_manager(),
      context.traffic_light_manager().GetOriginalTlStateMap(), 10, 5,
      MapSampler::SampleType::ADAPTIVE);

  const auto lane_centers =
      map_sampler.GetLaneCentersWithRadius(Vec2d::Zero(), kAvMapRadius);
  const auto lane_boundaries =
      map_sampler.GetSolidLaneBoundariesWithRadius(Vec2d::Zero(), kAvMapRadius);
  const Box2d region_box = GetRegionBox(Vec2d::Zero(), 0.0, 150, 150, 100);
  const auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2dWithLanes(
      region_box, kMaxLaneCenterNum, lane_centers,
      /*compute_boundary_distance=*/false);
  const auto lbs =
      map_sampler.GetNearestLaneBoundaryPolylinesInBox2dWithLaneBoundaries(
          region_box, kMaxLaneBoundaryNum, lane_boundaries);

  std::vector<ObjectIDType> obj_ids({kEgoId1, kEgoId2, kEgoId3, kObject1Id});
  // warm up
  cutin_sl_net_inferencer.PredictForObjects(obj_ids, obj_sampler, &map_sampler,
                                            *context.av_drive_passage());
  cutin_sl_net_inferencer.PredictForObjects(obj_ids, obj_sampler, &map_sampler,
                                            *context.av_drive_passage());

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(cutin_sl_net_inferencer.PredictForObjects(
        obj_ids, obj_sampler, &map_sampler, *context.av_drive_passage()));
  }
}
BENCHMARK(BM_CutinNetJ5PredictForObjects);
}  // namespace
}  // namespace prediction
}  // namespace qcraft
BENCHMARK_MAIN();
