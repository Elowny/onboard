
#include <vector>

#include "benchmark/benchmark.h"
#include "glog/logging.h"

#include "onboard/math/vec.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature.h"
#include "onboard/prediction/feature_extractor/cutin_sl_feature_extractor.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/inferencer/cutin_sl_net_inferencer.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kEgoInitPosX = 10.0;
constexpr double kEgoVel = 2.0;
constexpr double kObject1InitPosY = 10.0;
constexpr double kObject1Vel = 1.0;
constexpr double kMaxHeadingDiff = M_PI / 6.0;
constexpr double kForwardLane = 50.0;          // m.
constexpr double kForwardExtendLength = 10.0;  // m.
constexpr double kDrivePassageStepS = 4.0;
constexpr double kDrivePassageBackwardLength = 40.0;  // m.
constexpr char kObject1Id[] = "999";
constexpr char kEgoId[] = "9999";

constexpr int kNumHistory = 10;
constexpr double kUpdateTimeStep = 0.1;
constexpr double kAvMapRadius = 210.0;
}  // namespace
static void BM_CutinSLNetPrepareInputs(benchmark::State& state) {  // NOLINT
  NetParam cutin_sl_net_param;
  CHECK_OK(
      GetProtoParamById("Q0001", "cutin_sl_net_param", &cutin_sl_net_param));
  cutin_sl_net::CutinSLNetInferencer cutin_sl_net_inferencer(
      cutin_sl_net_param);

  // get obj_sampler
  auto agent_history = BuildVehicleHistoryByConstVel(
      kEgoId, kNumHistory, Vec2d(kEgoInitPosX, 0.0), kEgoVel);
  auto obj_history = BuildVehicleHistoryByConstVel(
      kObject1Id, kNumHistory, Vec2d(0.0, kObject1InitPosY), kObject1Vel);
  auto av_history = BuildVehicleHistoryByConstVel(kAvObjectId, kNumHistory,
                                                  Vec2d::Zero(), kEgoVel);
  const auto obj_sampler = ObjectHistorySampler(
      {&agent_history, &obj_history}, av_history,
      kUpdateTimeStep * (kNumHistory - 1), kUpdateTimeStep, kNumHistory,
      /*enable_smoothing=*/false,
      /*use_tracker_history=*/false);

  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  MapSampler map_sampler(psmm, tl_state, kFeatureV2MaxMapSampleLen,
                         kFeatureV2MapSegmentNum,
                         MapSampler::SampleType::ADAPTIVE);
  const auto* av_motion_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_motion_history->states.back().pos;
  const auto lane_centers =
      map_sampler.GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler.GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);
  const auto crosswalks =
      map_sampler.GetCrosswalksWithRadius(av_pos, kAvMapRadius);
  const auto av_resampled_histories = obj_sampler.GetResampledAVMotionHistory();
  const auto av_last_state = av_resampled_histories->states.back();

  const auto lane_id_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, av_last_state.pos, av_last_state.heading,
          /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);

  const auto lps =
      SearchLanePath(av_last_state.pos, psmm, *lane_id_or, kForwardLane,
                     /*is_reverse_driving=*/false);
  auto extended_lp =
      ExtendMostStraightLanePath(lps[0], psmm, kForwardExtendLength,
                                 /*is_reversed=*/false);
  auto drive_passage = planner::BuildDrivePassageForPrediction(
      psmm, extended_lp, kDrivePassageStepS,
      /*avoid_loop=*/true,
      /*backward_extend_len=*/kDrivePassageBackwardLength,
      kMaxLateralBoundaryForPredictionDp);

  // 2. run bm
  for (auto _ : state) {
    const auto feature =
        ExtractCutinSLFeature(kEgoId, obj_sampler, *drive_passage, lane_centers,
                              lane_boundaries, crosswalks, &map_sampler);
    std::vector<prediction::CutinSLFeature> features = {*feature};
    benchmark::DoNotOptimize(
        cutin_sl_net_inferencer.GetBatchInputs(cutin_sl_net_param, features));
  }
}

static void BM_CutinSLNetPredictForObjects(benchmark::State& state) {  // NOLINT
  NetParam cutin_sl_net_param;
  CHECK_OK(
      GetProtoParamById("Q0001", "cutin_sl_net_param", &cutin_sl_net_param));
  cutin_sl_net::CutinSLNetInferencer cutin_sl_net_inferencer(
      cutin_sl_net_param);

  // get obj_sampler
  auto agent_history = BuildVehicleHistoryByConstVel(
      kEgoId, kNumHistory, Vec2d(kEgoInitPosX, 0.0), kEgoVel);
  auto obj_history = BuildVehicleHistoryByConstVel(
      kObject1Id, kNumHistory, Vec2d(0.0, kObject1InitPosY), kObject1Vel);
  auto av_history = BuildVehicleHistoryByConstVel(kAvObjectId, kNumHistory,
                                                  Vec2d::Zero(), kEgoVel);
  const auto obj_sampler = ObjectHistorySampler(
      {&agent_history, &obj_history}, av_history,
      kUpdateTimeStep * (kNumHistory - 1), kUpdateTimeStep, kNumHistory,
      /*enable_smoothing=*/false,
      /*use_tracker_history=*/false);

  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  MapSampler map_sampler(psmm, tl_state, kFeatureV2MaxMapSampleLen,
                         kFeatureV2MapSegmentNum,
                         MapSampler::SampleType::ADAPTIVE);
  const auto* av_motion_history = obj_sampler.GetResampledAVMotionHistory();
  const auto& av_pos = av_motion_history->states.back().pos;
  const auto lane_centers =
      map_sampler.GetLaneCentersWithRadius(av_pos, kAvMapRadius);
  const auto lane_boundaries =
      map_sampler.GetSolidLaneBoundariesWithRadius(av_pos, kAvMapRadius);
  const auto crosswalks =
      map_sampler.GetCrosswalksWithRadius(av_pos, kAvMapRadius);
  const auto av_resampled_histories = obj_sampler.GetResampledAVMotionHistory();
  const auto av_last_state = av_resampled_histories->states.back();

  const auto lane_id_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, av_last_state.pos, av_last_state.heading,
          /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);

  const auto lps =
      SearchLanePath(av_last_state.pos, psmm, *lane_id_or, kForwardLane,
                     /*is_reverse_driving=*/false);
  auto extended_lp =
      ExtendMostStraightLanePath(lps[0], psmm, kForwardExtendLength,
                                 /*is_reversed=*/false);
  auto drive_passage = planner::BuildDrivePassageForPrediction(
      psmm, extended_lp, kDrivePassageStepS,
      /*avoid_loop=*/true,
      /*backward_extend_len=*/kDrivePassageBackwardLength,
      kMaxLateralBoundaryForPredictionDp);
  const auto feature =
      ExtractCutinSLFeature(kEgoId, obj_sampler, *drive_passage, lane_centers,
                            lane_boundaries, crosswalks, &map_sampler);
  std::vector<prediction::CutinSLFeature> features = {*feature};

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        cutin_sl_net_inferencer.PredictForObjects(features));
  }
}
BENCHMARK(BM_CutinSLNetPrepareInputs);
BENCHMARK(BM_CutinSLNetPredictForObjects);
}  // namespace prediction
}  // namespace qcraft

BENCHMARK_MAIN();
