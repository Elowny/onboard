#include <algorithm>  // for max
#include <map>        // for map, operator==
#include <memory>     // for make_unique
#include <string>     // for to_string, allocator
#include <vector>     // for vector

#include "absl/types/span.h"  // for Span
#include "benchmark/benchmark.h"

#include "onboard/async/thread_pool.h"
#include "onboard/global/buffered_logger.h"   // for BufferedLoggerWrapper
#include "onboard/lite/check.h"               // for QCHECK_OK
#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/params/param_finder.h"      // for GetProtoParamById
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/av_context.h"      // for AvContext
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/object_history_span.h"  // for ObjectHistorySpan
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/container/prediction_object.h"  // for PredictionObject
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/lane_selection_net_j5_inferencer.h"  // for ActNetJ5Inferencer
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/prediction_defs.h"  // for ObjectIDType
#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"
#include "onboard/prediction/predictor/vehicle_lane_follow_predictor.h"
#include "onboard/prediction/proto/prediction_common.pb.h"  // for ObjectPredictionScenario
#include "onboard/prediction/scheduler/scenario_analyzer.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"  // for BuildMultiObjectsPredictionContext
#include "onboard/proto/vehicle.pb.h"        // for VehicleGeometryParamsProto
#include "onboard/utils/elements_history.h"  // for Node
namespace qcraft {
namespace prediction {
namespace {

[[maybe_unused]] static void BM_MakeLaneSelectionNetJ5Prediction(
    benchmark::State& state) {  // NOLINT
  NetParam lane_selection_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                              &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_j5_inferencer(lane_selection_net_j5_param);
  constexpr int kNumObj = 16;
  // 1. build a proto
  const auto& psmm = planner::CreateDojoTestPSMM();

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(kNumObj, &input);

  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(kNumObj);
  for (int i = 0; i < kNumObj; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  const auto object_scenarios = AnalyzeScenarios(context, objects_history);
  auto thread_pool = std::make_unique<ThreadPool>(2);

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(MakeLaneSelectionNetJ5Prediction(
        context, objects_history, lane_selection_net_j5_inferencer,
        object_scenarios, obj_sampler, thread_pool.get()));
  }
}

[[maybe_unused]] static void BM_SelectLaneSelectionJ5PredictedObjectsWithDps(
    benchmark::State& state) {  // NOLINT
  NetParam lane_selection_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                              &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_j5_inferencer(lane_selection_net_j5_param);
  constexpr int kNumObj = 16;
  // 1. build a proto
  const auto& psmm = planner::CreateDojoTestPSMM();

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(kNumObj, &input);

  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(kNumObj);
  for (int i = 0; i < kNumObj; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  const auto object_scenarios = AnalyzeScenarios(context, objects_history);

  const auto& av_context = context.av_context();
  const ObjectHistorySpan av_history =
      av_context.GetAvObjectHistory().GetHistory();
  auto thread_pool = std::make_unique<ThreadPool>(2);

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(SelectLaneSelectionJ5PredictedObjectsWithDps(
        av_history.back().val.bounding_box(), objects_history, obj_sampler,
        context, object_scenarios, thread_pool.get()));
  }
}

[[maybe_unused]] static void
BM_GenerateLaneSelectionTrajsByModelAndHeuristicInfo(
    benchmark::State& state) {  // NOLINT
  NetParam lane_selection_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                              &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_j5_inferencer(lane_selection_net_j5_param);
  constexpr int kNumObj = 16;
  // 1. build a proto
  const auto& psmm = planner::CreateDojoTestPSMM();

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(kNumObj, &input);

  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(kNumObj);
  for (int i = 0; i < kNumObj; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  const auto object_scenarios = AnalyzeScenarios(context, objects_history);

  const auto& av_context = context.av_context();
  const ObjectHistorySpan av_history =
      av_context.GetAvObjectHistory().GetHistory();

  auto thread_pool = std::make_unique<ThreadPool>(2);
  auto agent_dps_map = SelectLaneSelectionJ5PredictedObjectsWithDps(
      av_history.back().val.bounding_box(), objects_history, obj_sampler,
      context, object_scenarios, thread_pool.get());

  const auto infer_result_map =
      lane_selection_net_j5_inferencer.PredictForObjects(obj_sampler,
                                                         agent_dps_map);

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(GenerateLaneSelectionTrajsByModelAndHeuristicInfo(
        agent_dps_map, infer_result_map, obj_sampler, context,
        /*is_mapless=*/false, thread_pool.get()));
  }
}

[[maybe_unused]] int MakeWholePredictionForObjects(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const lane_selection_net::LaneSelectionNetJ5Inferencer&
        lane_selection_net_j5_inferencer,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler) {
  auto thread_pool = std::make_unique<ThreadPool>(2);
  const auto obj_pred_map = MakeLaneSelectionNetJ5Prediction(
      prediction_context, objects_history, lane_selection_net_j5_inferencer,
      object_scenarios, obj_sampler, thread_pool.get());

  for (const auto& obj : objects_history) {
    if (obj_pred_map.find(obj->id()) == obj_pred_map.end()) {
      MakeVehicleLaneFollowPrediction(
          obj_sampler.GetResampledMotionHistoryById(obj->id()),
          prediction_context, object_scenarios.at(obj->id()),
          /*ignore_off_road=*/false);
    }
  }

  return 0;
}

[[maybe_unused]] int WrapperFuncLaneSelectionForComparing(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const lane_selection_net::LaneSelectionNetJ5Inferencer&
        lane_selection_net_j5_inferencer,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler) {
  auto thread_pool = std::make_unique<ThreadPool>(2);
  const auto obj_pred_map = MakeLaneSelectionNetJ5Prediction(
      prediction_context, objects_history, lane_selection_net_j5_inferencer,
      object_scenarios, obj_sampler, thread_pool.get());
  return 0;
}

[[maybe_unused]] int WrapperFuncLaneFollowForComparing(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler) {
  for (const auto& obj : objects_history) {
    MakeVehicleLaneFollowPrediction(
        obj_sampler.GetResampledMotionHistoryById(obj->id()),
        prediction_context, object_scenarios.at(obj->id()),
        /*ignore_off_road=*/false);
  }

  return 0;
}

[[maybe_unused]] static void BM_MakeWholePredictionForObjects(
    benchmark::State& state) {  // NOLINT
  NetParam lane_selection_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                              &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_j5_inferencer(lane_selection_net_j5_param);
  constexpr int kNumObj = 16;
  // 1. build a proto
  const auto& psmm = planner::CreateDojoTestPSMM();

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(kNumObj, &input);

  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(kNumObj);
  for (int i = 0; i < kNumObj; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  const auto object_scenarios = AnalyzeScenarios(context, objects_history);

  // warm up
  MakeWholePredictionForObjects(context, objects_history,
                                lane_selection_net_j5_inferencer,
                                object_scenarios, obj_sampler);

  MakeWholePredictionForObjects(context, objects_history,
                                lane_selection_net_j5_inferencer,
                                object_scenarios, obj_sampler);

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(MakeWholePredictionForObjects(
        context, objects_history, lane_selection_net_j5_inferencer,
        object_scenarios, obj_sampler));
  }
}

[[maybe_unused]] static void BM_WrapperFuncLaneSelectionForComparing(
    benchmark::State& state) {  // NOLINT
  NetParam lane_selection_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                              &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_j5_inferencer(lane_selection_net_j5_param);
  constexpr int kNumObj = 16;
  // 1. build a proto
  const auto& psmm = planner::CreateDojoTestPSMM();

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(kNumObj, &input);

  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(kNumObj);
  for (int i = 0; i < kNumObj; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  const auto object_scenarios = AnalyzeScenarios(context, objects_history);

  // warm up
  WrapperFuncLaneSelectionForComparing(context, objects_history,
                                       lane_selection_net_j5_inferencer,
                                       object_scenarios, obj_sampler);
  WrapperFuncLaneSelectionForComparing(context, objects_history,
                                       lane_selection_net_j5_inferencer,
                                       object_scenarios, obj_sampler);
  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(WrapperFuncLaneSelectionForComparing(
        context, objects_history, lane_selection_net_j5_inferencer,
        object_scenarios, obj_sampler));
  }
}

[[maybe_unused]] static void BM_WrapperFuncLaneFollowForComparing(
    benchmark::State& state) {  // NOLINT
  NetParam lane_selection_net_j5_param;
  QCHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                              &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_j5_inferencer(lane_selection_net_j5_param);
  constexpr int kNumObj = 16;
  // 1. build a proto
  const auto& psmm = planner::CreateDojoTestPSMM();

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  auto context = BuildMultiObjectsPredictionContext(kNumObj, &input);

  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(kNumObj);
  for (int i = 0; i < kNumObj; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const ObjectHistorySampler obj_sampler(objects_history, *objects_history[0],
                                         objects_history[0]->timestamp(), 0.1,
                                         10,
                                         /*enable_smoothing=*/false,
                                         /*use_tracker_history=*/true);
  const auto object_scenarios = AnalyzeScenarios(context, objects_history);

  // warm up
  WrapperFuncLaneFollowForComparing(context, objects_history, object_scenarios,
                                    obj_sampler);
  WrapperFuncLaneFollowForComparing(context, objects_history, object_scenarios,
                                    obj_sampler);
  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(WrapperFuncLaneFollowForComparing(
        context, objects_history, object_scenarios, obj_sampler));
  }
}
// BENCHMARK(BM_MakeLaneSelectionNetJ5Prediction);
// BENCHMARK(BM_SelectLaneSelectionJ5PredictedObjectsWithDps);
// BENCHMARK(BM_GenerateLaneSelectionTrajsByModelAndHeuristicInfo);

// compare lane selection and lane follow:
// pred all objs using both lane selection and lane follow
BENCHMARK(BM_MakeWholePredictionForObjects);
// pred all objs using only lane follow
BENCHMARK(BM_WrapperFuncLaneFollowForComparing);
// pred part objs using lane selection
BENCHMARK(BM_WrapperFuncLaneSelectionForComparing);

}  // namespace
}  // namespace prediction
}  // namespace qcraft
BENCHMARK_MAIN();
