#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"

#include <algorithm>  // for max
#include <memory>     // for make_unique
#include <string>     // for to_string

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"   // for BufferedLoggerWrapper
#include "onboard/lite/check.h"               // for QCHECK_OK
#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/params/param_finder.h"      // for GetProtoParamById
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/av_context.h"           // for AvContext
#include "onboard/prediction/container/object_history_span.h"  // for ObjectHistorySpan
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_object.h"  // for PredictionObject
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/lane_selection_net_j5_inferencer.h"  // for ActNetJ5Inferencer
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/scheduler/scenario_analyzer.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"  // for BuildMultiObjectsPredictionContext
#include "onboard/proto/vehicle.pb.h"        // for VehicleGeometryParamsProto
#include "onboard/utils/elements_history.h"  // for Node

namespace qcraft {
namespace prediction {
namespace {

TEST(LaneSelectionNetJ5PredictorTest, MakeLaneSelectionNetJ5Prediction) {
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

  const auto pred_results = MakeLaneSelectionNetJ5Prediction(
      context, objects_history, lane_selection_net_j5_inferencer,
      object_scenarios, obj_sampler, thread_pool.get());
  EXPECT_EQ(pred_results.size(), kNumObj);
}

TEST(LaneSelectionNetJ5PredictorTest,
     SelectLaneSelectionJ5PredictedObjectsWithDps) {
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
  const auto results = SelectLaneSelectionJ5PredictedObjectsWithDps(
      av_history.back().val.bounding_box(), objects_history, obj_sampler,
      context, object_scenarios, thread_pool.get());
  EXPECT_EQ(results.size(), kNumObj);
}

TEST(LaneSelectionNetJ5PredictorTest,
     GenerateLaneSelectionTrajsByModelAndHeuristicInfo) {
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

  const auto results = GenerateLaneSelectionTrajsByModelAndHeuristicInfo(
      agent_dps_map, infer_result_map, obj_sampler, context,
      /*is_mapless=*/false, thread_pool.get());
  EXPECT_EQ(results.size(), kNumObj);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
