#include "onboard/prediction/net/horizon/lane_selection_net_j5_inferencer.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/lane_selection_net.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(LaneSelectionNetJ5InferencerTest, PredictForObjects) {
  NetParam lane_selection_net_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                             &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_inferencer(lane_selection_net_j5_param);
  const int max_bs = lane_selection_net_j5_param.max_batch_size();

  // 1. build a proto
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

  // 2. build dp
  int total_lane_paths_num = 0;
  AgentDrivePassagesMap agent_dps_map;
  std::map<ObjectIDType, std::vector<planner::DrivePassage>>
      agent_dp_options_vec;

  for (int i = 0; i < objects_history.size(); ++i) {
    const auto& agent_hist = objects_history[i];
    const auto& agent_id = agent_hist->id();
    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);

    const auto agent_dp_options =
        BuildLaneSelectionObjectDrivePassages(agent_history, context);
    if (agent_dp_options.size() <= 0) {
      continue;
    }
    total_lane_paths_num += agent_dp_options.size();
    if (total_lane_paths_num > lane_selection_net::kMaxPredLanePathsNum) {
      break;
    }
    agent_dp_options_vec[agent_id] = agent_dp_options;
  }

  for (const auto& agent_dp_options : agent_dp_options_vec) {
    for (int j = 0; j < agent_dp_options.second.size(); ++j) {
      agent_dps_map[agent_dp_options.first].push_back(
          &agent_dp_options.second[j]);
    }
  }
  // 3. run test

  const auto inference_results =
      lane_selection_net_inferencer.PredictForObjects(obj_sampler,
                                                      agent_dps_map);
  EXPECT_LE(inference_results.size(), max_bs);
}

TEST(LaneSelectionNetJ5InferencerTest, GenBatchInputs) {
  NetParam lane_selection_net_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                             &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_inferencer(lane_selection_net_j5_param);
  const int max_bs = lane_selection_net_j5_param.max_batch_size();
  // 1. build a proto
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

  // build dp
  int total_lane_paths_num = 0;
  AgentDrivePassagesMap agent_dps_map;
  std::map<ObjectIDType, std::vector<planner::DrivePassage>>
      agent_dp_options_vec;

  for (int i = 0; i < objects_history.size(); ++i) {
    const auto& agent_hist = objects_history[i];
    const auto& agent_id = agent_hist->id();
    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);

    const auto agent_dp_options =
        BuildLaneSelectionObjectDrivePassages(agent_history, context);
    if (agent_dp_options.size() <= 0) {
      continue;
    }
    total_lane_paths_num += agent_dp_options.size();
    if (total_lane_paths_num > lane_selection_net::kMaxPredLanePathsNum) {
      break;
    }
    agent_dp_options_vec[agent_id] = agent_dp_options;
  }

  for (const auto& agent_dp_options : agent_dp_options_vec) {
    for (int j = 0; j < agent_dp_options.second.size(); ++j) {
      agent_dps_map[agent_dp_options.first].push_back(
          &agent_dp_options.second[j]);
    }
  }

  // 2. run test

  const auto lane_path_index_info =
      lane_selection_net_inferencer.GenBatchInputs(agent_dps_map, obj_sampler);

  EXPECT_LE(lane_path_index_info.size(), max_bs);
}
TEST(LaneSelectionNetJ5InferencerTest, ExtractOutputs) {
  NetParam lane_selection_net_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "lane_selection_net_j5_param",
                             &lane_selection_net_j5_param));
  lane_selection_net::LaneSelectionNetJ5Inferencer
      lane_selection_net_inferencer(lane_selection_net_j5_param);
  const int max_bs = lane_selection_net_j5_param.max_batch_size();

  // 1. build a proto
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

  // build dp
  int total_lane_paths_num = 0;
  AgentDrivePassagesMap agent_dps_map;
  std::map<ObjectIDType, std::vector<planner::DrivePassage>>
      agent_dp_options_vec;

  for (int i = 0; i < objects_history.size(); ++i) {
    const auto& agent_hist = objects_history[i];
    const auto& agent_id = agent_hist->id();
    const auto& agent_history =
        obj_sampler.GetResampledMotionHistoryById(agent_id);

    const auto agent_dp_options =
        BuildLaneSelectionObjectDrivePassages(agent_history, context);
    if (agent_dp_options.size() <= 0) {
      continue;
    }
    total_lane_paths_num += agent_dp_options.size();
    if (total_lane_paths_num > lane_selection_net::kMaxPredLanePathsNum) {
      break;
    }
    agent_dp_options_vec[agent_id] = agent_dp_options;
  }

  for (const auto& agent_dp_options : agent_dp_options_vec) {
    for (int j = 0; j < agent_dp_options.second.size(); ++j) {
      agent_dps_map[agent_dp_options.first].push_back(
          &agent_dp_options.second[j]);
    }
  }
  const std::vector<lane_selection_net::AgentDpsIndexInfo> batch_idx_info =
      lane_selection_net_inferencer.GenBatchInputs(agent_dps_map, obj_sampler);

  // 2. run test
  const auto res = lane_selection_net_inferencer.ExtractOutputs(batch_idx_info);
  EXPECT_LE(res.size(), max_bs);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
