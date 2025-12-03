#include "onboard/prediction/net/horizon/act_net_j5_inferencer.h"

#include <algorithm>
#include <unordered_map>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/nets/proto/net_param.pb.h"
#include "onboard/params/param_finder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/map_sampler.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/net/horizon/horizon_tensor_wrapper.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(ActNetJ5InferencerTest, GenBatchInputs) {
  NetParam act_net_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "act_net_j5_param", &act_net_j5_param));
  actnetj5::ActNetJ5Inferencer act_net_j5_inferencer(act_net_j5_param);
  const int max_bs = act_net_j5_param.max_batch_size();
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

  MapSampler map_sampler(
      context.semantic_map_manager(),
      context.traffic_light_manager().GetOriginalTlStateMap(), 10, 5,
      MapSampler::SampleType::ADAPTIVE);

  // 2. run test
  std::vector<actnetj5::AgentRef> agent_origins;
  const int cur_bs = act_net_j5_inferencer.GenBatchInputs(
      objects_history, obj_sampler, &map_sampler, agent_origins);
  EXPECT_EQ(cur_bs, max_bs);

  auto* j5_qnn = act_net_j5_inferencer.GetJ5QNN();
  const auto& input_tensor_map = j5_qnn->GetInputMap();
  // Tensor with shape [64, 11, 10, 32] for actor_info_hist.
  const auto& actor_info_hist = input_tensor_map.at("actor_info_hist");
  // Check agent history state info.
  EXPECT_NEAR(actor_info_hist->index_get_deqat(0, 0, 9, 0), 0.5, 1e-6);
  EXPECT_NEAR(actor_info_hist->index_get_deqat(0, 2, 9, 0), 0.166667, 1e-6);
  EXPECT_NEAR(actor_info_hist->index_get_deqat(0, 4, 9, 0), 1.0, 1e-6);

  // Tensor with shape [64, 18, 1, 32] for actor_info_attr.
  const auto& actor_info_attr = input_tensor_map.at("actor_info_attr");
  // Check agent attribute info.
  EXPECT_NEAR(actor_info_attr->index_get_deqat(0, 2, 0, 0), 1.0, 1e-6);
  EXPECT_NEAR(actor_info_attr->index_get_deqat(0, 16, 0, 0), 0.4, 1e-6);
  EXPECT_NEAR(actor_info_attr->index_get_deqat(0, 17, 0, 0), 0.666667, 1e-6);

  // Tensor with shape [64, 2, 1, 32] for actor_ctrs
  const auto& actor_ctrs = input_tensor_map.at("actor_ctrs");
  // Check agent current pos.
  EXPECT_NEAR(actor_ctrs->index_get_deqat(0, 0, 0, 0), 0.0, 1e-6);
  EXPECT_NEAR(actor_ctrs->index_get_deqat(0, 1, 0, 0), 0.0, 1e-6);
}

TEST(ActNetJ5InferencerTest, PredictForObjects) {
  NetParam act_net_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "act_net_j5_param", &act_net_j5_param));
  actnetj5::ActNetJ5Inferencer act_net_j5_inferencer(act_net_j5_param);
  const int max_bs = act_net_j5_param.max_batch_size();
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

  auto threadpool = std::make_unique<ThreadPool>(0);

  MapSampler map_sampler(
      context.semantic_map_manager(),
      context.traffic_light_manager().GetOriginalTlStateMap(), 10, 5,
      MapSampler::SampleType::ADAPTIVE);

  // 2. run test
  const auto inference_results = act_net_j5_inferencer.PredictForObjects(
      objects_history, obj_sampler, &map_sampler, threadpool.get());
  EXPECT_EQ(inference_results.size(), max_bs);
  auto* j5_qnn = act_net_j5_inferencer.GetJ5QNN();
  EXPECT_NE(j5_qnn, nullptr);
  const auto& input_tensor_map = j5_qnn->GetInputMap();
  EXPECT_EQ(input_tensor_map.size(), act_net_j5_param.input_tensor_list_size());

  const auto& actor_info_hist = input_tensor_map.at("actor_info_hist");
  EXPECT_EQ(actor_info_hist->size(), max_bs * 11 * 10 * 32);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
