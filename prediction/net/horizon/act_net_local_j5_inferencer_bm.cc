#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "benchmark/benchmark.h"
#include "glog/logging.h"

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
#include "onboard/prediction/net/horizon/act_net_local_j5_inferencer.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/errors.h"

namespace qcraft {
namespace prediction {
static void BM_ActNetLocalJ5PrepareInputs(benchmark::State& state) {  // NOLINT
  NetParam act_net_local_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "act_net_local_j5_param",
                             &act_net_local_j5_param));
  actnetlocalj5::ActNetLocalJ5Inferencer act_net_inferencer(
      act_net_local_j5_param);
  const int max_bs = act_net_local_j5_param.max_batch_size();
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

  std::vector<actnetlocalj5::AgentRef> agent_origins;
  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(act_net_inferencer.GetBatchInputs(
        objects_history, obj_sampler, &map_sampler, agent_origins));
  }
}

static void BM_ActnetLocalJ5PredictForObjects(
    benchmark::State& state) {  // NOLINT
  NetParam act_net_local_j5_param;
  CHECK_OK(GetProtoParamById("Q0001", "act_net_local_j5_param",
                             &act_net_local_j5_param));
  actnetlocalj5::ActNetLocalJ5Inferencer act_net_inferencer(
      act_net_local_j5_param);
  const int max_bs = act_net_local_j5_param.max_batch_size();
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

  auto threadpool = std::make_unique<ThreadPool>(2);

  MapSampler map_sampler(
      context.semantic_map_manager(),
      context.traffic_light_manager().GetOriginalTlStateMap(), 10, 5,
      MapSampler::SampleType::ADAPTIVE);

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(act_net_inferencer.PredictForObjects(
        objects_history, obj_sampler, &map_sampler, threadpool.get()));
  }
}

BENCHMARK(BM_ActNetLocalJ5PrepareInputs);
BENCHMARK(BM_ActnetLocalJ5PredictForObjects);
}  // namespace prediction
}  // namespace qcraft

BENCHMARK_MAIN();
