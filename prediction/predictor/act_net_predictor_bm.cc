#include <algorithm>  // for max
#include <memory>     // for allocator, make_unique, unique_ptr
#include <string>     // for to_string
#include <vector>     // for vector

#include "absl/types/span.h"      // for Span
#include "benchmark/benchmark.h"  // for State, DoNotOptimize, State::StateIterator

#include "onboard/async/thread_pool.h"        // for ThreadPool
#include "onboard/lite/check.h"               // for QCHECK_OK
#include "onboard/lite/logging.h"             // for BufferedLoggerWrapper
#include "onboard/nets/proto/net_param.pb.h"  // for NetParam
#include "onboard/params/param_finder.h"      // for GetProtoParamById
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"   // for ObjectHistory
#include "onboard/prediction/container/objects_history.h"  // for ObjectsHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/inferencer/act_net_inferencer.h"  // for ActNetInferencer
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/predictor/act_net_predictor.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"  // for BuildMultiObjectsPredictionContext
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
namespace {

static void BM_MakeActNetPrediction(benchmark::State& state) {  // NOLINT
  NetParam act_net_param;
  QCHECK_OK(GetProtoParamById("Q0001", "act_net_param", &act_net_param));
  actnet::ActNetInferencer act_net_inferencer(act_net_param);
  const int max_bs = act_net_param.max_batch_size();
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

  // 2. run bm
  for (auto _ : state) {
    benchmark::DoNotOptimize(MakeActNetPrediction(
        context, objects_history, &act_net_inferencer,
        /*cutin_sl_inferencer_ptr=*/nullptr, obj_sampler, threadpool.get()));
  }
}
BENCHMARK(BM_MakeActNetPrediction);

}  // namespace
}  // namespace prediction
}  // namespace qcraft
BENCHMARK_MAIN();
