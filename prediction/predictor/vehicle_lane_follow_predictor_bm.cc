#include <memory>
#include <string>
#include <utility>

#include "benchmark/benchmark.h"

#include "onboard/math/vec.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/predictor/vehicle_lane_follow_predictor.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::prediction {
namespace {
const double kTimeStep = 0.1;
const int kHistoryNum = 10;
static void BM_MakeVehicleLaneFollowPrediction(
    benchmark::State& state) {  // NOLINT
  const auto& psmm = planner::CreateDojoTestPSMM();

  const Vec2d pos(141.2, -216.5);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(10.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  FLAGS_prediction_use_tracker_history = false;

  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  input.objects_history->Update(objects_proto, *input.localization_transform,
                                /*thread_pool=*/nullptr);
  const auto& hist = input.objects_history->at(id);

  const double current_ts = hist.timestamp();
  const ObjectHistorySampler obj_sampler(
      {&hist}, hist, current_ts, kTimeStep, kHistoryNum,
      /*enable_smoothing=*/true, FLAGS_prediction_use_tracker_history);
  ObjectPredictionScenario scenario;
  PredictionContext context(input);

  for (auto _ : state) {
    benchmark::DoNotOptimize(MakeVehicleLaneFollowPrediction(
        obj_sampler.GetResampledMotionHistoryById(id), context, scenario,
        /*ignore_off_road=*/false));
  }
}

BENCHMARK(BM_MakeVehicleLaneFollowPrediction);
}  // namespace
}  // namespace qcraft::prediction

BENCHMARK_MAIN();
