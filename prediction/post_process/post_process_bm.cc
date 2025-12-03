#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "benchmark/benchmark.h"

#include "onboard/global/trace.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/post_process.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_message_compressor.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {

namespace {

absl::StatusOr<std::unique_ptr<ObjectsPredictionProto>> AssemblePredictionProto(
    const std::map<ObjectIDType, ObjectPredictionResult>&
        object_prediction_results) {
  SCOPED_QTRACE("AssemblePredictionProto");
  std::vector<const ObjectPredictionResult*> objects;
  objects.reserve(object_prediction_results.size());
  for (const auto& key_val : object_prediction_results) {
    objects.push_back(&key_val.second);
  }
  std::sort(objects.begin(), objects.end(),
            [](const auto* a, const auto* b) { return a->id < b->id; });
  auto prediction = std::make_unique<ObjectsPredictionProto>();
  prediction->Clear();
  prediction->mutable_objects()->Reserve(object_prediction_results.size());
  for (const auto* object : objects) {
    object->ToCompressedProto(prediction->add_objects());
  }
  return prediction;
}

std::vector<ObjectPrediction> BuildObjectPredictions() {
  // Two object. Stationary & dynamic. Dynamic need resolution.
  const auto obj_1 = planner::PerceptionObjectBuilder().set_id("1").Build();
  planner::ObjectPredictionBuilder builder_1;
  builder_1.set_object(obj_1)
      .add_predicted_trajectory()
      ->set_probability(0.3)
      .set_straight_line(/*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(45.0, 0.0),
                         /*init_v=*/5.0, /*last_v=*/4.0);
  builder_1.add_predicted_trajectory()->set_probability(0.7).set_straight_line(
      /*start=*/Vec2d(0.0, 0.0), /*end=*/Vec2d(50.0, 0.0),
      /*init_v=*/5.0, /*last_v=*/5.0);
  auto obj_prediction_1 = builder_1.Build();
  const auto obj_2 = planner::PerceptionObjectBuilder()
                         .set_pos(Vec2d(30.0, 0.0))
                         .set_speed(Vec2d::Zero())
                         .set_accel(Vec2d::Zero())
                         .set_id("2")
                         .Build();
  planner::ObjectPredictionBuilder builder_2;
  auto obj_prediction_2 = builder_2.set_object(obj_2).Build();

  return {std::move(obj_prediction_1), std::move(obj_prediction_2)};
}

std::map<ObjectIDType, ObjectPredictionResult>
ObjectsPredictionToObjectPredictionResultMap(
    absl::Span<const ObjectPrediction> objects_prediction) {
  std::map<ObjectIDType, ObjectPredictionResult> map;
  for (const auto& pred : objects_prediction) {
    map.emplace(pred.id(), ObjectPredictionResult{
                               .id = pred.id(),
                               .priority = ObjectPredictionPriority::OPP_P1,
                               .perception_object = pred.perception_object(),
                               .trajectories = pred.trajectories(),
                           });
  }
  return map;
}

ObjectsPredictionProto ObjectsPredictionToObjectsPredictionProto(
    absl::Span<const ObjectPrediction> objects_prediction) {
  ObjectsPredictionProto proto;
  for (const auto& obj_pred : objects_prediction) {
    obj_pred.ToProto(proto.add_objects());
  }
  return proto;
}

std::vector<ObjectPrediction> ObjectsPredictionProtoToObjectsPrediction(
    const ObjectsPredictionProto& objects_prediction_proto) {
  std::vector<ObjectPrediction> objects_prediction;
  objects_prediction.reserve(objects_prediction_proto.objects_size());

  for (const auto& obj_pred_proto : objects_prediction_proto.objects()) {
    objects_prediction.emplace_back(ObjectPrediction(obj_pred_proto));
  }
  return objects_prediction;
}

void BM_PostProcessObjectPredictionResults(benchmark::State& state) {  // NOLINT
  const auto objects_prediction = BuildObjectPredictions();
  auto result_map =
      ObjectsPredictionToObjectPredictionResultMap(objects_prediction);

  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams resolver_params;
  resolver_params.LoadParams();
  ConflictResolverDebugProto debug_proto;

  // Prediction module: resolver from object prediction result map.
  for (auto _ : state) {
    // Run in prediction module.
    const auto object_pred_pp = FromObjectPredictionResults(&result_map);
    benchmark::DoNotOptimize(RunPredictionPostProcess(
        psmm, /*red_tls=*/{}, resolver_params, object_pred_pp, &debug_proto,
        /*thread_pool=*/nullptr));
    auto objects_prediction_to_planner_or = AssemblePredictionProto(result_map);
    if (!objects_prediction_to_planner_or.ok()) continue;
    // Run in planner module.
    DecompressObjectsPredictionProto((*objects_prediction_to_planner_or).get());
    planner::BuildPlannerObjects(
        /*perception=*/nullptr, (*objects_prediction_to_planner_or).get(),
        /*align_time=*/std::nullopt, /*thread_pool=*/nullptr);
  }
}

BENCHMARK(BM_PostProcessObjectPredictionResults);

void BM_PostProcessObjectsPredictionProto(benchmark::State& state) {  // NOLINT
  const auto objects_prediction = BuildObjectPredictions();
  const auto objects_prediction_proto =
      ObjectsPredictionToObjectsPredictionProto(objects_prediction);

  const auto& psmm = planner::CreateDojoTestPSMM();
  ConflictResolverParams resolver_params;
  resolver_params.LoadParams();
  ConflictResolverDebugProto debug_proto;

  // Planner module:
  // Decompress ObjectsPredictionProto, to ObjectsPrediction, PostProcess
  // prediction result, build planner objects.

  for (auto _ : state) {
    auto objects_prediction =
        ObjectsPredictionProtoToObjectsPrediction(objects_prediction_proto);
    const auto object_pred_pp =
        FromObjectPredictions(absl::MakeSpan(objects_prediction));
    benchmark::DoNotOptimize(RunPredictionPostProcess(
        psmm, /*red_tls=*/{}, resolver_params, object_pred_pp, &debug_proto,
        /*thread_pool=*/nullptr));
    planner::BuildPlannerObjectsFromObjectPrediction(
        /*perception=*/nullptr, &objects_prediction,
        /*align_time=*/std::nullopt, /*thread_pool=*/nullptr);
  }
}

BENCHMARK(BM_PostProcessObjectsPredictionProto);

}  // namespace

}  // namespace prediction
}  // namespace qcraft

BENCHMARK_MAIN();
