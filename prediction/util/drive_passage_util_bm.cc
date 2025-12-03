#include <algorithm>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "benchmark/benchmark.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"  // for ConflictResolverParams
#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"
#include "onboard/prediction/test_util/prediction_context_builder.h"
#include "onboard/prediction/util/drive_passage_util.h"
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

namespace qcraft {
namespace prediction {
namespace {
constexpr double kDrivePassageFrontLength = 160.0;          // m.
constexpr double kDrivePassageBackLength = 100.0;           // m.
constexpr double kDrivePassageBackwardExtendLength = 80.0;  // m.
constexpr double kUpdateTimeStep = 0.1;
constexpr int kNumHistory = 10;

static void BM_BuildAvDrivePassageWithRouting(
    benchmark::State& state) {  // NOLINT
  std::vector<mapping::SectionId> section_ids(
      {mapping::SectionId(12401), mapping::SectionId(12400),
       mapping::SectionId(12408), mapping::SectionId(12412),
       mapping::SectionId(12414)});
  planner::RouteSections route_sections = planner::RouteSections(
      0.0, 1.0, section_ids, mapping::LanePoint(mapping::ElementId(53), 1.0));
  const auto& psmm = planner::CreateDojoTestPSMM();
  planner::LaneBoundaryCache cache;
  // warm up
  [[maybe_unused]] const auto res1 = BuildAvDrivePassageWithRouting(
      psmm, cache, route_sections, Vec2d(0.0, 0.0),
      kDrivePassageBackwardExtendLength, kDrivePassageFrontLength,
      kDrivePassageBackLength);
  [[maybe_unused]] const auto res2 = BuildAvDrivePassageWithRouting(
      psmm, cache, route_sections, Vec2d(0.0, 0.0),
      kDrivePassageBackwardExtendLength, kDrivePassageFrontLength,
      kDrivePassageBackLength);

  for (auto _ : state) {
    benchmark::DoNotOptimize(BuildAvDrivePassageWithRouting(
        psmm, cache, route_sections, Vec2d(0.0, 0.0),
        kDrivePassageBackwardExtendLength, kDrivePassageFrontLength,
        kDrivePassageBackLength));
  }
}

static void BM_BuildLaneSelectionObjectDrivePassages(
    benchmark::State& state) {  // NOLINT

  const auto& psmm = planner::CreateDojoTestPSMM();
  VehicleGeometryParamsProto vparams;
  ConflictResolverParams cparams;
  auto input = BuildPredictionInput(psmm, vparams, cparams);
  const auto context =
      BuildMultiObjectsPredictionContext(/*obj_num=*/2, &input);
  std::vector<const ObjectHistory*> objects_history;
  objects_history.reserve(2);
  for (int i = 0; i < 2; ++i) {
    const auto& hist = context.object_history_manager().at(std::to_string(i));
    objects_history.push_back(&hist);
  }

  const auto obj_sampler = ObjectHistorySampler(
      objects_history, *objects_history[1], kUpdateTimeStep * (kNumHistory - 1),
      kUpdateTimeStep, kNumHistory,
      /*enable_smoothing=*/false,
      /*use_tracker_history=*/false);

  // warm up
  [[maybe_unused]] const auto res1 = BuildLaneSelectionObjectDrivePassages(
      obj_sampler.GetResampledMotionHistoryById(objects_history[0]->id()),
      context);
  [[maybe_unused]] const auto res2 = BuildLaneSelectionObjectDrivePassages(
      obj_sampler.GetResampledMotionHistoryById(objects_history[0]->id()),
      context);

  for (auto _ : state) {
    benchmark::DoNotOptimize(BuildLaneSelectionObjectDrivePassages(
        obj_sampler.GetResampledMotionHistoryById(objects_history[0]->id()),
        context));
  }
}

BENCHMARK(BM_BuildAvDrivePassageWithRouting);
BENCHMARK(BM_BuildLaneSelectionObjectDrivePassages);

}  // namespace
}  // namespace prediction
}  // namespace qcraft
BENCHMARK_MAIN();
