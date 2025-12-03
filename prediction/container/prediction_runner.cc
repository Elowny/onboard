#include "onboard/prediction/container/prediction_runner.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"  // for Span
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/util.h"  // for Vec2dToProto
#include "onboard/planner/router/drive_passage.h"  // for Station, StationVector, DrivePassage
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/scheduler/scheduler.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/objects_view.h"
// Used for checking prediciton state, should be removed later.
namespace qcraft::prediction {

namespace {
void OutputDrivePassageInfo(
    absl::Span<const std::unique_ptr<planner::DrivePassage>> dps,
    PredictionDebugProto* debug) {
  for (const auto& dp_ptr : dps) {
    auto* dp_proto = debug->add_dps();
    for (const auto& station : dp_ptr->stations()) {
      Vec2dToProto(station.xy(), dp_proto->add_xy());
    }
  }
}
std::shared_ptr<const ObjectsProto> GetAllObjects(
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects) {
  SCOPED_QTRACE("GetAllObjects");

  ObjectsView objects_view;
  if (real_objects != nullptr) {
    objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, real_objects);
  }
  if (virtual_objects != nullptr) {
    objects_view.UpdateObjects(ObjectsProto::SCOPE_VIRTUAL, virtual_objects);
  }
  return objects_view.ExportAllObjectsProto();
}
}  // namespace

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

absl::StatusOr<std::unique_ptr<ObjectsPredictionProto>> ComputePrediction(
    const PredictionInput& input, ModelPool* model_pool,
    ThreadPool* thread_pool, PredictionDebugProto* debug) {
  FUNC_QTRACE();
  ScopedMultiTimer prediction_timer("prediction_debug");

  const auto objects_proto =
      GetAllObjects(input.real_objects, input.virtual_objects);
  // Run motion prediction.
  VLOG(2) << "Running prediction.";
  prediction_timer.Mark("prediction start");
  // 1. Update prediction world view.
  PredictionContext prediction_context(input);
  if (FLAGS_prediction_debug_av_drive_passage &&
      prediction_context.av_drive_passage() != nullptr) {
    OutputDrivePassageInfo({std::make_unique<planner::DrivePassage>(
                               *prediction_context.av_drive_passage())},
                           debug);
  }

  // If no object is added to the prediction context cache, do not make
  // prediction.
  if (!prediction_context.HasObjects()) {
    auto prediction = std::make_unique<ObjectsPredictionProto>();
    return prediction;
  }

  // 2. Get objects that need prediction and make prediction.
  const auto objects_to_predict =
      prediction_context.GetObjectsToPredict(*objects_proto);
  if (objects_to_predict.empty()) {
    auto prediction = std::make_unique<ObjectsPredictionProto>();
    return prediction;
  }

  const double current_ts = GetCurrentTimeStamp(
      prediction_context.av_context().GetAvObjectHistory().timestamp(),
      objects_to_predict);
  // Use tracker history.
  std::unique_ptr<ObjectHistorySampler> obj_sampler_ptr =
      std::make_unique<ObjectHistorySampler>(
          objects_to_predict,
          prediction_context.av_context().GetAvObjectHistory(), current_ts,
          kFeatureV2HistoryStepLen, kFeatureV2HistoryStepNum,
          /*enable_smoothing=*/false, FLAGS_prediction_use_tracker_history);
  QCHECK_NOTNULL(obj_sampler_ptr);
  const auto object_prediction_results =
      SchedulePrediction(prediction_context, *model_pool, *obj_sampler_ptr,
                         objects_to_predict, thread_pool, debug);

  if (FLAGS_prediction_debug_drive_passage) {
    OutputDrivePassageInfo(prediction_context.drive_passages(), debug);
  }

  // 3. Assemble prediction proto.
  return AssemblePredictionProto(object_prediction_results);
}
}  // namespace qcraft::prediction
