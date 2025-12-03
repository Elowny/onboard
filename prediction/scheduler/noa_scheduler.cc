#include "onboard/prediction/scheduler/noa_scheduler.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <vector>

#include "absl/types/span.h"

#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/post_process.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/act_net_j5_predictor.h"
#include "onboard/prediction/predictor/lane_selection_net_j5_predictor.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/scheduler/priority_analyzer.h"
#include "onboard/prediction/scheduler/scenario_analyzer.h"
#include "onboard/prediction/scheduler/scheduler_utils.h"
#include "onboard/proto/assist_driving_system_state.pb.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
namespace {

void PredictObjectsByModel(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriorityInfo>&
        object_priorities,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs) {
  SCOPED_QTRACE("PredictObjectsByModel");
  std::vector<const ObjectHistory*> filtered_by_priority_objs;
  filtered_by_priority_objs.reserve(objs_to_predict.size());
  for (const auto* obj : objs_to_predict) {
    const auto& priority = object_priorities.at(obj->id()).priority;
    if (priority != OPP_P3 && priority != OPP_P2 &&
        (!obj->GetHistory().back().val.IsReversed())) {
      filtered_by_priority_objs.push_back(obj);
    }
  }
  if (FLAGS_prediction_enable_act_net_j5) {
    // 1.1 Do ActNetJ5 Net prediction.
    auto actnet_out_map = MakeActNetJ5Prediction(
        prediction_context, filtered_by_priority_objs,
        *model_pool.GetActNetJ5Inferencer(),
        model_pool.GetCutinSLNetJ5Inferencer(), obj_sampler, thread_pool);
    for (const auto& [id, actnet_out] : actnet_out_map) {
      TryInsertTrajectories(object_trajs, actnet_out.pred_trajs, id);
    }
  } else if (FLAGS_prediction_replace_act_net_j5_with_local) {
    // 1.2 Do ActNetLocalJ5 Net prediction.
    auto actnet_out_map = MakeActNetLocalJ5Prediction(
        prediction_context, filtered_by_priority_objs,
        *model_pool.GetActNetLocalJ5Inferencer(),
        model_pool.GetCutinSLNetJ5Inferencer(), obj_sampler, thread_pool);
    for (const auto& [id, actnet_out] : actnet_out_map) {
      TryInsertTrajectories(object_trajs, actnet_out.pred_trajs, id);
    }
  } else if (FLAGS_prediction_enable_lane_selection_net_j5) {
    // 1.3 Do LaneSelectionNet prediction.
    auto lane_selection_net_out_map = MakeLaneSelectionNetJ5Prediction(
        prediction_context, filtered_by_priority_objs,
        *model_pool.GetLaneSelectionNetJ5Inferencer(), object_scenarios,
        obj_sampler, thread_pool);
    for (const auto& [id, lane_selection_out] : lane_selection_net_out_map) {
      if (lane_selection_out.pred_trajs.empty()) {
        continue;
      }
      TryInsertTrajectories(object_trajs, lane_selection_out.pred_trajs, id);
    }
  }
}

std::map<ObjectIDType, ObjectPredictionResult>
ScheduleNoaPredictionUsingPerceptionMap(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();
  ScopedMultiTimer timer("ScheduleNoaPredictionUsingPerceptionMap");

  const auto object_scenarios =
      AnalyzeScenarios(prediction_context, objs_to_predict);
  const auto object_priorities =
      AnalyzePriorities(prediction_context, objs_to_predict, object_scenarios);
  timer.Mark(
      "ScheduleNoaPredictionUsingPerceptionMap: analyze scenarios & "
      "priorities");

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  // 0. Assign void prediction to P3 object directly.
  PredictIgnoredObjects(objs_to_predict, obj_sampler, object_priorities,
                        &object_trajs);

  // 1. Filter out stationary & reverse driving and
  // assign them stationary/reverse predictions.
  PredictStationaryAndReversed(objs_to_predict, obj_sampler, &object_trajs,
                               object_scenarios,
                               FLAGS_prediction_enable_ignore_off_road_object);
  timer.Mark(
      "ScheduleNoaPredictionUsingPerceptionMap:stationary & reverse "
      "prediction");

  if (FLAGS_prediction_enable_lane_selection_net_j5) {
    // use lane selection net to help lane follow
    PredictObjectsByLaneSelection(
        prediction_context, objs_to_predict, object_scenarios, obj_sampler,
        thread_pool, &object_trajs, model_pool,
        FLAGS_prediction_enable_ignore_off_road_object);
    timer.Mark("ScheduleNoaPredictionUsingPerceptionMap: heuristic prediction");
  }

  // 3. use heuristic predictors for all non-predicted objs.
  PredictObjectsByHeuristic(prediction_context, objs_to_predict,
                            object_scenarios, obj_sampler, thread_pool,
                            &object_trajs,
                            FLAGS_prediction_enable_ignore_off_road_object);
  timer.Mark("ScheduleNoaPredictionUsingPerceptionMap: heuristic prediction");

  std::map<ObjectIDType, ObjectPredictionResult> results;
  AssemblePredictionResult(prediction_context, objs_to_predict, obj_sampler,
                           object_scenarios, object_priorities, &object_trajs,
                           &results);

  timer.Mark("ScheduleNoaPredictionUsingPerceptionMap: results");
  QCHECK_EQ(results.size(), objs_to_predict.size())
      << "Prediction input object size does not match output size.";
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return results;
}

std::map<ObjectIDType, ObjectPredictionResult> ScheduleNoaPredictionUsingHdMap(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();
  ScopedMultiTimer timer("ScheduleNoaPredictionUsingHdMap");

  const auto object_scenarios =
      AnalyzeScenarios(prediction_context, objs_to_predict);
  const auto object_priorities =
      AnalyzePriorities(prediction_context, objs_to_predict, object_scenarios);
  timer.Mark(
      "ScheduleNoaPredictionUsingHdMap: analyze scenarios & "
      "priorities");

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  // 0. Assign void prediction to P3 object directly.
  PredictIgnoredObjects(objs_to_predict, obj_sampler, object_priorities,
                        &object_trajs);

  // 1. Filter out stationary & reverse driving and
  // assign them stationary/reverse predictions.
  PredictStationaryAndReversed(objs_to_predict, obj_sampler, &object_trajs,
                               object_scenarios,
                               /*ignore_off_road=*/false);
  timer.Mark(
      "ScheduleNoaPredictionUsingHdMap:stationary & reverse "
      "prediction");

  // 2. We use agent-centric models only for objects driving forward & with
  // priority >= OPP_P2.
  PredictObjectsByModel(prediction_context, model_pool, objs_to_predict,
                        object_scenarios, object_priorities, obj_sampler,
                        thread_pool, &object_trajs);
  timer.Mark("ScheduleNoaPredictionUsingHdMap: Wait and get prediction");

  // 3. Final stage of prediction, use heuristic predictors for all
  // non-predicted objs.
  PredictObjectsByHeuristic(prediction_context, objs_to_predict,
                            object_scenarios, obj_sampler, thread_pool,
                            &object_trajs, /*ignore_off_road=*/false);
  timer.Mark("ScheduleNoaPredictionUsingHdMap: heuristic prediction");

  std::map<ObjectIDType, ObjectPredictionResult> results;
  AssemblePredictionResult(prediction_context, objs_to_predict, obj_sampler,
                           object_scenarios, object_priorities, &object_trajs,
                           &results);

  timer.Mark("ScheduleNoaPredictionUsingHdMap: results");
  QCHECK_EQ(results.size(), objs_to_predict.size())
      << "Prediction input object size does not match output size.";
  const auto obj_pred_pp = FromObjectPredictionResults(&results);
  RunCutoffByCurbPostProcess(prediction_context.semantic_map_manager(),
                             prediction_context.conflict_resolver_params(),
                             obj_pred_pp, thread_pool);
  timer.Mark("ScheduleNoaPredictionUsingHdMap: Cut off post processing");
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return results;
}

std::map<ObjectIDType, ObjectPredictionResult> ScheduleNoaPredictionUsingNoaMap(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool) {
  // temporary use hdmap scheduler
  return ScheduleNoaPredictionUsingHdMap(prediction_context, model_pool,
                                         obj_sampler, objs_to_predict,
                                         thread_pool);
}

std::map<ObjectIDType, ObjectPredictionResult> ScheduleParkingPrediction(
    const PredictionContext& prediction_context,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  PredictUnderParkingScenario(objs_to_predict, obj_sampler, thread_pool,
                              &object_trajs);
  std::map<ObjectIDType, ObjectPredictionScenario> object_scenarios;
  std::map<ObjectIDType, ObjectPredictionPriorityInfo> object_priorities;

  ObjectPredictionScenario scenario;
  scenario.set_abs_dist_to_nearest_lane(std::numeric_limits<double>::max());
  scenario.set_abs_dist_to_nearest_intersection(
      std::numeric_limits<double>::max());
  scenario.set_road_status(ObjectRoadStatus::ORS_NONE);

  const auto prio_info = ObjectPredictionPriorityInfo{
      .priority = OPP_P1,
      .priority_annotation = "Normal",
  };

  for (const auto& obj : objs_to_predict) {
    object_scenarios[obj->id()] = scenario;
    object_priorities[obj->id()] = prio_info;
  }

  std::map<ObjectIDType, ObjectPredictionResult> results;
  AssemblePredictionResult(prediction_context, objs_to_predict, obj_sampler,
                           object_scenarios, object_priorities, &object_trajs,
                           &results);
  return results;
}

bool SwitchToParkingPrediction(const AutonomyStateProto& autonomy_state) {
  if (!autonomy_state.has_assist_state()) {
    return false;
  }
  if (!autonomy_state.assist_state().has_assist_apa_state()) {
    return false;
  }
  const auto apa_state =
      autonomy_state.assist_state().assist_apa_state().state();
  if (apa_state == AssistApaStateProto::APA_STATE_PARKING_ACTIVE_ON ||
      apa_state == AssistApaStateProto::APA_STATE_PARKING_ACTIVE_PAUSE) {
    return true;
  }

  if (apa_state == AssistApaStateProto::APA_STATE_PARKING_OUT_ACTIVE_ON ||
      apa_state == AssistApaStateProto::APA_STATE_PARKING_OUT_ACTIVE_PAUSE) {
    return true;
  }
  return false;
}

}  // namespace

std::map<ObjectIDType, ObjectPredictionResult> ScheduleNoaPrediction(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool) {
  // switch noa scheduler by assist_drive_system_state
  const auto& autonomy_state = prediction_context.autonomy_state();
  if (autonomy_state != nullptr) {
    if (SwitchToParkingPrediction(*autonomy_state)) {
      return ScheduleParkingPrediction(prediction_context, obj_sampler,
                                       objs_to_predict, thread_pool);
    }

    if (MustReceiveHDMapForPrediction(*autonomy_state)) {
      return ScheduleNoaPredictionUsingHdMap(prediction_context, model_pool,
                                             obj_sampler, objs_to_predict,
                                             thread_pool);
    } else {
      return ScheduleNoaPredictionUsingPerceptionMap(
          prediction_context, model_pool, obj_sampler, objs_to_predict,
          thread_pool);
    }
  }

  if (FLAGS_prediction_enable_debug_no_map ||
      FLAGS_prediction_enable_debug_perception_map) {
    return ScheduleNoaPredictionUsingPerceptionMap(
        prediction_context, model_pool, obj_sampler, objs_to_predict,
        thread_pool);
  } else if (FLAGS_prediction_enable_debug_noa_map) {
    return ScheduleNoaPredictionUsingNoaMap(prediction_context, model_pool,
                                            obj_sampler, objs_to_predict,
                                            thread_pool);
  } else {
    return ScheduleNoaPredictionUsingHdMap(prediction_context, model_pool,
                                           obj_sampler, objs_to_predict,
                                           thread_pool);
  }
}

}  // namespace prediction
}  // namespace qcraft
