
#include "onboard/prediction/scheduler/fsd_scheduler.h"

#include <algorithm>
#include <optional>
#include <ostream>
#include <set>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"

#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/post_process/post_process.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/act_net_predictor.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/scheduler/priority_analyzer.h"
#include "onboard/prediction/scheduler/scenario_analyzer.h"
#include "onboard/prediction/scheduler/scheduler_utils.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/elements_history.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace {

inline std::set<mapping::ElementId> GetAllRedLightIds(
    const absl::flat_hash_map<mapping::ElementId, TrafficLightStateProto>&
        inferred_tl_state_map) {
  std::set<mapping::ElementId> red_tls;
  for (const auto& [id, state] : inferred_tl_state_map) {
    if (state.has_color() && state.color() == TL_RED) {
      red_tls.insert(id);
    }
  }
  return red_tls;
}

void PredictObjectsByModel(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionPriorityInfo>&
        object_priorities,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    ObjectsActNetPredMap* actnet_out_map,
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
  if (FLAGS_prediction_enable_act_net) {
    // 1. Do Act Net prediction.
    *actnet_out_map = MakeActNetPrediction(
        prediction_context, filtered_by_priority_objs,
        model_pool.GetActNetInferencer(), model_pool.GetCutinSLNetInferencer(),
        obj_sampler, thread_pool);
    for (const auto& [id, actnet_out] : *actnet_out_map) {
      TryInsertTrajectories(object_trajs, actnet_out.pred_trajs, id);
    }
  }
}

void AssignStartupProbs(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectsActNetPredMap& actnet_out_map,
    std::map<ObjectIDType, ObjectPredictionResult>* results) {
  FUNC_QTRACE();
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto hist = objs_to_predict[i]->GetHistory();
    const auto& id = hist.id();
    if (hist.back().val.IsStationary() &&
        (GuessType(*objs_to_predict[i]) == OT_VEHICLE ||
         GuessType(*objs_to_predict[i]) == OT_LARGE_VEHICLE)) {
      auto* actnet_out = FindOrNull(actnet_out_map, id);
      if (actnet_out && (*actnet_out).startup_prob.has_value()) {
        const double startup_prob = *(*actnet_out).startup_prob;
        (*results)[id].startup_trajs = FilterOutStaticAndLowProbTrajectory(
            (*actnet_out).pred_trajs, startup_prob);
        // If we predict at least one start-up trajectory, assign a
        // start-up probability.
        if (!(*results)[id].startup_trajs.empty()) {
          (*results)[id].startup_prob = startup_prob;
        }
      }
    }
  }
}

}  // namespace

std::map<ObjectIDType, ObjectPredictionResult> ScheduleFsdPrediction(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool, PredictionDebugProto* debug) {
  FUNC_QTRACE();
  ScopedMultiTimer timer("PredictionScheduler::ScheduleFsdPrediction");

  const auto object_scenarios =
      AnalyzeScenarios(prediction_context, objs_to_predict);
  const auto object_priorities =
      AnalyzePriorities(prediction_context, objs_to_predict, object_scenarios);
  timer.Mark(
      "PredictionScheduler::ScheduleFsdPrediction: analyze scenarios & "
      "priorities");

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;
  ObjectsActNetPredMap actnet_out_map;
  // 1. Filter out stationary & reverse driving and
  // assign them stationary/reverse predictions.
  PredictStationaryAndReversed(objs_to_predict, obj_sampler, &object_trajs,
                               object_scenarios,
                               /*ignore_off_road=*/false);
  timer.Mark(
      "PredictionScheduler::ScheduleFsdPrediction:stationary & reverse "
      "prediction");

  // 2. We use agent-centric models only for objects driving forward & with
  // priority >= OPP_P2.
  PredictObjectsByModel(prediction_context, model_pool, objs_to_predict,
                        object_priorities, obj_sampler, thread_pool,
                        &actnet_out_map, &object_trajs);
  timer.Mark(
      "PredictionScheduler::ScheduleFsdPrediction: Wait and get prediction");

  // 3. Final stage of prediction, use heuristic predictors for all
  // non-predicted objs.
  PredictObjectsByHeuristic(prediction_context, objs_to_predict,
                            object_scenarios, obj_sampler, thread_pool,
                            &object_trajs, /*ignore_off_road=*/false);
  timer.Mark(
      "PredictionScheduler::ScheduleFsdPrediction: heuristic prediction");

  std::map<ObjectIDType, ObjectPredictionResult> results;
  AssemblePredictionResult(prediction_context, objs_to_predict, obj_sampler,
                           object_scenarios, object_priorities, &object_trajs,
                           &results);
  AssignStartupProbs(objs_to_predict, actnet_out_map, &results);

  timer.Mark("PredictionScheduler::ScheduleFsdPrediction: results");
  QCHECK_EQ(results.size(), objs_to_predict.size())
      << "Prediction input object size does not match output size.";
  if (!FLAGS_prediction_run_post_process_in_planner) {
    const auto& inferred_tl_states =
        prediction_context.traffic_light_manager().GetInferedTlStateMap();
    const auto red_tls = GetAllRedLightIds(inferred_tl_states);
    const auto obj_preds_post_process = FromObjectPredictionResults(&results);
    const auto post_process_status = RunPredictionPostProcess(
        prediction_context.semantic_map_manager(), red_tls,
        prediction_context.conflict_resolver_params(), obj_preds_post_process,
        debug->mutable_conflict_resolver_debug(), thread_pool);
  }
  timer.Mark("PredictionScheduler::ScheduleFsdPrediction: Post processing");
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  return results;
}
}  // namespace prediction
}  // namespace qcraft
