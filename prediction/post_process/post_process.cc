#include "onboard/prediction/post_process/post_process.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/prediction/post_process/conflict_resolver.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/trajectory_modifier.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

namespace prediction {

void RunCutoffByCurbPostProcess(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const ConflictResolverParams& resolver_params,
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions,
    ThreadPool* thread_pool) {
  std::vector<const ObjectPredictionPostProcess*> prediction_results;
  prediction_results.reserve(mutable_object_predictions.size());
  for (const auto& [id, prediction_result] : mutable_object_predictions) {
    prediction_results.push_back(&prediction_result);
  }
  ParallelFor(0, prediction_results.size(), thread_pool, [&](int i) {
    auto* prediction_result = prediction_results[i];
    const auto& object_proto = *prediction_result->object_proto;
    const auto& object_config =
        resolver_params.GetConfigByObjectType(object_proto.type());
    const auto& ptrs_mutable_trajs = prediction_result->ptrs_mutable_trajs;
    for (auto* ptr_mutable_traj : ptrs_mutable_trajs) {
      CutoffTrajByCurb(semantic_map_manager, object_proto, object_config,
                       ptr_mutable_traj);
    }
  });
}

absl::Status RunPredictionPostProcess(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const std::set<mapping::ElementId>& red_tls,
    const ConflictResolverParams& resolver_params,
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions,
    ConflictResolverDebugProto* debug_proto, ThreadPool* thread_pool) {
  FUNC_QTRACE();
  // Step 1. CurbCutoff.
  RunCutoffByCurbPostProcess(semantic_map_manager, resolver_params,
                             mutable_object_predictions, thread_pool);

  // Step 2. Resolve trajectory conflicts.
  // TODO(changqing): change to in-place modification.
  ResolveConflict(semantic_map_manager, red_tls, resolver_params,
                  mutable_object_predictions, debug_proto, thread_pool);
  return absl::OkStatus();
}

std::set<mapping::ElementId> FindRedTrafficLights(
    const planner::PlannerSemanticMapManager& psmm,
    const TrafficLightStatesProto& tl_states) {
  std::set<mapping::ElementId> red_tls;
  // Find red traffic light from direct observation.
  for (const auto& tl_state : tl_states.states()) {
    const auto* tl_ptr = psmm.FindTrafficLightByIdOrNull(
        mapping::ElementId(tl_state.traffic_light_id()));
    if (tl_ptr == nullptr) continue;

    // Red lights.
    if (tl_state.color() == TrafficLightColor::TL_RED) {
      red_tls.insert(mapping::ElementId(tl_state.traffic_light_id()));
      for (const auto& id : tl_ptr->synced_tl_ids()) {
        red_tls.insert(mapping::ElementId(id));
      }
    }

    // Green/yellow lights blocking lights.
    if (tl_state.color() == TrafficLightColor::TL_GREEN ||
        tl_state.color() == TrafficLightColor::TL_YELLOW) {
      for (const auto& id : tl_ptr->blocking_tl_ids()) {
        red_tls.insert(mapping::ElementId(id));
      }
    }
  }
  return red_tls;
}

std::map<ObjectIDType, ObjectPredictionPostProcess> FromObjectPredictionResults(
    std::map<ObjectIDType, ObjectPredictionResult>* mutable_obj_pred_results) {
  std::map<ObjectIDType, ObjectPredictionPostProcess> obj_preds_post_process;
  for (auto& [id, result] : *mutable_obj_pred_results) {
    auto [it, inserted] = obj_preds_post_process.try_emplace(
        id, ObjectPredictionPostProcess{
                .object_proto = &result.perception_object,
            });
    if (inserted) {
      auto& obj_pred_pp = it->second;
      obj_pred_pp.ptrs_mutable_trajs.clear();
      obj_pred_pp.ptrs_mutable_trajs.reserve(result.trajectories.size());
      std::transform(result.trajectories.begin(), result.trajectories.end(),
                     std::back_insert_iterator(obj_pred_pp.ptrs_mutable_trajs),
                     [](auto& traj) { return &traj; });
    }
  }
  return obj_preds_post_process;
}

std::map<ObjectIDType, ObjectPredictionPostProcess> FromObjectPredictions(
    absl::Span<ObjectPrediction> mutable_obj_preds) {
  std::map<ObjectIDType, ObjectPredictionPostProcess> obj_preds_post_process;
  for (auto& obj_pred : mutable_obj_preds) {
    auto [it, inserted] = obj_preds_post_process.try_emplace(
        obj_pred.id(), ObjectPredictionPostProcess{
                           .object_proto = &obj_pred.perception_object(),
                       });
    if (inserted) {
      auto& obj_pred_pp = it->second;
      auto& trajs = *obj_pred.mutable_trajectories();
      obj_pred_pp.ptrs_mutable_trajs.clear();
      obj_pred_pp.ptrs_mutable_trajs.reserve(trajs.size());
      std::transform(trajs.begin(), trajs.end(),
                     std::back_insert_iterator(obj_pred_pp.ptrs_mutable_trajs),
                     [](auto& traj) { return &traj; });
    }
  }
  return obj_preds_post_process;
}

}  // namespace prediction
}  // namespace qcraft
