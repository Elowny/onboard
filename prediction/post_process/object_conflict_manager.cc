#include "onboard/prediction/post_process/object_conflict_manager.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <string>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/prediction/post_process/conflict_resolver_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::prediction {

namespace {
bool IgnoredObjectType(const ObjectType& type) {
  switch (type) {
    // Only consider vehicle conflict now.
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
      return false;
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_UNKNOWN_MOVABLE:
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return true;
  }
}

absl::Status ParallelBuildMovingTrajectoryInfoMap(
    absl::Span<const std::string> moving_trajs, const TrajectoryMap& traj_map,
    TrajectoryInfoMap* ptr_info_map,
    std::vector<planner::DiscretizedPath>* ptr_paths,
    std::vector<planner::SpeedVector>* ptr_ref_speeds,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();
  std::vector<absl::string_view> valid_indices;
  std::vector<const PredictedTrajectory*> valid_trajs;
  const int mov_traj_size = moving_trajs.size();
  valid_indices.reserve(mov_traj_size);
  valid_trajs.reserve(mov_traj_size);
  for (const auto& traj_idx : moving_trajs) {
    const auto* it = FindOrNull(traj_map, traj_idx);
    if (it == nullptr) {
      return absl::InternalError(absl::StrFormat(
          "Cannot find predicted trajectory for traj idx: %s.", traj_idx));
    }
    if ((*it)->points().empty()) {
      QEVENT("changqing", "prediction_predicted_trajectory_empty_points",
             [&traj_idx](QEvent* qevent) {
               qevent->AddField("traj_idx", traj_idx);
             });
      continue;
    }
    valid_trajs.push_back(*it);
    valid_indices.push_back(traj_idx);
  }

  // Prepare containers.
  const int valid_size = valid_trajs.size();
  auto& ref_speeds = *ptr_ref_speeds;
  auto& paths = *ptr_paths;
  ref_speeds.resize(valid_size);
  paths.resize(valid_size);

  ParallelFor(0, valid_size, thread_pool, [&](int i) {
    // First: path, second: speed vector.
    auto pair = PredictedTrajectoryToPurePathAndSpeedVector(*valid_trajs[i]);
    paths[i] = std::move(pair.first);
    ref_speeds[i] = std::move(pair.second);
  });

  for (int i = 0; i < valid_size; ++i) {
    auto [it, success] = ptr_info_map->try_emplace(
        valid_indices[i], TrajectoryInfo({
                              .path = &paths[i],
                              .ref_speed = &ref_speeds[i],
                          }));
    if (!success) {
      return absl::InternalError(absl::StrFormat(
          "Inserting trajectory info fails of %s.", valid_indices[i]));
    }
  }

  return absl::OkStatus();
}

void CollectInfluencersToTrajectoryInfoMap(
    const std::vector<TrajectoryNode>& trajs, TrajectoryInfoMap* ptr_info_map) {
  FUNC_QTRACE();
  auto& info_map = *ptr_info_map;
  for (const auto& traj : trajs) {
    auto* it = FindOrNull(info_map, traj.traj_id);
    if (it == nullptr) {
      // traj is stationary, so it's not recorded in info map.
      continue;
    }
    auto& info = *it;
    info.influencers = &traj.influencers;
  }
}
}  // namespace

ObjectConflictManager::ObjectConflictManager(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const ConflictResolutionConfigProto::ConflictResolverConfig&
        resolver_config,
    const ConflictResolutionLevel resolution_level,
    const std::map<std::string, ObjectPredictionPostProcess>&
        objects_prediction_map,
    ThreadPool* thread_pool)
    : thread_pool_(thread_pool),
      semantic_map_manager_(&semantic_map_manager),
      general_config_(&resolver_config),
      level_(resolution_level) {
  SCOPED_QTRACE("BuildObjectConflictManager");
  // Collect all trajectory level infos (object id + traj idx).
  object_trajs_.reserve(objects_prediction_map.size());
  for (const auto& [id, obj_pred] : objects_prediction_map) {
    ObjectPredictionResultRef obj_pred_result;
    obj_pred_result.object_proto = obj_pred.object_proto;
    for (const auto* traj : obj_pred.ptrs_mutable_trajs) {
      if (traj->type() == PT_VOID) {
        continue;
      }
      obj_pred_result.predicted_trajs.push_back(traj);
      const auto traj_id = MakeTrajectoryId(id, traj->index());
      auto [it, success] =
          original_trajectory_map_.emplace(std::make_pair(traj_id, traj));
      if (!success) {
        continue;
      } else {
        object_trajs_.push_back(traj_id);
        if (traj->type() != PT_STATIONARY) {
          moving_object_trajs_.push_back(traj_id);
        } else {
          stationary_object_trajs_.push_back(traj_id);
        }
      }
    }
    origin_result_.emplace(std::make_pair(id, std::move(obj_pred_result)));
  }
  QCHECK_EQ(object_trajs_.size(), original_trajectory_map_.size());
}

absl::Status ObjectConflictManager::Init() {
  FUNC_QTRACE();
  RETURN_IF_ERROR(ParallelBuildMovingTrajectoryInfoMap(
      absl::MakeSpan(moving_object_trajs_), original_trajectory_map_,
      &moving_trajs_info_map_, &moving_trajs_paths_, &moving_trajs_ref_speeds_,
      thread_pool_));
  // Use on path analyzer.
  std::vector<AgentRelationAnalyzerInput> inputs =
      BuildOnPathAnalyzerBatchInput(original_trajectory_map_, origin_result_);
  traj_nodes_ = BatchOnPathAnalyze(absl::MakeSpan(inputs));
  nodes_priority_layers_ = BuildPriorityGraph(&traj_nodes_);
  // Fill in trajectory infos according to priority graph and analyzer result.
  CollectInfluencersToTrajectoryInfoMap(traj_nodes_, &moving_trajs_info_map_);
  return absl::OkStatus();
}

absl::StatusOr<std::vector<std::vector<std::string>>>
ObjectConflictManager::GetResolvingOrder() const {
  FUNC_QTRACE();
  std::vector<std::vector<std::string>> order;
  order.reserve(nodes_priority_layers_.size());
  for (int i = 0; i < nodes_priority_layers_.size(); ++i) {
    // Return the first layer to resolve their red traffic light conflict.
    const auto& layer = nodes_priority_layers_[i];
    std::vector<std::string> cur_layer;
    cur_layer.reserve(layer.size());
    for (const auto traj_node_idx : layer) {
      cur_layer.push_back(traj_nodes_[traj_node_idx.value()].traj_id);
    }
    VLOG(1) << absl::StrFormat("Stage %d: solve %s.", i,
                               absl::StrJoin(cur_layer, ","));
    order.push_back(std::move(cur_layer));
  }
  return order;
}

absl::StatusOr<const ObjectProto*>
ObjectConflictManager::GetObjectProtoByTrajId(
    const std::string& traj_id) const {
  const auto* it =
      FindOrNull(origin_result_, GetObjectIdFromTrajectoryId(traj_id));
  if (it == nullptr) {
    return absl::InternalError(
        "Cannot find object from prediction raw results! Should not come "
        "here.");
  }
  return it->object_proto;
}

absl::StatusOr<const TrajectoryInfo*>
ObjectConflictManager::GetTrajectoryInfoByTrajId(
    const std::string& traj_id) const {
  const auto* it = FindOrNull(moving_trajs_info_map_, traj_id);
  if (it == nullptr) {
    return absl::InternalError(
        absl::StrFormat("Cannot find trajectory info for %s.", traj_id));
  }
  return it;
}

absl::StatusOr<const PredictedTrajectory* const>
ObjectConflictManager::GetPredictedTrajectoryByTrajId(
    const std::string& traj_id) const {
  const auto* it_modified = FindOrNull(modified_trajectory_map_, traj_id);
  if (it_modified != nullptr) {
    return it_modified;
  }
  const auto* it = FindOrNull(original_trajectory_map_, traj_id);
  if (it == nullptr) {
    return absl::InternalError(
        "Cannot find trajectory from trajectory id. Should not come here!");
  }
  return *it;
}

absl::Status ObjectConflictManager::ModifiedPredictions(
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions) const {
  FUNC_QTRACE();
  for (const auto& [traj_id, modified_trajectory] : modified_trajectory_map_) {
    const auto object_id = GetObjectIdFromTrajectoryId(traj_id);
    const auto* mutable_object_result =
        FindOrNull(mutable_object_predictions, object_id);
    if (mutable_object_result == nullptr) {
      // Continue adding other modified trajectories.
      continue;
    }
    const auto& ptrs_mutable_trajs = mutable_object_result->ptrs_mutable_trajs;
    for (auto* origin_trajectory : ptrs_mutable_trajs) {
      if (origin_trajectory->index() == modified_trajectory.index()) {
        // Trajectories vector's indexes not necessarily correspond to index()
        // of PredictedTrajectory. Cannot use vector idx to access directly.
        *origin_trajectory = modified_trajectory;
        break;
      }
    }
  }
  return absl::OkStatus();
}

std::vector<AgentRelationAnalyzerInput>
ObjectConflictManager::BuildOnPathAnalyzerBatchInput(
    const TrajectoryMap& traj_map,
    const std::map<ObjectIDType, ObjectPredictionResultRef>& result) {
  std::vector<AgentRelationAnalyzerInput> inputs;
  for (const auto& pair : traj_map) {
    const auto object_id =
        ObjectConflictManager::GetObjectIdFromTrajectoryId(pair.first);
    const auto* object_result = FindOrNull(result, object_id);
    if (object_result == nullptr) continue;
    const auto& object_proto = *object_result->object_proto;
    if (IgnoredObjectType(object_proto.type())) {
      continue;
    }
    inputs.push_back(AgentRelationAnalyzerInput({
        .traj_id = pair.first,
        .object_proto = object_result->object_proto,
        .trajectory = pair.second,
        .segments = nullptr,
    }));
  }
  return inputs;
}
}  // namespace qcraft::prediction
