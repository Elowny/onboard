#ifndef ONBOARD_PREDICTION_POST_PROCESS_OBJECT_CONFLICT_MANAGER_H_
#define ONBOARD_PREDICTION_POST_PROCESS_OBJECT_CONFLICT_MANAGER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/post_process/post_process_defs.h"
#include "onboard/prediction/post_process/relation_analyzer.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::prediction {
// The manager considers all given object prediction and prediction context,
// build decision constraints (traffic light, stopline, etc.) and which objects
// to consider in conflict resolution for each prediction object.

struct TrajectoryInfo {
  const planner::DiscretizedPath* path = nullptr;
  const planner::SpeedVector* ref_speed = nullptr;
  const absl::flat_hash_set<TrajectoryNodeIndex>* influencers = nullptr;
};

using TrajectoryMap =
    absl::flat_hash_map<std::string, const PredictedTrajectory*>;
using TrajectoryInfoMap = absl::flat_hash_map<std::string, TrajectoryInfo>;

class ObjectConflictManager {
 public:
  explicit ObjectConflictManager(
      const planner::PlannerSemanticMapManager& semantic_map_manager,
      const ConflictResolutionConfigProto::ConflictResolverConfig&
          resolver_config,
      const ConflictResolutionLevel resolution_level,
      const std::map<std::string, ObjectPredictionPostProcess>&
          objects_prediction_map,
      ThreadPool* thread_pool);

  absl::Status Init();
  absl::StatusOr<std::vector<std::vector<std::string>>> GetResolvingOrder()
      const;

  absl::Status ModifiedPredictions(
      const std::map<ObjectIDType, ObjectPredictionPostProcess>&
          mutable_object_predictions) const;

  // Add modified trajectory in current stage. Update both modified trajectory
  // map and original_trajectory_map_ because when resolving objects in next
  // stage will depend on trajecotry_map_ to give PreidictedTrajectory to map.
  void AddModifiedTrajectory(const std::string& traj_id,
                             PredictedTrajectory predicted_trajectory) {
    // Update both modified map and trajectory map at the same time.
    modified_trajectory_map_[traj_id] = std::move(predicted_trajectory);
  }

  absl::StatusOr<const TrajectoryInfo*> GetTrajectoryInfoByTrajId(
      const std::string& traj_id) const;
  // Query ObjectProto by TrajId.
  absl::StatusOr<const ObjectProto*> GetObjectProtoByTrajId(
      const std::string& traj_id) const;
  // Query PredictedTrajectory* by TrajId.
  absl::StatusOr<const PredictedTrajectory* const>
  GetPredictedTrajectoryByTrajId(const std::string& traj_id) const;

  inline absl::Span<const TrajectoryNode> traj_nodes() const {
    return absl::MakeSpan(traj_nodes_);
  }
  absl::Span<const std::string> object_traj_ids() const {
    return absl::MakeSpan(object_trajs_);
  }

  const planner::PlannerSemanticMapManager* semantic_map_manager() const {
    return semantic_map_manager_;
  }
  inline ConflictResolutionLevel level() const { return level_; }

  inline int object_trajs_size() const { return object_trajs_.size(); }
  std::string DebugString() const;

  // A utility function to generate trajectory's id.
  static std::string MakeTrajectoryId(const std::string& obj_id,
                                      int traj_index) {
    return absl::StrFormat("%s-idx%d", obj_id, traj_index);
  }

  // A utility function to extract object's id from trajectory id.
  static std::string GetObjectIdFromTrajectoryId(const std::string& traj_id) {
    const std::vector<std::string> tokens = absl::StrSplit(traj_id, "-");
    QCHECK(!tokens.empty());
    return tokens.front();
  }

 private:
  struct ObjectPredictionResultRef {
    const ObjectProto* object_proto = nullptr;
    std::vector<const PredictedTrajectory*> predicted_trajs;
  };

  std::vector<AgentRelationAnalyzerInput> BuildOnPathAnalyzerBatchInput(
      const TrajectoryMap& traj_map,
      const std::map<ObjectIDType, ObjectPredictionResultRef>& result);

  ThreadPool* thread_pool_ = nullptr;
  const planner::PlannerSemanticMapManager* semantic_map_manager_;
  const ConflictResolutionConfigProto::ConflictResolverConfig* general_config_;
  const ConflictResolutionLevel level_;
  std::map<ObjectIDType, ObjectPredictionResultRef> origin_result_;

  absl::flat_hash_map<std::string, const PredictedTrajectory*>
      original_trajectory_map_;

  // Results.
  absl::flat_hash_map<std::string, PredictedTrajectory>
      modified_trajectory_map_;

  // For moving trajectories that possibly need resolving conflict.
  absl::flat_hash_map<std::string, TrajectoryInfo> moving_trajs_info_map_;
  std::vector<planner::DiscretizedPath> moving_trajs_paths_;
  std::vector<planner::SpeedVector> moving_trajs_ref_speeds_;

  // Path analyzer result.
  std::vector<TrajectoryNode> traj_nodes_;
  // Vectors of trajectories that can be solved in parallel.
  std::vector<std::vector<TrajectoryNodeIndex>> nodes_priority_layers_;

  // TODO(changqing): possiblity to filter some of the trajectories that
  // won't interact with av?
  std::vector<std::string> object_trajs_;
  std::vector<std::string> moving_object_trajs_;
  std::vector<std::string> stationary_object_trajs_;

  // Constraints from prediction context.
  std::vector<mapping::ElementId> ref_tl_controlled_lanes_;
};

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_POST_PROCESS_OBJECT_CONFLICT_MANAGER_H_
