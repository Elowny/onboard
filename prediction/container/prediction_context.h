
#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_CONTEXT_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_CONTEXT_H_
// IWYU pragma: no_include "absl/hash/hash.h"

#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"  // IWYU pragma: keep
#include "absl/time/time.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
using DrivePassageCache =
    absl::flat_hash_map<mapping::ElementId,
                        std::vector<const planner::DrivePassage*>>;
// TODO(xiang): this class should be removed totally.
class PredictionContext {
 public:
  PredictionContext() = delete;
  explicit PredictionContext(const PredictionInput& input);

  const ObjectsHistory& object_history_manager() const {
    return *objects_history_;
  }

  const ObjectsHistory& object_long_term_history_manager() const {
    return *obj_long_term_hist_mgr_;
  }

  const AvContext& av_context() const { return *av_context_; }

  const VehicleGeometryParamsProto& vehicle_geometry_params() const {
    return *veh_geom_params_;
  }

  const planner::PlannerSemanticMapManager& semantic_map_manager() const {
    return *semantic_map_manager_;
  }

  const ConflictResolverParams& conflict_resolver_params() const {
    return *conflict_resolver_params_;
  }

  const TrafficLightManager& traffic_light_manager() const {
    return traffic_light_manager_;
  }

  const planner::LaneBoundaryCache& lane_boundary_cache() const {
    return lane_boundary_cache_;
  }

  const planner::DrivePassage* av_drive_passage() const {
    return av_drive_passage_.get();
  }

  absl::Time PredictionInitTime() const { return prediction_init_time_; }

  const AutonomyStateProto* autonomy_state() const { return autonomy_state_; }

  const planner::RouteManagerOutputProto* route_manager_output() const {
    return route_manager_output_;
  }

  bool HasObjects() const { return !objects_history_->empty(); }

  std::vector<const ObjectHistory*> GetObjectsToPredict(
      const ObjectsProto& objects_proto) const;

  const std::vector<std::unique_ptr<planner::DrivePassage>>& drive_passages()
      const {
    return drive_passages_;
  }

  const DrivePassageCache& drive_passage_cache() const {
    return drive_passage_cache_;
  }

  const std::vector<const planner::DrivePassage*>* drive_passages_by_id(
      mapping::ElementId id) const {
    return FindOrNull(drive_passage_cache_, id);
  }

 private:
  absl::Time prediction_init_time_;
  // The following member variables must have value.
  const ObjectsHistory* objects_history_ = nullptr;
  const ObjectsHistory* obj_long_term_hist_mgr_ = nullptr;
  const AvContext* av_context_ = nullptr;
  const VehicleGeometryParamsProto* veh_geom_params_ = nullptr;
  const planner::PlannerSemanticMapManager* semantic_map_manager_ = nullptr;
  const ConflictResolverParams* conflict_resolver_params_ = nullptr;
  // The following member variables can be nullptr
  const AutonomyStateProto* autonomy_state_ = nullptr;  // can be nullptr.
  const planner::RouteManagerOutputProto* route_manager_output_ =
      nullptr;                                               // can be nullptr
  std::unique_ptr<planner::DrivePassage> av_drive_passage_;  // can be nullptr.
  TrafficLightManager traffic_light_manager_;
  planner::LaneBoundaryCache lane_boundary_cache_;
  DrivePassageCache drive_passage_cache_;
  std::vector<std::unique_ptr<planner::DrivePassage>> drive_passages_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_CONTEXT_H_
