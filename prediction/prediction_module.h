#ifndef ONBOARD_PREDICTION_PREDICTION_MODULE_H_
#define ONBOARD_PREDICTION_PREDICTION_MODULE_H_

#include <memory>
#include <optional>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"

#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_module.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace prediction {

class PredictionModule : public LiteModule {
 public:
  explicit PredictionModule(LiteClientBase* client);
  ~PredictionModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override {}
  const PredictionInput* prediction_input() { return prediction_input_.get(); }

 private:
  void HandlePose(std::shared_ptr<const PoseProto> pose);
  void HandleLocalizationTransform(
      std::shared_ptr<const LocalizationTransformProto>
          localization_transform_proto);
  void HandlePlannerState(
      const std::shared_ptr<const planner::PlannerStateProto>& planner_state);
  void HandleObjects(std::shared_ptr<const ObjectsProto> objects);
  void HandleTrafficLightStates(
      std::shared_ptr<const TrafficLightStatesProto> tl_states);
  void HandleOnlineSemanticMap(
      std::shared_ptr<const mapping::OnlineSemanticMapProto>
          online_semantic_map);
  void HandleRoutingManagerOutputResult(
      std::shared_ptr<const planner::RouteManagerOutputProto>
          route_manager_output);
  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy);
  // The onboard wrapper function.
  absl::Status PredictionMainLoop();

  // A helper function to update semantic map.
  absl::Status UpdateOnlineSemanticMap();
  void UpdateOnlineHDMap();

  void UpdateReceivedObjects();

  // Planner main thread pool.
  std::unique_ptr<ThreadPool> thread_pool_;

  PredictionDebugProto prediction_debug_;

  std::unique_ptr<PredictionInput> prediction_input_;

  std::optional<std::pair<std::shared_ptr<const ObjectsProto>,
                          std::shared_ptr<const LocalizationTransformProto>>>
      unprocessed_real_objects_or_
          ABSL_GUARDED_BY(unprocessed_objects_or_mutex_);
  std::optional<std::pair<std::shared_ptr<const ObjectsProto>,
                          std::shared_ptr<const LocalizationTransformProto>>>
      unprocessed_virtual_objects_or_
          ABSL_GUARDED_BY(unprocessed_objects_or_mutex_);
  mutable absl::Mutex unprocessed_objects_or_mutex_;

  VehicleParamApi vehicle_params_;
  std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex>
      hd_semantic_map_multilevel_spatial_index_;
  std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex>
      vision_semantic_map_multilevel_spatial_index_;
  std::shared_ptr<planner::PlannerSemanticMapManager>
      planner_semantic_map_manager_;
  mutable std::shared_ptr<
      mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc>
      hd_semantic_map_listener_;

  bool pending_load_hd_psmm_ = false;
  Future<std::shared_ptr<planner::PlannerSemanticMapManager>> psmm_future_;

  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map_;

  std::unique_ptr<ModelPool> prediction_model_pool_;
  ConflictResolverParams prediction_conflict_resolver_params_;

  absl::Time last_iteration_time_;

  RouteSectionSequenceProto section_seq_;
  std::optional<planner::PlannerState::HdMapState> hd_map_state_;
};

REGISTER_LITE_MODULE(PredictionModule);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_MODULE_H_
