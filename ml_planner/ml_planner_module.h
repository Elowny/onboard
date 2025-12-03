#ifndef ONBOARD_ML_PLANNER_ML_PLANNER_MODULE_H_
#define ONBOARD_ML_PLANNER_ML_PLANNER_MODULE_H_

#include <memory>
#include <optional>
// #include <queue>
// #include <utility>

#include "absl/status/status.h"

#include "onboard/async/future.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/route.pb.h"
// #include "absl/time/time.h"

// #include "onboard/async/future.h"
// #include "onboard/async/thread_pool.h"
#include "onboard/lite/lite_client_base.h"
#include "onboard/lite/lite_module.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_listener.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/ml_planner/ml_planner_input.h"
#include "onboard/ml_planner/proto/ml_planner_debug.pb.h"
#include "onboard/ml_planner/proto/ml_planner_output.pb.h"
#include "onboard/planner/planner_state.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace mlplanner {

class MlPlannerModule : public LiteModule {
 public:
  explicit MlPlannerModule(LiteClientBase* client);
  ~MlPlannerModule();

  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override {}

 private:
  void HandlePose(std::shared_ptr<const PoseProto> pose);
  void HandlePrediction(
      const std::shared_ptr<const ObjectsPredictionProto>& prediction);
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
  void HandleAutonomyState(std::shared_ptr<const AutonomyStateProto> autonomy);

  // A helper function to update semantic map.
  absl::Status UpdateOnlineSemanticMap();
  void UpdateOnlineHDMap();

  // The onboard wrapper function.
  absl::Status MLPlannerMainLoop();

  // Contains all the input data to start an iteration of ml planner input.
  std::unique_ptr<MLPlannerInput> ml_planner_input_;

  // Contains the output data of ml planner module. It should include published
  // proto messages, produced events, issues, canvas etc.
  std::unique_ptr<MlPlannerOutputProto> ml_planner_output_;

  //   // Planner main thread pool.
  //   std::unique_ptr<ThreadPool> thread_pool_;

  std::unique_ptr<MlPlannerDebugProto> ml_planner_debug_;

  // PlannerState planner_state_;

  // Map related
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

  RouteSectionSequenceProto section_seq_;
  std::optional<planner::PlannerState::HdMapState> hd_map_state_;
};

REGISTER_LITE_MODULE(MlPlannerModule);

}  // namespace mlplanner
}  // namespace qcraft

#endif  // ONBOARD_ML_PLANNER_ML_PLANNER_MODULE_H_
