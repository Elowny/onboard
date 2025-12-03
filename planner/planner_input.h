#ifndef ONBOARD_PLANNER_PLANNER_INPUT_H_
#define ONBOARD_PLANNER_PLANNER_INPUT_H_

#include <memory>
#include <string>
#include <utility>

#include "onboard/async/async_util.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/parking_spot_finder.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/parking/parking_freespace.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/semantic_map_modification.pb.h"
#include "onboard/utils/objects_view.h"

namespace qcraft::planner {

// The input data of planner module. The struct should be read-only after
// construction.
struct PlannerInput {
  std::shared_ptr<mapping::v2::SemanticMapManager> semantic_map_manager;
  std::shared_ptr<PlannerSemanticMapManager> planner_semantic_map_manager;
  std::shared_ptr<const mapping::v2::SemanticMapMultilevelSpatialIndex>
      semantic_map_multilevel_spatial_index;

  // The state of previous planner iteration.
  std::shared_ptr<const PlannerStateProto> planner_state_proto;
  // Deprecated in Onboard, only used for snapshot.
  PlannerDebugProto prev_planner_debug;

  PlannerParamsProto planner_params;

  // Load params for prediction post process.
  prediction::ConflictResolverParams prediction_conflict_resolver_params;

  VehicleParamApi vehicle_params;

  std::shared_ptr<const PoseProto> pose;
  // Three ObjectsProto:scope different objects
  std::shared_ptr<const ObjectsProto> real_objects;
  std::shared_ptr<const ObjectsProto> virtual_objects;
  std::shared_ptr<const ObjectsProto> av_objects;
  std::shared_ptr<const AutonomyStateProto> autonomy_state;
  std::shared_ptr<const TrafficLightStatesProto> traffic_light_states;
  std::shared_ptr<const DriverAction> driver_action;
  std::shared_ptr<const ReroutingRequestProto> rerouting_request;
  std::shared_ptr<const RemoteAssistToCarProto> remote_assist_to_car;
  std::shared_ptr<const Chassis> chassis;
  std::shared_ptr<const LocalizationTransformProto> localization_transform;
  std::shared_ptr<const RoutingResultProto> routing_result;
  std::shared_ptr<const RouteManagerOutputProto> route_mgr_output;
  std::shared_ptr<const SemanticMapModificationProto> semantic_map_modification;
  std::shared_ptr<const SensorFovsProto> sensor_fovs;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map;
  std::shared_ptr<const ParkingSpotFinderProto> parking_spot_finder;
  std::shared_ptr<const FusionParkingFreespaceProto> fusion_parking_freespace;

  // Optional in onboard as planner computes prediction inside planner module.
  // Required in offboard case as planner may not able to reproduce the same
  // prediction result.
  std::unique_ptr<const ObjectsPredictionProto> prediction;
  std::shared_ptr<const PredictionDebugProto> prediction_debug;

  // Planner Models
  std::unique_ptr<ModelPool> planner_model_pool;
  // cross-iteration
  // TODO(Jinyun): Define another avcontext class with less unnecessary
  // operations.
  std::unique_ptr<prediction::AvContext> av_context;

  // Simulation only: use oracle prediction from log
  std::unique_ptr<const ObjectsPredictionProto> log_prediction;
  std::shared_ptr<const TrajectoryProto> log_av_trajectory;

  // Process this struct before use it for next planner iteration.
  void BeforeNextIteration(ThreadPool* thread_pool) {
    // Despose of all objects asynchronously to avoid blocking main thread.
    ScheduleFuture(thread_pool,
                   [unused_1 = std::move(remote_assist_to_car)] {});
  }

  std::string DebugString() const {
    if (planner_state_proto == nullptr) {
      return "empty planner state";
    } else {
      return planner_state_proto->DebugString();
    }
  }
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_INPUT_H_
