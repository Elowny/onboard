#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_INPUT_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_INPUT_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {

struct MultiTasksCruisePlannerInput {
  // Meant to add one count to shared_ptr here for async planner.
  const CoordinateConverter* coordinate_converter;

  std::shared_ptr<PlannerSemanticMapManager> planner_semantic_map_manager =
      nullptr;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map =
      nullptr;

  const PlannerParamsProto* planner_params = nullptr;
  const VehicleParamApi* vehicle_params = nullptr;

  const RouteManagerOutput* rm_output = nullptr;
  const RouteSections* route_sections_from_start = nullptr;
  const PlanStartPointInfo* start_point_info = nullptr;
  mapping::ElementId ego_nearest_lane_id = mapping::kInvalidElementId;
  absl::Duration min_path_look_ahead_duration;

  const std::shared_ptr<const SpacetimeTrajectoryManager> st_traj_mgr = nullptr;
  const std::shared_ptr<const PlannerObjectManager> object_manager = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const ExternalCommandStatus* ext_cmd_status = nullptr;

  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const TrafficLightInfoMap* tl_info_map = nullptr;

  const PoseProto* pose = nullptr;
  const Chassis* chassis = nullptr;

  AutonomyStateProto::State autonomy_state = AutonomyStateProto::NOT_READY;

  // Prev states:
  const TrajectoryProto* previous_trajectory = nullptr;
  const mapping::LanePath* prev_target_lane_path = nullptr;
  const RouteSections* prev_route_sections = nullptr;
  double prev_length_along_route = std::numeric_limits<double>::max();
  double prev_max_reach_length = std::numeric_limits<double>::max();
  const mapping::LanePoint* station_anchor = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
  const mapping::LanePath* prev_lane_path_before_lc = nullptr;
  const mapping::LanePath* preferred_lane_path = nullptr;
  DriverAction::LaneChangeCommand new_lc_command = DriverAction::LC_CMD_NONE;
  std::optional<bool> alc_confirmation = std::nullopt;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  bool prev_smooth_state = false;
  absl::Time parking_brake_release_time;
  const DeciderStateProto* decider_state = nullptr;
  const InitializerStateProto* initializer_state = nullptr;
  const TrajectoryOptimizerStateProto*
      selected_trajectory_optimizer_state_proto = nullptr;
  const SpacetimePlannerObjectTrajectoriesProto*
      st_planner_object_trajectories = nullptr;
  const SelectorState* selector_state = nullptr;
  const YellowLightObservationsNew* yellow_light_observations = nullptr;

  const TrafficLightStatesProto* traffic_light_states = nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;
  const PredictionDebugProto* prediction_debug = nullptr;
  const ModelPool* planner_model_pool = nullptr;

  std::shared_ptr<const ObjectsProto> real_objects = nullptr;
  std::shared_ptr<const ObjectsProto> virtual_objects = nullptr;
  const prediction::AvContext* planner_av_context = nullptr;
  std::shared_ptr<const ml::ContextFeature> context_feature = nullptr;

  bool is_standwait = false;
  bool prev_collision_warning_request = false;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_MULTI_TASKS_CRUISE_PLANNER_INPUT_H_
