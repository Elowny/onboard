#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_INPUT_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_INPUT_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct MotionSearchInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const ApolloTrajectoryPointProto* start_point = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  absl::Time plan_time;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const GeometryGraph* geom_graph = nullptr;
  const std::vector<Vec2d>* reference_line_points = nullptr;
  const GeometryFormBuilder* form_builder = nullptr;
  const CollisionChecker* collision_checker = nullptr;
  const std::vector<double>* stop_s_vec = nullptr;
  const std::vector<LeadingGroup>* leading_groups = nullptr;
  const ConstraintProto::LeadingObjectProto* blocking_static_traj = nullptr;
  const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;
  bool is_lane_change = false;
  bool eval_safety = false;
  LaneChangeStyle lc_style = LC_STYLE_NORMAL;
  const TrajectoryProto* log_av_trajectory = nullptr;
  bool is_run_model_l4 = false;
};

struct InitializerInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const StPathPlanStartPointInfo* path_start_point_info = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  const LaneChangeStateProto* lane_change_state = nullptr;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const InitializerStateProto* prev_initializer_state = nullptr;
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const VehicleParamApi* vehicle_params = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  int plan_id = 0;
  const TrajectoryProto* log_av_trajectory = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  bool borrow_lane = false;
  const FrenetBox* av_frenet_box = nullptr;
  bool is_run_model_l4 = false;

  // For rebuilding constraint manager on lc pause.
  const PlanStartPointInfo* start_point_info = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const PlannerObjectManager* obj_mgr = nullptr;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const DeciderStateProto* prev_decider_state = nullptr;
  absl::Time parking_brake_release_time;
  bool enable_pull_over = false;
  bool enable_traffic_light_stopping = true;
  std::optional<double> brake_to_stop = std::nullopt;
  bool enable_force_stop = false;

  const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;
};

struct ReferenceLineSearcherInput {
  const GeometryGraph* geometry_graph = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const VehicleDriveParamsProto* vehicle_drive = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_INPUT_H_
