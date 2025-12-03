#ifndef ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct StPathPlannerInput {
  int plan_id = 0;
  bool is_run_model_l4 = false;
  const StPathPlanStartPointInfo* st_path_start_point_info = nullptr;
  absl::Duration path_look_ahead_duration = absl::ZeroDuration();
  const VehicleParamApi* vehicle_params = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  // Not a const since it may be modified and will be moved.
  SchedulerOutput scheduler_output;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;

  // For rebuilding constraint manager on lc pause.
  const PlanStartPointInfo* start_point_info = nullptr;
  const PlannerObjectManager* obj_mgr = nullptr;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  const DeciderStateProto* prev_decider_state = nullptr;
  absl::Time parking_brake_release_time;
  bool enable_pull_over = false;
  bool enable_traffic_light_stopping = true;
  std::optional<double> brake_to_stop = std::nullopt;
  bool enable_force_stop = false;

  SpacetimePlannerObjectTrajectories init_st_planner_object_traj;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  DeciderOutput decider_output;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const InitializerStateProto* prev_initializer_state = nullptr;
  const TrajectoryOptimizerStateProto* trajectory_optimizer_state_proto =
      nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;

  const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;
  // Params.
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  // Lane change style params.
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_radical_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_normal_params =
      nullptr;
  const TrajectoryOptimizerParamsProto*
      trajectory_optimizer_lc_conservative_params = nullptr;
};

// Should not contain pointers since the actual object might have been destroyed
// and cannot be used in speed finder in async planner.
struct StPathPlannerOutput {
  SchedulerOutput scheduler_output;
  DiscretizedPath path;
  std::vector<PathPoint> st_path_points;
  ConstraintManager constraint_manager;
  absl::flat_hash_set<std::string> follower_set;
  double follower_max_decel = 0.0;  // Should be larger than or equal to 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  std::optional<NudgeOjbectInfo> nudge_object_info;
  LeadingGroup leading_trajs;
  SpacetimePlannerObjectTrajectories st_planner_object_traj;
  InitializerDebugProto initializer_debug_proto;
  vis::vantage::ChartDataBundleProto chart_data;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;
  DeciderStateProto decider_state;
  InitializerStateProto initializer_state;
  TrajectoryOptimizerStateProto trajectory_optimizer_state_proto;
  LaneChangeSafetyDebugProto lane_change_safety_debug_proto;
  // Optimizer Auto Tuning
  AutoTuningTrajectoryProto candidate_auto_tuning_traj_proto;
  AutoTuningTrajectoryProto expert_auto_tuning_traj_proto;
};

PlannerStatus RunStPathPlanner(StPathPlannerInput input,
                               StPathPlannerOutput* out,
                               ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
