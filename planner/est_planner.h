#ifndef ONBOARD_PLANNER_EST_PLANNER_H_
#define ONBOARD_PLANNER_EST_PLANNER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

struct EstPlannerInput {
  const DrivingMapTopo* driving_map_topo = nullptr;
  const mapping::v2::SemanticMapManager* semantic_map_manager = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  int plan_id = 1 /*for plot and debugging*/;
  const VehicleParamApi* vehicle_params = nullptr;
  bool is_run_model_l4 = false;

  // Prev states:
  absl::Time parking_brake_release_time;
  const DeciderStateProto* decider_state = nullptr;
  const InitializerStateProto* initializer_state = nullptr;
  const TrajectoryOptimizerStateProto* trajectory_optimizer_state_proto =
      nullptr;
  const SpacetimePlannerObjectTrajectoriesProto*
      st_planner_object_trajectories = nullptr;
  bool prev_collision_warning_request = false;

  const PlannerObjectManager* obj_mgr = nullptr;
  const PlanStartPointInfo* start_point_info = nullptr;
  const StPathPlanStartPointInfo* st_path_start_point_info = nullptr;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map = nullptr;
  const absl::flat_hash_set<std::string>* stalled_objects = nullptr;
  const SceneOutputProto* scene_reasoning = nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  LaneChangeStyle lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  bool enable_pull_over = false;
  bool enable_traffic_light_stopping = true;
  std::optional<double> brake_to_stop = std::nullopt;
  bool enable_force_stop = false;

  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;

  const ml::captain_net::CaptainNetOutput* captain_net_output = nullptr;

  // For ml.
  const prediction::AvContext* planner_av_context = nullptr;
  const ObjectsProto* real_objects = nullptr;
  const ObjectsProto* virtual_objects = nullptr;
  const ModelPool* planner_model_pool = nullptr;

  // Params.
  const DecisionConstraintConfigProto* decision_constraint_config = nullptr;
  const InitializerConfig* initializer_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  // Lane change style params.
  const SpeedFinderParamsProto* speed_finder_lc_radical_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_lc_conservative_params = nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_radical_params =
      nullptr;
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_lc_normal_params =
      nullptr;
  const TrajectoryOptimizerParamsProto*
      trajectory_optimizer_lc_conservative_params = nullptr;
  const SpacetimePlannerObjectTrajectoriesParamsProto*
      spacetime_planner_object_trajectories_params = nullptr;
};

PlannerStatus RunEstPlanner(const EstPlannerInput& input,
                            SchedulerOutput scheduler_output,
                            EstPlannerOutput* est_output,
                            EstPlannerDebug* debug_info,
                            vis::vantage::ChartDataBundleProto* chart_data,
                            ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_EST_PLANNER_H_
