#ifndef ONBOARD_PLANNER_PLAN_MULTI_TASKS_ALCC_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_MULTI_TASKS_ALCC_PLANNER_H_

#include <memory>
#include <vector>

#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

struct MultiTasksAlccPlannerInput {
  // Meant to add one count to shared_ptr here for async planner.
  std::shared_ptr<const PlannerSemanticMapManager>
      planner_semantic_map_manager = nullptr;
  const PoseProto* pose = nullptr;
  const Chassis* chassis = nullptr;
  const AutonomyStateProto* autonomy_state = nullptr;
  const AlccTaskParamsProto* alcc_params = nullptr;
  const AccTaskParamsProto* acc_params = nullptr;
  const VehicleParamApi* vehicle_params = nullptr;
  const PlanStartPointInfo* plan_start_point_info = nullptr;
  absl::Time plan_time;
  std::shared_ptr<const SpacetimeTrajectoryManager> st_traj_mgr = nullptr;
  std::shared_ptr<const PlannerObjectManager> object_manager = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj_points =
      nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map =
      nullptr;
  absl::Duration path_look_ahead_time;
  const AssistPlanStateProto* assist_plan_state = nullptr;
  absl::Time parking_brake_release_time;
  const DeciderStateProto* decider_state = nullptr;
  const InitializerStateProto* initializer_state = nullptr;
  const TrajectoryOptimizerStateProto*
      selected_trajectory_optimizer_state_proto = nullptr;
  const SpacetimePlannerObjectTrajectoriesProto*
      st_planner_object_trajectories = nullptr;
  const TrajectoryProto* previous_trajectory = nullptr;
  const ExternalCommandStatus* ext_cmd_status = nullptr;
  std::shared_ptr<const PlannerSemanticMapManager> prev_low_freq_psmm = nullptr;
  bool use_online_semantic_map = false;
  bool is_engage_steer_only = false;
  DriverAction::LaneChangeCommand new_lc_command = DriverAction::LC_CMD_NONE;
  const prediction::AvContext* av_context = nullptr;
  const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>*
      online_map_drift_buffer = nullptr;
  bool is_standwait = false;
  bool prev_collision_warning_request = false;
};

PlannerStatus RunMultiTasksAlccPlanner(const MultiTasksAlccPlannerInput& input,
                                       PathBoundedEstPlannerOutput* output,
                                       ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_MULTI_TASKS_ALCC_PLANNER_H_
