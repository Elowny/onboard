#ifndef ONBOARD_PLANNER_MAPLESS_MULTI_TASKS_MAPLESS_PLANNER_H_
#define ONBOARD_PLANNER_MAPLESS_MULTI_TASKS_MAPLESS_PLANNER_H_

#include <memory>
#include <vector>

#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/external_command_info.h"
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
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct MultiTasksMaplessPlannerInput {
  // Meant to add one count to shared_ptr here for async planner.
  std::shared_ptr<const PlannerSemanticMapManager>
      planner_semantic_map_manager = nullptr;
  const PoseProto* pose = nullptr;
  const AutonomyStateProto* autonomy_state = nullptr;
  // TODO(zixuan): May be replaced by mapless param.
  const PlannerParamsProto* planner_params = nullptr;
  const VehicleParamApi* vehicle_params = nullptr;
  const PlanStartPointInfo* plan_start_point_info = nullptr;
  std::shared_ptr<const SpacetimeTrajectoryManager> st_traj_mgr = nullptr;
  std::shared_ptr<const PlannerObjectManager> object_manager = nullptr;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map =
      nullptr;
  absl::Duration path_look_ahead_time;
  absl::Time parking_brake_release_time;
  const SpacetimePlannerObjectTrajectoriesProto*
      st_planner_object_trajectories = nullptr;

  // Prev states:
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj_points =
      nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;
  const DeciderStateProto* decider_state = nullptr;
  const InitializerStateProto* initializer_state = nullptr;
  const TrajectoryOptimizerStateProto*
      selected_trajectory_optimizer_state_proto = nullptr;
  const TrajectoryProto* previous_trajectory = nullptr;
  const ExternalCommandStatus* ext_cmd_status = nullptr;
  std::shared_ptr<const PlannerSemanticMapManager> prev_low_freq_psmm = nullptr;
  const mapping::LanePath* prev_target_lane_path = nullptr;
  const LaneChangeStateProto* lane_change_state = nullptr;
  bool prev_collision_warning_request = false;
};

PlannerStatus RunMultiTasksMaplessPlanner(
    const MultiTasksMaplessPlannerInput& input,
    PathBoundedEstPlannerOutput* output, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_MAPLESS_MULTI_TASKS_MAPLESS_PLANNER_H_
