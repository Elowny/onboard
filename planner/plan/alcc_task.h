#ifndef ONBOARD_PLANNER_PLAN_ALCC_TASK_H_
#define ONBOARD_PLANNER_PLAN_ALCC_TASK_H_

#include <memory>
#include <vector>

#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

struct AlccTaskOutput {
  TrajectoryProto trajectory_info;
  // TODO(weijun): min dependency.
  PlannerDebugProto debug_info;
  vis::vantage::ChartsDataProto chart_data;
  HmiContentProto hmi_content;

  bool scheduled_async_low_freq = false;
};

struct AlccTaskInput {
  // Meant to add one count to shared_ptr here for async planner.
  std::shared_ptr<PlannerSemanticMapManager> planner_semantic_map_manager =
      nullptr;
  const PoseProto* pose = nullptr;
  const Chassis* chassis = nullptr;
  const AutonomyStateProto* autonomy_state = nullptr;
  const AlccTaskParamsProto* alcc_params = nullptr;
  const AccTaskParamsProto* acc_params = nullptr;
  const VehicleParamApi* vehicle_params = nullptr;
  const PlanStartPointInfo* plan_start_point_info = nullptr;
  const ExternalCommandQueue* ext_cmd_queue = nullptr;
  absl::Time plan_time;
  std::shared_ptr<const SpacetimeTrajectoryManager> st_traj_mgr = nullptr;
  std::shared_ptr<const PlannerObjectManager> object_manager = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj_points =
      nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map =
      nullptr;
  bool use_online_semantic_map = false;
  const prediction::AvContext* av_context = nullptr;
  const HistoryBufferAbslTime<PiecewiseLinearFunction<double>>*
      online_map_drift_buffer = nullptr;
};

PlannerStatus RunAlccTask(const AlccTaskInput& input, AlccTaskOutput* output,
                          PlannerState* planner_state,
                          ExternalCommandStatus* ext_cmd_status,
                          ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ALCC_TASK_H_
