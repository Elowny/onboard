#ifndef ONBOARD_PLANNER_PLAN_CRUISE_TASK_H_
#define ONBOARD_PLANNER_PLAN_CRUISE_TASK_H_

#include <memory>
#include <vector>

#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/aeb_planner.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct CruiseTaskOutput {
  TrajectoryProto trajectory_info;
  PlannerStatusProto planner_status_proto;
  // TODO(weijun): min dependency.
  PlannerDebugProto debug_info;
  vis::vantage::ChartsDataProto chart_data;
  HmiContentProto hmi_content;

  bool scheduled_async_low_freq = false;
};

struct CruiseTaskInput {
  bool rerouted;
  const CoordinateConverter* coordinate_converter;
  const PlannerInput* planner_input;
  const PlannerParamsProto* planner_params;
  const RouteManagerOutput* route_output;
  const PlanStartPointInfo* plan_start_point_info;
  const ExternalCommandQueue* ext_cmd_queue = nullptr;
  absl::Time plan_time;

  std::shared_ptr<const SpacetimeTrajectoryManager> st_traj_mgr = nullptr;
  std::shared_ptr<const PlannerObjectManager> object_manager = nullptr;

  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj_points;
  const TrajectoryProto* previous_trajectory;
  const AebPlannerOutput* aeb_output;
};

PlannerStatus RunCruiseTask(const CruiseTaskInput& input,
                            CruiseTaskOutput* result,
                            PlannerState* planner_state,
                            ExternalCommandStatus* ext_cmd_status,
                            ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // #ifndef ONBOARD_PLANNER_PLAN_CRUISE_TASK_H_
