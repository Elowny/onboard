#ifndef ONBOARD_PLANNER_PLAN_PLAN_TASK_SWITCHER_H_
#define ONBOARD_PLANNER_PLAN_PLAN_TASK_SWITCHER_H_

#include <deque>
#include <memory>

#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/proto/assist_state.pb.h"

namespace qcraft::planner {

struct SwitchTaskResult {
  bool switched;
  std::deque<PlanTask> new_task_queue;
};

// Switch among ACC, LCC and L4(NOA)
SwitchTaskResult SwitchPlanTask(
    int run_mode, int task_init_type,
    const std::deque<PlanTask>& current_task_queue,
    const std::shared_ptr<PlannerSemanticMapManager>& psmm,
    AssistStateProto::AssistDriveSystemState assist_state,
    const RouteManagerOutput& route_output, bool rerouted);

void UpdatePlannerStateOnTaskSwitch(PlanTaskType prev_task,
                                    PlanTaskType new_task,
                                    int cruise_async_low_freq_cycle_iterations,
                                    int alcc_async_low_freq_cycle_iterations,
                                    PlannerState* planner_state);

void ResetAssistPlanStateByTaskType(PlanTaskType type,
                                    AssistPlanStateProto* assist_plan_state,
                                    ExternalCommandStatus* ext_cmd_status);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PLAN_TASK_SWITCHER_H_
