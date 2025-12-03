#ifndef ONBOARD_PLANNER_MAPLESS_MAPLESS_TASK_H_
#define ONBOARD_PLANNER_MAPLESS_MAPLESS_TASK_H_

#include "onboard/async/thread_pool.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/planner_state.h"

namespace qcraft::planner {
PlannerStatus RunMaplessTask(PlannerState* planner_state,
                             ExternalCommandStatus* ext_cmd_status,
                             ThreadPool* thread_pool);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_MAPLESS_MAPLESS_TASK_H_
