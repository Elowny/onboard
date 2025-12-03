#ifndef ONBOARD_PLANNER_PLAN_ACC_TASK_H_
#define ONBOARD_PLANNER_PLAN_ACC_TASK_H_

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/plan/acc/acc_task_input.h"
#include "onboard/planner/plan/acc/acc_task_output.h"

namespace qcraft::planner {

PlannerStatus RunAccTask(const AccTaskInput& input, AccTaskOutput* output);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_TASK_H_
