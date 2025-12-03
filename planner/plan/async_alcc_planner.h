#ifndef ONBOARD_PLANNER_PLAN_ASYNC_ALCC_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_ASYNC_ALCC_PLANNER_H_

#include <optional>

#include "common/proto/qacc.pb.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/plan/acc/acc_planner_output.h"
#include "onboard/planner/plan/async_planner_output.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/multi_tasks_alcc_planner.h"

namespace qcraft {
namespace planner {

// The function reuses prev_path if provided. Otherwise, it recomputes the path.
PlannerStatus RunAsyncAlccPlanner(
    const MultiTasksAlccPlannerInput& input,
    const QACCTaskProto& acc_task_proto,
    std::optional<AccPlannerOutput>* acc_output_ptr, AsyncPlannerOutput* output,
    AsyncPlannerState* async_planner_state, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ASYNC_ALCC_PLANNER_H_
