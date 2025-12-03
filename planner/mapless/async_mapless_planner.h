#ifndef ONBOARD_PLANNER_MAPLESS_ASYNC_MAPLESS_PLANNER_H_
#define ONBOARD_PLANNER_MAPLESS_ASYNC_MAPLESS_PLANNER_H_

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/mapless/multi_tasks_mapless_planner.h"
#include "onboard/planner/plan/async_planner_output.h"
#include "onboard/planner/plan/async_planner_state.h"

namespace qcraft {
namespace planner {

PlannerStatus RunAsyncMaplessPlanner(const MultiTasksMaplessPlannerInput& input,
                                     AsyncPlannerOutput* output,
                                     AsyncPlannerState* async_planner_state,
                                     ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_MAPLESS_ASYNC_MAPLESS_PLANNER_H_
