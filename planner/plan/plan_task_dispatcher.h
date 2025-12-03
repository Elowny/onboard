#ifndef ONBOARD_PLANNER_PLAN_PLAN_TASK_DISPATCHER_H_
#define ONBOARD_PLANNER_PLAN_PLAN_TASK_DISPATCHER_H_

#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_output.pb.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {

PlannerStatus RunPlanTaskDispatcher(
    const CoordinateConverter& coordinate_converter, const PlannerInput& input,
    const RouteManagerOutput& route_output, const ObjectsProto* objects_proto,
    absl::Time current_time, double time_interval,
    ExternalCommandInfo* ext_cmd_info, PlannerState* planner_state,
    PlannerOutput* output, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PLAN_TASK_DISPATCHER_H_
