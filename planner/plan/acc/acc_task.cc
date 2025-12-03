#include "onboard/planner/plan/acc/acc_task.h"

#include <utility>

#include "common/proto/qacc.pb.h"

#include "onboard/global/trace.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/plan/acc/acc_planner_output.h"
#include "onboard/planner/plan/acc/acc_task_internal.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

PlannerStatus RunAccTask(const AccTaskInput& input, AccTaskOutput* output) {
  SCOPED_QTRACE("RunAccTask");
  AccPlannerOutput acc_planner_output;
  auto acc_planner_status = RunAccPlanner(input, &acc_planner_output);
  if (!acc_planner_status.ok()) {
    output->acc_state = QACCState::ACC_OFF;
    return acc_planner_status;
  }

  // TODO(changqing): check speed finder params settings!
  FillAccRelatedOutput(
      std::move(acc_planner_output), input.is_acc_standwait,
      input.prev_collision_warning_request,
      input.plan_start_point_info->start_point.v(),
      input.acc_params->speed_finder_params().follow_time_headway(),
      input.plan_time, input.lcc_cruising_speed_limit, output);

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
