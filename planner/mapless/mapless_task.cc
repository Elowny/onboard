#include "onboard/planner/mapless/mapless_task.h"

#include "onboard/planner/common/proto/planner_status.pb.h"

namespace qcraft::planner {

PlannerStatus RunMaplessTask(PlannerState* /*planner_state*/,
                             ExternalCommandStatus* /*ext_cmd_status*/,
                             ThreadPool* /*thread_pool*/) {
  return PlannerStatus(PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
                       "to be implemented.");
}

}  // namespace qcraft::planner
