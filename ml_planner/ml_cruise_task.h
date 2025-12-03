#ifndef ONBOARD_ML_PLANNER_ML_CRUISE_TASK_H_
#define ONBOARD_ML_PLANNER_ML_CRUISE_TASK_H_

#include "onboard/ml_planner/ml_cruise_task_input.h"
#include "onboard/ml_planner/ml_planner_status.h"
#include "onboard/ml_planner/proto/ml_planner_debug.pb.h"
#include "onboard/ml_planner/proto/ml_planner_output.pb.h"

namespace qcraft::mlplanner {

MLPlannerStatus RunMLCruiseTask(const MLCruiseTaskInput& input,
                                MlPlannerOutputProto* output,
                                MlPlannerDebugProto* ml_planner_debug);

}  // namespace qcraft::mlplanner

#endif  // ONBOARD_ML_PLANNER_ML_CRUISE_TASK_H_
