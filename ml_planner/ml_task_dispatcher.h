#ifndef ONBOARD_ML_PLANNER_ML_TASK_DISPATCHER_H_
#define ONBOARD_ML_PLANNER_ML_TASK_DISPATCHER_H_

#include "onboard/ml_planner/ml_planner_input.h"
#include "onboard/ml_planner/ml_planner_status.h"
#include "onboard/ml_planner/proto/ml_planner_debug.pb.h"
#include "onboard/ml_planner/proto/ml_planner_output.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::mlplanner {

MLPlannerStatus RunMLTaskDispatcher(const MLPlannerInput& input,
                                    const ObjectsProto* objects_proto,
                                    MlPlannerOutputProto* output,
                                    MlPlannerDebugProto* ml_planner_debug);

}  // namespace qcraft::mlplanner

#endif  // ONBOARD_ML_PLANNER_ML_TASK_DISPATCHER_H_
