#ifndef ONBOARD_ML_PLANNER_MULTI_CONDITIONS_ML_CRUISE_PLANNER_H_
#define ONBOARD_ML_PLANNER_MULTI_CONDITIONS_ML_CRUISE_PLANNER_H_

#include "onboard/ml_planner/ml_planner_status.h"
#include "onboard/ml_planner/multi_conditions_ml_cruise_planner_input.h"
#include "onboard/ml_planner/proto/ml_planner_debug.pb.h"
#include "onboard/ml_planner/proto/ml_planner_output.pb.h"

namespace qcraft::mlplanner {

MLPlannerStatus RunMultiConditionsMLCruisePlanner(
    const MultiConditionsMLCruisePlannerInput& input,
    MlPlannerOutputProto* output, MlPlannerDebugProto* ml_planner_debug);

}  // namespace qcraft::mlplanner

#endif  // ONBOARD_ML_PLANNER_MULTI_CONDITIONS_ML_CRUISE_PLANNER_H_
