#ifndef ONBOARD_PLANNER_PLAN_ACC_TARGET_WITHOUT_MAP_H_
#define ONBOARD_PLANNER_PLAN_ACC_TARGET_WITHOUT_MAP_H_

#include "onboard/planner/plan/acc/acc_target.h"

namespace qcraft::planner {

AccTargetPerCorridor SelectAccTargetWithoutMap(const AccTargetInput& input);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_TARGET_WITHOUT_MAP_H_
