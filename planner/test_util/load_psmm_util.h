#ifndef ONBOARD_PLANNER_TEST_UTIL_LOAD_PSMM_UTIL_H_
#define ONBOARD_PLANNER_TEST_UTIL_LOAD_PSMM_UTIL_H_

#include <memory>

#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

const PlannerSemanticMapManager& CreateDojoTestPSMM();

std::shared_ptr<PlannerSemanticMapManager> CreateDojoTestPSMMSharedPtr();

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_TEST_UTIL_LOAD_PSMM_UTIL_H_
