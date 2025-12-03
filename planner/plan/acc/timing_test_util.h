#ifndef ONBOARD_PLANNER_PLAN_ACC_TIMING_TEST_UTIL_
#define ONBOARD_PLANNER_PLAN_ACC_TIMING_TEST_UTIL_

#include <iostream>

#if defined(__ASPICE_TIMEING_TEST__)
#define PRINT_FUNCTION std::cout << "FUNCTION_NAME:" << __func__ << std::endl;
#else
#define PRINT_FUNCTION
#endif

#endif  // ONBOARD_PLANNER_PLAN_ACC_TIMING_TEST_UTIL_
