#ifndef ONBOARD_PLANNER_UTIL_PLANNER_STATUS_MACROS_H_
#define ONBOARD_PLANNER_UTIL_PLANNER_STATUS_MACROS_H_

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/utils/status_macros.h"

#define RETURN_PLANNER_STATUS_OR_ASSIGN(lhs, rexpr, planner_status_code)  \
  RETURN_PLANNER_STATUS_OR_ASSIGN_4_(                                     \
      STATUS_MACROS_IMPL_CONCAT_(_status_or_value, __LINE__), lhs, rexpr, \
      planner_status_code)

#define RETURN_PLANNER_STATUS_OR_ASSIGN_4_(statusor, lhs, rexpr,              \
                                           planner_status_code)               \
  auto statusor = (rexpr);                                                    \
  if (!statusor.ok()) {                                                       \
    return (PlannerStatus(planner_status_code, statusor.status().message())); \
  }                                                                           \
  lhs = std::move(statusor).value()

#endif  // ONBOARD_PLANNER_UTIL_PLANNER_STATUS_MACROS_H_
