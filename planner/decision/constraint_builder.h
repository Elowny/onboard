#ifndef ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_
#define ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_

#include "absl/status/statusor.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"

namespace qcraft::planner {

absl::StatusOr<DeciderOutput> BuildConstraints(
    const DeciderInput& decider_input);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_
