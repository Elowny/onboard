#ifndef ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_
#define ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_

#include <optional>
#include <vector>

#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"

namespace qcraft::planner {

struct DeciderOutput {
  ConstraintManager constraint_manager;
  DeciderStateProto decider_state;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
};

}  // namespace qcraft::planner

#endif
