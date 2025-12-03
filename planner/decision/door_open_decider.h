#ifndef ONBOARD_PLANNER_DECISION_DOOR_OPEN_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_DOOR_OPEN_DECIDER_H_

#include "absl/time/time.h"

#include "onboard/proto/trajectory.pb.h"

namespace qcraft {
namespace planner {

DoorDecision ComputeDoorDecision(absl::Time now,
                                 std::optional<bool> override_door_open,
                                 double last_override_door_time,
                                 bool stopped_at_end_of_route,
                                 bool open_door_at_route_end,
                                 double door_state_override_waiting_time);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_DOOR_OPEN_DECIDER_H_
