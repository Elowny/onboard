#include "onboard/planner/decision/door_open_decider.h"

#include "absl/time/time.h"

#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

DoorDecision ComputeDoorDecision(absl::Time now,
                                 std::optional<bool> override_door_open,
                                 double last_override_door_time,
                                 bool end_of_route, bool open_door_at_route_end,
                                 double door_state_override_waiting_time) {
  const bool should_override_door_to_open =
      override_door_open.has_value() && *override_door_open;
  if (open_door_at_route_end) {
    const bool driver_override_expired =
        ToUnixDoubleSeconds(now) >
        (last_override_door_time + door_state_override_waiting_time);
    bool door_open = should_override_door_to_open;
    if (end_of_route && driver_override_expired) {
      door_open = true;
    }
    DoorDecision door_decision;
    door_decision.set_door_state(door_open ? DoorDecision::DOOR_OPEN
                                           : DoorDecision::DOOR_CLOSE);
    door_decision.set_reason(door_open ? DoorDecision::ARRIVED_AT_STATION
                                       : DoorDecision::DEFAULT_CLOSE);
    return door_decision;
  } else {
    DoorDecision door_decision;
    door_decision.set_door_state(should_override_door_to_open
                                     ? DoorDecision::DOOR_OPEN
                                     : DoorDecision::DOOR_CLOSE);
    door_decision.set_reason(DoorDecision::TELEOP_OVERRIDE_DOOR_STATE);
    return door_decision;
  }
}

}  // namespace planner
}  // namespace qcraft
