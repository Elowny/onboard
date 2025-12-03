#include "onboard/planner/decision/door_open_decider.h"

#include "gtest/gtest.h"

#include "onboard/global/clock.h"
#include "onboard/utils/time_util.h"
namespace qcraft::planner {
namespace {
TEST(DoorOpenDeciderTest, OpenDoorAtRouteEnd) {
  {
    const auto now = Clock::Now();
    const std::optional<bool> override_door_open = false;
    const double last_override_door_time = 0.0;
    const bool stopped_at_end_of_route = true;
    const bool open_door_at_route_end = true;
    const double door_state_override_waiting_time = 40.0;
    const auto door_decision =
        ComputeDoorDecision(now, override_door_open, last_override_door_time,
                            stopped_at_end_of_route, open_door_at_route_end,
                            door_state_override_waiting_time);
    EXPECT_EQ(door_decision.door_state(), DoorDecision::DOOR_OPEN);
    EXPECT_EQ(door_decision.reason(), DoorDecision::ARRIVED_AT_STATION);
  }

  {
    const auto now = Clock::Now();
    const std::optional<bool> override_door_open = false;
    const double last_override_door_time = ToUnixDoubleSeconds(now);
    const bool stopped_at_end_of_route = true;
    const bool open_door_at_route_end = true;
    const double door_state_override_waiting_time = 40.0;
    const auto door_decision =
        ComputeDoorDecision(now, override_door_open, last_override_door_time,
                            stopped_at_end_of_route, open_door_at_route_end,
                            door_state_override_waiting_time);
    EXPECT_EQ(door_decision.door_state(), DoorDecision::DOOR_CLOSE);
    EXPECT_EQ(door_decision.reason(), DoorDecision::DEFAULT_CLOSE);
  }
}
TEST(DoorOpenDeciderTest, TeleopOpenDoor) {
  {
    const auto now = Clock::Now();
    const std::optional<bool> override_door_open = true;
    const double last_override_door_time = 0.0;
    const bool stopped_at_end_of_route = true;
    const bool open_door_at_route_end = false;
    const double door_state_override_waiting_time = 40.0;
    const auto door_decision =
        ComputeDoorDecision(now, override_door_open, last_override_door_time,
                            stopped_at_end_of_route, open_door_at_route_end,
                            door_state_override_waiting_time);
    EXPECT_EQ(door_decision.door_state(), DoorDecision::DOOR_OPEN);
    EXPECT_EQ(door_decision.reason(), DoorDecision::TELEOP_OVERRIDE_DOOR_STATE);
  }

  {
    const auto now = Clock::Now();
    const std::optional<bool> override_door_open = false;
    const double last_override_door_time = 0.0;
    const bool stopped_at_end_of_route = true;
    const bool open_door_at_route_end = false;
    const double door_state_override_waiting_time = 40.0;
    const auto door_decision =
        ComputeDoorDecision(now, override_door_open, last_override_door_time,
                            stopped_at_end_of_route, open_door_at_route_end,
                            door_state_override_waiting_time);
    EXPECT_EQ(door_decision.door_state(), DoorDecision::DOOR_CLOSE);
    EXPECT_EQ(door_decision.reason(), DoorDecision::TELEOP_OVERRIDE_DOOR_STATE);
  }
}
}  // namespace
}  // namespace qcraft::planner
