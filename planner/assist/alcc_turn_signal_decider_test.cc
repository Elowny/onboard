#include "onboard/planner/assist/alcc_turn_signal_decider.h"

#include "gtest/gtest.h"
namespace qcraft::planner {
namespace {
TEST(AlccTurnSignalDeciderTest, ExtCmdStatusTest) {
  {
    const auto res = RunAlccTurnSignalDecider(
        ExternalCommandStatus{
            .override_emergency_blinker_on = true,
        },
        /*state*/ QALCState::ALC_STANDBY_ENABLE);
    EXPECT_EQ(res.signal, TURN_SIGNAL_EMERGENCY);
    EXPECT_EQ(res.reason, TELEOP_TURN_SIGNAL);
  }

  {
    const auto res = RunAlccTurnSignalDecider(
        ExternalCommandStatus{
            .override_left_blinker_on = true,
            .override_right_blinker_on = true,
        },
        /*state*/ QALCState::ALC_STANDBY_ENABLE);
    EXPECT_EQ(res.signal, TURN_SIGNAL_EMERGENCY);
    EXPECT_EQ(res.reason, TELEOP_TURN_SIGNAL);
  }

  {
    const auto res = RunAlccTurnSignalDecider(
        ExternalCommandStatus{
            .override_left_blinker_on = true,
        },
        /*state*/ QALCState::ALC_STANDBY_ENABLE);
    EXPECT_EQ(res.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(res.reason, TELEOP_TURN_SIGNAL);
  }

  {
    const auto res = RunAlccTurnSignalDecider(
        ExternalCommandStatus{
            .override_right_blinker_on = true,
        },
        /*state*/ QALCState::ALC_STANDBY_ENABLE);
    EXPECT_EQ(res.signal, TURN_SIGNAL_RIGHT);
    EXPECT_EQ(res.reason, TELEOP_TURN_SIGNAL);
  }
}

TEST(AlccTurnSignalDeciderTest, LaneChangeSignalTest) {
  // Lane keep test.
  {
    const ExternalCommandStatus ext_cmd_status;
    const std::vector<QALCState> lane_keep_states = {
        ALC_OFF,       ALC_STANDBY,   ALC_STANDBY_ENABLE, ALC_RETURN_COMPLETED,
        ALC_COMPLETED, ALC_RETURNING,
    };

    for (const auto state : lane_keep_states) {
      const auto res = RunAlccTurnSignalDecider(ext_cmd_status, state);
      EXPECT_EQ(res.signal, TURN_SIGNAL_NONE);
      EXPECT_EQ(res.reason, TURN_SIGNAL_OFF);
    }
  }

  // Lane change left test.
  {
    const ExternalCommandStatus lc_left_status{
        .lane_change_command = DriverAction::LC_CMD_LEFT,
    };
    const std::vector<QALCState> lane_change_states = {ALC_PREPARE, ALC_ONGOING,
                                                       ALC_CROSSING_LANE};

    for (const auto state : lane_change_states) {
      const auto res = RunAlccTurnSignalDecider(lc_left_status, state);
      EXPECT_EQ(res.signal, TURN_SIGNAL_LEFT);
      EXPECT_EQ(res.reason, LANE_CHANGE_TURN_SIGNAL);
    }
  }

  // Lane change right test.
  {
    const ExternalCommandStatus lc_right_status{
        .lane_change_command = DriverAction::LC_CMD_RIGHT,
    };
    const std::vector<QALCState> lane_change_states = {ALC_PREPARE, ALC_ONGOING,
                                                       ALC_CROSSING_LANE};

    for (const auto state : lane_change_states) {
      const auto res = RunAlccTurnSignalDecider(lc_right_status, state);
      EXPECT_EQ(res.signal, TURN_SIGNAL_RIGHT);
      EXPECT_EQ(res.reason, LANE_CHANGE_TURN_SIGNAL);
    }
  }
}
}  // namespace
}  // namespace qcraft::planner
