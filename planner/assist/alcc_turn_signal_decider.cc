#include "onboard/planner/assist/alcc_turn_signal_decider.h"
namespace qcraft::planner {

AlccTurnSignalResult RunAlccTurnSignalDecider(
    const ExternalCommandStatus& ext_cmd_status, QALCState state) {
  // Teleop override turn signal.
  if (ext_cmd_status.override_emergency_blinker_on ||
      (ext_cmd_status.override_left_blinker_on &&
       ext_cmd_status.override_right_blinker_on)) {
    return {TURN_SIGNAL_EMERGENCY, TELEOP_TURN_SIGNAL};
  }
  if (ext_cmd_status.override_left_blinker_on) {
    return {TURN_SIGNAL_LEFT, TELEOP_TURN_SIGNAL};
  }
  if (ext_cmd_status.override_right_blinker_on) {
    return {TURN_SIGNAL_RIGHT, TELEOP_TURN_SIGNAL};
  }

  // Lane change signal.
  switch (state) {
    case QALCState::ALC_OFF:
    case QALCState::ALC_STANDBY:
    case QALCState::ALC_STANDBY_ENABLE:
    case QALCState::ALC_RETURN_COMPLETED:
    case QALCState::ALC_COMPLETED:
    case QALCState::ALC_RETURNING:
      return {TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
    case QALCState::ALC_PREPARE:
    case QALCState::ALC_ONGOING:
    case QALCState::ALC_CROSSING_LANE:
      return {ext_cmd_status.lane_change_command == DriverAction::LC_CMD_LEFT
                  ? TURN_SIGNAL_LEFT
                  : TURN_SIGNAL_RIGHT,
              LANE_CHANGE_TURN_SIGNAL};
  }
}
}  // namespace qcraft::planner
