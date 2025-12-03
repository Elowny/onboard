#ifndef ONBOARD_PLANNER_ASSIST_ALCC_TURN_SIGNAL_DECIDER_H_
#define ONBOARD_PLANNER_ASSIST_ALCC_TURN_SIGNAL_DECIDER_H_

#include "onboard/planner/assist/external_command_info.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft::planner {
struct AlccTurnSignalResult {
  TurnSignal signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason reason = TurnSignalReason::TURN_SIGNAL_OFF;
};

AlccTurnSignalResult RunAlccTurnSignalDecider(
    const ExternalCommandStatus& ext_cmd_status, QALCState state);

}  // namespace qcraft::planner
#endif
