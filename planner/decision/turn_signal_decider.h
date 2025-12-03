#ifndef ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_

#include <optional>

#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft {
namespace planner {

struct TurnSignalResult {
  TurnSignal signal = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason reason = TurnSignalReason::TURN_SIGNAL_OFF;
};

// Planner 3.0
TurnSignalResult DecideTurnSignal(
    const PlannerSemanticMapManager& psmm, TurnSignal route_signal,
    TurnSignal pre_lane_change_signal,
    const mapping::LanePath& current_lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id,
    const LaneChangeStateProto& lc_state,
    const ExternalCommandStatus& ext_cmd_status,
    const DrivePassage& drive_passage, const FrenetBox& ego_sl_box,
    const TurnSignalResult& planned_result, const PoseProto& ego_pose);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_TURN_SIGNAL_DECIDER_H_
