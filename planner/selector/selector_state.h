#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_STATE_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_STATE_H_

#include <optional>

#include "absl/time/time.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/proto/lane_change_type.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft {
namespace planner {
struct PrefilterHistoryInfo {
  bool left_is_blocked_by_stalled = false;
  bool left_safty_check_failed = false;
  bool right_is_blocked_by_stalled = false;
  bool right_safty_check_failed = false;
};

struct SelectorState {
  // Store state for next frame.
  void FromProto(const SelectorStateProto& proto);
  void ToProto(SelectorStateProto* proto) const;
  void Reset();

  TargetLaneStateProto best_target_lane_state;
  TargetLaneStateProto selected_target_lane_state;
  std::optional<absl::Time> last_redlight_stop_time = std::nullopt;
  LaneChangeType lane_change_type = LaneChangeType::NO_CHANGE;
  LastLcInfoProto last_lc_info;
  SelectorLaneChangeRequestProto selector_lane_change_request;
  std::optional<absl::Time> last_user_reject_alc_time = std::nullopt;
  LaneChangeType last_user_reject_alc_type = LaneChangeType::NO_CHANGE;
  RouteTtcSettingProto route_ttc_setting;
  TurnSignal pre_turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  std::optional<mapping::LanePath> lc_prepare_stage_lane_path = std::nullopt;
  std::optional<absl::Time> activate_selector_time = std::nullopt;
  std::optional<absl::Time> start_lane_change_time = std::nullopt;
  std::optional<absl::Time> give_up_lane_change_time = std::nullopt;
  TurnSignal turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  LaneChangeGeneralType lane_change_general_type =
      LaneChangeGeneralType::LCGT_NO_CHANGE;
  PrefilterStateProto prefilter_state;
  HistoryBufferAbslTime<PrefilterHistoryInfo> prefilter_history_infos;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_STATE_H
