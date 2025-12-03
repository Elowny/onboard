#include "onboard/planner/selector/selector_state.h"

#include "absl/time/time.h"

#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

void SelectorState::FromProto(const SelectorStateProto& proto) {
  best_target_lane_state = proto.best_target_lane_state();
  selected_target_lane_state = proto.selected_target_lane_state();

  if (proto.has_last_redlight_stop_time()) {
    last_redlight_stop_time =
        qcraft::FromProto(proto.last_redlight_stop_time());
  } else {
    last_redlight_stop_time = std::nullopt;
  }

  lane_change_type = proto.lane_change_type();
  last_lc_info = proto.last_lc_info();
  selector_lane_change_request = proto.selector_lane_change_request();

  if (proto.has_last_user_reject_alc_time()) {
    last_user_reject_alc_time =
        qcraft::FromProto(proto.last_user_reject_alc_time());
  } else {
    last_user_reject_alc_time = std::nullopt;
  }
  last_user_reject_alc_type = proto.last_user_reject_alc_type();
  route_ttc_setting = proto.route_ttc_setting();
  pre_turn_signal = proto.pre_turn_signal();
  turn_signal = proto.turn_signal();
  if (proto.has_activate_selector_time()) {
    activate_selector_time = qcraft::FromProto(proto.activate_selector_time());
  }
  if (proto.has_start_lane_change_time()) {
    start_lane_change_time = qcraft::FromProto(proto.start_lane_change_time());
  }
  if (proto.has_give_up_lane_change_time()) {
    give_up_lane_change_time =
        qcraft::FromProto(proto.give_up_lane_change_time());
  }
  lane_change_general_type = proto.lane_change_general_type();
  prefilter_state = proto.prefilter_state();
}

void SelectorState::ToProto(SelectorStateProto* proto) const {
  proto->Clear();
  *proto->mutable_best_target_lane_state() = best_target_lane_state;
  *proto->mutable_selected_target_lane_state() = selected_target_lane_state;
  if (last_redlight_stop_time.has_value()) {
    qcraft::ToProto(*last_redlight_stop_time,
                    proto->mutable_last_redlight_stop_time());
  }
  proto->set_lane_change_type(lane_change_type);
  *proto->mutable_last_lc_info() = last_lc_info;
  *proto->mutable_selector_lane_change_request() = selector_lane_change_request;
  if (last_user_reject_alc_time.has_value()) {
    qcraft::ToProto(*last_user_reject_alc_time,
                    proto->mutable_last_user_reject_alc_time());
  }
  proto->set_last_user_reject_alc_type(last_user_reject_alc_type);
  proto->set_pre_turn_signal(pre_turn_signal);
  proto->set_turn_signal(turn_signal);
  *proto->mutable_route_ttc_setting() = route_ttc_setting;
  if (activate_selector_time.has_value()) {
    qcraft::ToProto(*activate_selector_time,
                    proto->mutable_activate_selector_time());
  }

  if (start_lane_change_time.has_value()) {
    qcraft::ToProto(*start_lane_change_time,
                    proto->mutable_start_lane_change_time());
  }

  if (give_up_lane_change_time.has_value()) {
    qcraft::ToProto(*give_up_lane_change_time,
                    proto->mutable_give_up_lane_change_time());
  }
  proto->set_lane_change_general_type(lane_change_general_type);
  *proto->mutable_prefilter_state() = prefilter_state;
}

void SelectorState::Reset() {
  best_target_lane_state.Clear();
  selected_target_lane_state.Clear();
  last_redlight_stop_time = std::nullopt;
  lane_change_type = LaneChangeType::NO_CHANGE;
  last_lc_info.Clear();
  selector_lane_change_request.Clear();
  last_user_reject_alc_time = std::nullopt;
  last_user_reject_alc_type = LaneChangeType::NO_CHANGE;
  lane_change_general_type = LaneChangeGeneralType::LCGT_NO_CHANGE;
  route_ttc_setting.Clear();
  pre_turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  lc_prepare_stage_lane_path = std::nullopt;
  activate_selector_time.reset();
  start_lane_change_time.reset();
  give_up_lane_change_time.reset();
  prefilter_state.Clear();
  prefilter_history_infos.clear();
}

}  // namespace planner
}  // namespace qcraft
