#include "onboard/planner/decision/traffic_light/tl_info.h"

#include <utility>

#include "absl/hash/hash.h"
#include "absl/strings/str_cat.h"

namespace qcraft {
namespace planner {

TlInfo::TlInfo(mapping::ElementId lane_id,
               std::vector<double> control_point_relative_s, bool can_go_on_red,
               absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tls,
               bool is_fresh, std::string last_error_msg, Direction direction)
    : lane_id_(lane_id),
      control_point_relative_s_(std::move(control_point_relative_s)),
      can_go_on_red_(can_go_on_red),
      tls_(std::move(tls)),
      is_fresh_(is_fresh),
      last_error_msg_(std::move(last_error_msg)),
      direction_(direction),
      is_empty_(false) {
  if (tls_.count(TrafficLightDirection::STRAIGHT) &&
      tls_.count(TrafficLightDirection::LEFT)) {
    tl_control_type_ = TrafficLightControlType::LEFT_WAITING_AREA;
  }
}

std::string TlInfo::DebugString() const {
  if (is_empty_) return "TlInfo is empty!";

  std::string debug_info = absl::StrCat("\n");
  absl::StrAppend(&debug_info, "lane_id: ", lane_id_, "\t");
  if (can_go_on_red_) {
    absl::StrAppend(&debug_info, "can_go_on_red\n");
  } else {
    absl::StrAppend(&debug_info, "control point relative s: ");
    for (const auto& s : control_point_relative_s_) {
      absl::StrAppend(&debug_info, s, " ");
    }
    absl::StrAppend(&debug_info, "\n");
  }

  absl::StrAppend(&debug_info, "Control Type: ",
                  TrafficLightControlType_Name(tl_control_type_), "\n");

  for (auto it = tls_.begin(); it != tls_.end(); ++it) {
    absl::StrAppend(&debug_info, "tl_id: ", it->second.tl_id, " ",
                    TrafficLightDirection_Name(it->first));
    absl::StrAppend(&debug_info, " ===> ",
                    TrafficLightState_Name(it->second.tl_state));
    if (it->second.tl_state == TrafficLightState::TL_STATE_GREEN_FLASHING ||
        it->second.tl_state == TrafficLightState::TL_STATE_YELLOW) {
      absl::StrAppend(&debug_info, ": ",
                      it->second.estimated_turn_red_time_left, "s");
    }
    absl::StrAppend(&debug_info, "\n");
  }

  absl::StrAppend(&debug_info, "The Tl Info is ",
                  (is_fresh_ ? "fresh\n" : "stale\n"));
  return debug_info;
}

void TlInfo::ToProto(TrafficLightInfoProto* proto) const {
  proto->Clear();
  if (is_empty_) return;

  const auto tl_color = [](TrafficLightState tl_state) -> TlColor {
    if (tl_state == TrafficLightState::TL_STATE_RED) {
      return TL_COLOR_RED;
    } else if (tl_state == TrafficLightState::TL_STATE_YELLOW ||
               tl_state == TrafficLightState::TL_STATE_YELLOW_FLASHING) {
      return TL_COLOR_YELLOW;
    } else if (tl_state == TrafficLightState::TL_STATE_GREEN ||
               tl_state == TrafficLightState::TL_STATE_GREEN_FLASHING) {
      return TL_COLOR_GREEN;
    } else {
      return TL_COLOR_UNKNOWN;
    }
  };
  const auto tl_flashing = [](TrafficLightState tl_state) -> bool {
    if (tl_state == TrafficLightState::TL_STATE_YELLOW_FLASHING ||
        tl_state == TrafficLightState::TL_STATE_GREEN_FLASHING) {
      return true;
    } else {
      return false;
    }
  };

  proto->set_lane_id(lane_id_.value());

  if (tl_control_type_ == TrafficLightControlType::SINGLE_DIRECTION) {
    const auto& single_tl_info = tls_.begin()->second;
    proto->mutable_isolated_tl_info()->set_traffic_light_id(
        single_tl_info.tl_id.value());
    proto->mutable_isolated_tl_info()->set_color(
        tl_color(single_tl_info.tl_state));
    proto->mutable_isolated_tl_info()->set_flashing(
        tl_flashing(single_tl_info.tl_state));
    proto->mutable_isolated_tl_info()->set_turn_red_time_left(
        single_tl_info.estimated_turn_red_time_left);
    proto->mutable_isolated_tl_info()->set_control_point_relative_s(
        control_point_relative_s_.front());
    if (single_tl_info.countdown.has_value()) {
      proto->mutable_isolated_tl_info()->set_countdown(
          *single_tl_info.countdown);
    }
  } else if (tl_control_type_ == TrafficLightControlType::LEFT_WAITING_AREA) {
    const auto& left_tl_info = tls_.find(TrafficLightDirection::LEFT)->second;
    const auto& straight_tl_info =
        tls_.find(TrafficLightDirection::STRAIGHT)->second;

    proto->mutable_left_tl_info()->set_traffic_light_id(
        left_tl_info.tl_id.value());
    proto->mutable_left_tl_info()->set_color(tl_color(left_tl_info.tl_state));
    proto->mutable_left_tl_info()->set_flashing(
        tl_flashing(left_tl_info.tl_state));
    proto->mutable_left_tl_info()->set_turn_red_time_left(
        left_tl_info.estimated_turn_red_time_left);
    proto->mutable_left_tl_info()->set_control_point_relative_s(
        control_point_relative_s_.back());
    if (left_tl_info.countdown.has_value()) {
      proto->mutable_left_tl_info()->set_countdown(*left_tl_info.countdown);
    }

    proto->mutable_straight_tl_info()->set_traffic_light_id(
        straight_tl_info.tl_id.value());
    proto->mutable_straight_tl_info()->set_color(
        tl_color(straight_tl_info.tl_state));
    proto->mutable_straight_tl_info()->set_flashing(
        tl_flashing(straight_tl_info.tl_state));
    proto->mutable_straight_tl_info()->set_turn_red_time_left(
        straight_tl_info.estimated_turn_red_time_left);
    proto->mutable_straight_tl_info()->set_control_point_relative_s(
        control_point_relative_s_.front());
    if (straight_tl_info.countdown.has_value()) {
      proto->mutable_straight_tl_info()->set_countdown(
          *straight_tl_info.countdown);
    }
  }

  proto->set_direction(direction_);
}

}  // namespace planner
}  // namespace qcraft
