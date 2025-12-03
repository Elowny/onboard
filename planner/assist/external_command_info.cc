#include "onboard/planner/assist/external_command_info.h"

#include "onboard/utils/time_util.h"

namespace qcraft::planner {

void ExternalCommandOutput::ToProto(
    PlannerExternalCommandStatusProto* proto) const {
  if (cruising_speed_limit.has_value()) {
    proto->set_cruising_speed_limit(*cruising_speed_limit);
  }

  if (current_lane_width.has_value()) {
    proto->set_current_lane_width(*current_lane_width);
  }

  if (current_lane_length.has_value()) {
    proto->set_current_lane_length(*current_lane_length);
  }

  if (current_lane_average_curvature_radius.has_value()) {
    proto->set_current_lane_average_curvature_radius(
        *current_lane_average_curvature_radius);
  }

  if (distance_to_toll.has_value()) {
    proto->set_distance_to_toll(*distance_to_toll);
  }

  if (distance_to_traffic_light.has_value()) {
    proto->set_distance_to_traffic_light(*distance_to_traffic_light);
  }

  if (is_av_overlap_boundary.has_value()) {
    proto->set_is_av_overlap_boundary(*is_av_overlap_boundary);
  }

  if (is_valid_both_side_boundary.has_value()) {
    proto->set_is_valid_both_side_boundary(*is_valid_both_side_boundary);
  }

  if (lane_path_lost.has_value()) {
    proto->set_lane_path_lost(*lane_path_lost);
  }

  if (is_av_in_emergency_lane.has_value()) {
    proto->set_is_av_in_emergency_lane(*is_av_in_emergency_lane);
  }

  if (apa_parking_spot_id.has_value()) {
    proto->set_apa_parking_spot_id(*apa_parking_spot_id);
  }

  proto->set_is_acc_engage_only(is_acc_engage_only);

  proto->set_is_route_lane_change_fail(is_route_lane_change_fail);
  proto->set_planner_to_router_command(planner_to_router_command);
}

void ExternalCommandOutput::FromProto(
    const PlannerExternalCommandStatusProto& proto) {
  if (proto.has_cruising_speed_limit()) {
    cruising_speed_limit = proto.cruising_speed_limit();
  }

  if (proto.has_current_lane_width()) {
    current_lane_width = proto.current_lane_width();
  }

  if (proto.has_current_lane_length()) {
    current_lane_length = proto.current_lane_length();
  }

  if (proto.has_current_lane_average_curvature_radius()) {
    current_lane_average_curvature_radius =
        proto.current_lane_average_curvature_radius();
  }

  if (proto.has_distance_to_toll()) {
    distance_to_toll = proto.distance_to_toll();
  }

  if (proto.has_distance_to_traffic_light()) {
    distance_to_traffic_light = proto.distance_to_traffic_light();
  }

  if (proto.has_is_av_overlap_boundary()) {
    is_av_overlap_boundary = proto.is_av_overlap_boundary();
  }

  if (proto.has_is_valid_both_side_boundary()) {
    is_valid_both_side_boundary = proto.is_valid_both_side_boundary();
  }

  if (proto.has_lane_path_lost()) {
    lane_path_lost = proto.lane_path_lost();
  }

  if (proto.has_is_av_in_emergency_lane()) {
    is_av_in_emergency_lane = proto.is_av_in_emergency_lane();
  }

  if (proto.has_is_route_lane_change_fail()) {
    is_route_lane_change_fail = proto.is_route_lane_change_fail();
  }

  if (proto.has_planner_to_router_command()) {
    planner_to_router_command = proto.planner_to_router_command();
  }

  if (proto.has_apa_parking_spot_id()) {
    apa_parking_spot_id = proto.apa_parking_spot_id();
  }

  if (proto.has_is_acc_engage_only()) {
    is_acc_engage_only = proto.is_acc_engage_only();
  }
}

bool ExternalCommandOutput::operator==(
    const ExternalCommandOutput& other) const {
  if (cruising_speed_limit != other.cruising_speed_limit ||
      current_lane_width != other.current_lane_width ||
      current_lane_length != other.current_lane_length ||
      current_lane_average_curvature_radius !=
          other.current_lane_average_curvature_radius ||
      distance_to_toll != other.distance_to_toll ||
      distance_to_traffic_light != other.distance_to_traffic_light ||
      is_av_overlap_boundary != other.is_av_overlap_boundary ||
      is_valid_both_side_boundary != other.is_valid_both_side_boundary ||
      lane_path_lost != other.lane_path_lost ||
      is_av_in_emergency_lane != other.is_av_in_emergency_lane ||
      is_route_lane_change_fail != other.is_route_lane_change_fail ||
      planner_to_router_command != other.planner_to_router_command ||
      apa_parking_spot_id != other.apa_parking_spot_id ||
      is_acc_engage_only != other.is_acc_engage_only) {
    return false;
  }

  return true;
}

PlannerExternalCommandStatusProto ExternalCommandStatus::ToProto() const {
  PlannerExternalCommandStatusProto proto;

  proto.mutable_enable_feature_status()->set_traffic_light_stopping_is_enabled(
      enable_traffic_light_stopping);

  proto.mutable_enable_feature_status()->set_pull_over_is_enabled(
      enable_pull_over);

  proto.mutable_enable_feature_status()->set_lc_objects_is_enabled(
      enable_lc_objects);

  proto.set_lane_change_style(lane_change_style);

  proto.mutable_left_blinker_override_status()->set_on(
      override_left_blinker_on);
  proto.mutable_right_blinker_override_status()->set_on(
      override_right_blinker_on);
  proto.mutable_emergency_blinker_override_status()->set_on(
      override_emergency_blinker_on);

  proto.set_enable_stop_polyline_stopping(enable_stop_polyline_stopping);

  if (override_door_open.has_value()) {
    proto.set_override_door_open(*override_door_open);
  }

  if (brake_to_stop.has_value()) {
    proto.set_brake_to_stop(*brake_to_stop);
  }

  proto.set_lcc_state(lcc_state);
  proto.set_alc_state(alc_state);
  proto.set_acc_state(acc_state);

  proto.set_lane_change_cmd(lane_change_command);

  if (plc_prepare_start_time.has_value()) {
    qcraft::ToProto(*plc_prepare_start_time,
                    proto.mutable_plc_prepare_start_time());
  }

  proto.set_following_distance_level(following_distance_level);

  if (noa_need_lane_change_confirmation.has_value()) {
    proto.set_noa_need_lane_change_confirmation(
        *noa_need_lane_change_confirmation);
  }
  if (alc_confirmation.has_value()) {
    proto.set_alc_confirmation(*alc_confirmation);
  }

  if (apa_parking_spot_id.has_value()) {
    proto.set_apa_parking_spot_id(*apa_parking_spot_id);
  }

  output.ToProto(&proto);

  return proto;
}

void ExternalCommandStatus::FromProto(
    const PlannerExternalCommandStatusProto& proto) {
  if (proto.has_enable_feature_status()) {
    if (proto.enable_feature_status().has_pull_over_is_enabled()) {
      enable_pull_over = proto.enable_feature_status().pull_over_is_enabled();
    }

    if (proto.enable_feature_status().has_traffic_light_stopping_is_enabled()) {
      enable_traffic_light_stopping =
          proto.enable_feature_status().traffic_light_stopping_is_enabled();
    }

    if (proto.enable_feature_status().has_lc_objects_is_enabled()) {
      enable_lc_objects = proto.enable_feature_status().lc_objects_is_enabled();
    }
  }

  if (proto.has_lane_change_style()) {
    lane_change_style = proto.lane_change_style();
  }

  if (proto.has_left_blinker_override_status()) {
    override_left_blinker_on = proto.left_blinker_override_status().on();
  }
  if (proto.has_right_blinker_override_status()) {
    override_right_blinker_on = proto.right_blinker_override_status().on();
  }
  if (proto.has_emergency_blinker_override_status()) {
    override_emergency_blinker_on =
        proto.emergency_blinker_override_status().on();
  }

  if (proto.has_enable_stop_polyline_stopping()) {
    enable_stop_polyline_stopping = proto.enable_stop_polyline_stopping();
  }

  if (proto.has_override_door_open()) {
    override_door_open = proto.override_door_open();
  }
  if (proto.has_brake_to_stop()) {
    brake_to_stop = proto.brake_to_stop();
  }

  if (proto.has_lcc_state()) {
    lcc_state = proto.lcc_state();
  }

  if (proto.has_alc_state()) {
    alc_state = proto.alc_state();
  }

  if (proto.has_acc_state()) {
    acc_state = proto.acc_state();
  }

  if (proto.has_lane_change_cmd()) {
    lane_change_command = proto.lane_change_cmd();
  }
  if (proto.has_plc_prepare_start_time()) {
    plc_prepare_start_time = qcraft::FromProto(proto.plc_prepare_start_time());
  } else {
    plc_prepare_start_time = std::nullopt;
  }

  if (proto.has_following_distance_level()) {
    following_distance_level = proto.following_distance_level();
  }
  if (proto.has_alc_confirmation()) {
    alc_confirmation = proto.alc_confirmation();
  } else {
    alc_confirmation = std::nullopt;
  }

  if (proto.has_noa_need_lane_change_confirmation()) {
    noa_need_lane_change_confirmation =
        proto.noa_need_lane_change_confirmation();
  }

  if (proto.has_apa_parking_spot_id()) {
    apa_parking_spot_id = proto.apa_parking_spot_id();
  }

  output.FromProto(proto);
}

bool ExternalCommandStatus::operator==(
    const ExternalCommandStatus& other) const {
  if (enable_pull_over != other.enable_pull_over ||
      enable_traffic_light_stopping != other.enable_traffic_light_stopping ||
      enable_lc_objects != other.enable_lc_objects ||
      override_left_blinker_on != other.override_left_blinker_on ||
      override_right_blinker_on != other.override_right_blinker_on ||
      override_emergency_blinker_on != other.override_emergency_blinker_on ||
      enable_stop_polyline_stopping != other.enable_stop_polyline_stopping ||
      override_door_open != other.override_door_open ||
      brake_to_stop != other.brake_to_stop || lcc_state != other.lcc_state ||
      alc_state != other.alc_state || acc_state != other.acc_state ||
      lane_change_style != other.lane_change_style ||
      lcc_cruising_speed_limit != other.lcc_cruising_speed_limit ||
      noa_cruising_speed_limit != other.noa_cruising_speed_limit ||
      noa_need_lane_change_confirmation !=
          other.noa_need_lane_change_confirmation ||
      following_distance_level != other.following_distance_level ||
      alc_confirmation != other.alc_confirmation ||
      lane_change_command != other.lane_change_command ||
      plc_prepare_start_time != other.plc_prepare_start_time ||
      apa_parking_spot_id != other.apa_parking_spot_id ||
      !(output == other.output)) {
    return false;
  }

  return true;
}

}  // namespace qcraft::planner
