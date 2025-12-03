#include "onboard/planner/planner_state.h"

#include <map>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "google/protobuf/message.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/assist/proto/online_map_drift.pb.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/scheduler/proto/smooth_reference_line.pb.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/time_util.h"
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

namespace qcraft::planner {

void PlannerState::FromProto(const PlannerStateProto& proto) {
  header = proto.header();

  planner_frame_seq_num = proto.planner_frame_seq_num();

  for (const auto& e : proto.yellow_light_observations()) {
    yellow_light_observations[mapping::ElementId(e.first)].FromProto(e.second);
  }

  previous_trajectory = proto.previous_trajectory();
  previous_st_path_global_including_past.clear();
  for (const auto& e : proto.previous_st_path_global_including_past()) {
    previous_st_path_global_including_past.push_back(e);
  }

  last_audio_alert_time = qcraft::FromProto(proto.last_audio_alert_time());

  parking_brake_release_time =
      qcraft::FromProto(proto.parking_brake_release_time());

  lane_change_state = proto.lane_change_state();

  planner_skip_counter = proto.planner_skip_counter();

  input_seq_num = proto.input_seq_num();

  previous_autonomy_state = proto.previous_autonomy_state();

  previous_trajectory_plan_counter = proto.previous_trajectory_plan_counter();

  version = proto.version();

  last_door_override_time = proto.last_door_override_time();

  decider_state = proto.decider_state();
  initializer_state = proto.initializer_state();

  if (proto.has_selected_trajectory_optimizer_state()) {
    selected_trajectory_optimizer_state_proto =
        proto.selected_trajectory_optimizer_state();
  } else {
    selected_trajectory_optimizer_state_proto = std::nullopt;
  }

  st_planner_object_trajectories = proto.st_planner_object_trajectories();
  current_time = absl::FromUnixMicros(proto.current_time());

  route_update_id = proto.route_update_id();

  // Freespace planner state.
  freespace_planner_state = proto.freespace_planner_state();

  mission_stage = proto.mission_stage();

  // Parallel planner state.
  // prev_target_lane_path restore later because it depends semantic map.
  // After map patch, it can restore here.
  prev_length_along_route = proto.prev_length_along_route();
  prev_max_reach_length = proto.prev_max_reach_length();
  station_anchor.FromProto(proto.station_anchor());

  prev_route_sections =
      RouteSections::BuildFromProto(proto.prev_route_sections());

  // Reference line smooth.
  prev_smooth_state = proto.prev_smooth_state();

  // Plan task queue
  for (const auto& task_proto : proto.plan_task_queue()) {
    plan_task_queue.emplace_back(task_proto);
  }

  // Previous trajectory end info.
  if (proto.has_previous_trajectory_end_info()) {
    prev_traj_end_info = proto.previous_trajectory_end_info();
  } else {
    prev_traj_end_info = std::nullopt;
  }

  stopped_at_route_end = proto.stopped_at_route_end();

  assist_plan_state = proto.assist_plan_state();

  selector_state.FromProto(proto.selector_state());

  tja_state.FromProto(proto.tja_state());

  async_planner_state.FromProto(proto.async_planner_state());

  previously_triggered_aeb = proto.previously_triggered_aeb();

  if (proto.has_prev_online_map_id()) {
    prev_online_map_id = proto.prev_online_map_id();
  }

  if (proto.has_history_transform()) {
    history_transform = proto.history_transform();
  }

  online_map_drift_buffer.clear();
  for (const auto& drift : proto.online_map_drift_buffer()) {
    online_map_drift_buffer.push_back(
        qcraft::FromProto(drift.time()),
        PiecewiseLinearFunctionFromProto(drift.plf()));
  }

  object_history_map.clear();
  for (const auto& buffer : proto.objects_motion_state_buffer()) {
    for (const auto& state : buffer.second.states()) {
      ObjectMotionState object_motion_state;
      object_motion_state.heading = state.heading();
      object_motion_state.accel.FromProto(state.accel());
      object_motion_state.vel.FromProto(state.vel());
      object_motion_state.pos.FromProto(state.pos());
      object_history_map[buffer.first].push_back(
          absl::FromUnixMicros(state.time()), std::move(object_motion_state));
    }
  }

  if (proto.has_hd_map_state()) {
    hd_map_state->load_distance = proto.hd_map_state().load_distance();
    hd_map_state->has_destination = proto.hd_map_state().has_destination();
  } else {
    hd_map_state = std::nullopt;
  }
  prev_collision_warning_request = proto.prev_collision_warning_request();
}

// If FromProto/ToProto slow, use move instead
void PlannerState::ToProto(PlannerStateProto* proto) const {
  SCOPED_QTRACE("PlannerState::ToProto");
  proto->Clear();

  *proto->mutable_header() = header;

  proto->set_planner_frame_seq_num(planner_frame_seq_num);

  for (const auto& e : yellow_light_observations) {
    e.second.ToProto(
        &(*proto->mutable_yellow_light_observations())[e.first.value()]);
  }

  *proto->mutable_previous_trajectory() = previous_trajectory;
  for (const auto& e : previous_st_path_global_including_past) {
    auto* point = proto->add_previous_st_path_global_including_past();
    *point = e;
  }

  qcraft::ToProto(last_audio_alert_time,
                  proto->mutable_last_audio_alert_time());

  if (parking_brake_release_time < absl::UnixEpoch()) {
    qcraft::ToProto(absl::UnixEpoch(),
                    proto->mutable_parking_brake_release_time());
  } else {
    qcraft::ToProto(parking_brake_release_time,
                    proto->mutable_parking_brake_release_time());
  }

  *proto->mutable_lane_change_state() = lane_change_state;

  proto->set_planner_skip_counter(planner_skip_counter);

  *proto->mutable_input_seq_num() = input_seq_num;

  *proto->mutable_previous_autonomy_state() = previous_autonomy_state;

  proto->set_previous_trajectory_plan_counter(previous_trajectory_plan_counter);

  proto->set_version(version);

  proto->set_last_door_override_time(last_door_override_time);

  *proto->mutable_decider_state() = decider_state;

  *proto->mutable_initializer_state() = initializer_state;

  prev_lane_path_before_lc.ToProto(proto->mutable_prev_lane_path_before_lc());
  *proto->mutable_st_planner_object_trajectories() =
      st_planner_object_trajectories;
  proto->set_current_time(absl::ToUnixMicros(current_time));
  proto->set_route_update_id(route_update_id);
  *proto->mutable_freespace_planner_state() = freespace_planner_state;

  *proto->mutable_mission_stage() = mission_stage;

  // Parallel planner state.
  prev_target_lane_path.ToProto(proto->mutable_prev_target_lane_path());
  proto->set_prev_length_along_route(prev_length_along_route);
  proto->set_prev_max_reach_length(prev_max_reach_length);
  station_anchor.ToProto(proto->mutable_station_anchor());
  prev_route_sections.ToProto(proto->mutable_prev_route_sections());

  // Reference line smooth.
  proto->set_prev_smooth_state(prev_smooth_state);
  proto->mutable_smooth_result_map()->mutable_lane_id_vec()->Reserve(
      smooth_result_map.smoothed_result_map().size());
  for (const auto& it : smooth_result_map.smoothed_result_map()) {
    auto* lane_id_vec = proto->mutable_smooth_result_map()->add_lane_id_vec();
    for (const auto id : it.first) {
      lane_id_vec->add_lane_id(id.value());
    }
  }

  // For teleop lane change.
  preferred_lane_path.ToProto(proto->mutable_preferred_lane_path());

  // Plan task queue
  proto->mutable_plan_task_queue()->Reserve(plan_task_queue.size());
  int index = 0;
  for (const auto& task : plan_task_queue) {
    task.ToProto(proto->add_plan_task_queue(), index);
    index++;
  }

  // Selected trajectory optimizer state.
  if (selected_trajectory_optimizer_state_proto.has_value()) {
    *proto->mutable_selected_trajectory_optimizer_state() =
        (*selected_trajectory_optimizer_state_proto);
  } else {
    proto->mutable_selected_trajectory_optimizer_state()->Clear();
  }

  // Previous trajectory end info.
  if (prev_traj_end_info.has_value()) {
    proto->mutable_previous_trajectory_end_info()->CopyFrom(
        *prev_traj_end_info);
  } else {
    proto->mutable_previous_trajectory_end_info()->Clear();
  }

  proto->set_stopped_at_route_end(stopped_at_route_end);

  *proto->mutable_assist_plan_state() = assist_plan_state;

  selector_state.ToProto(proto->mutable_selector_state());

  tja_state.ToProto(proto->mutable_tja_state());

  async_planner_state.ToProto(proto->mutable_async_planner_state());

  proto->set_previously_triggered_aeb(previously_triggered_aeb);

  if (prev_online_map_id != kInvalidOnlineMapId) {
    proto->set_prev_online_map_id(prev_online_map_id);
  }
  *proto->mutable_history_transform() = history_transform;

  if (!online_map_drift_buffer.empty()) {
    auto* buffer = proto->mutable_online_map_drift_buffer();
    buffer->Reserve(online_map_drift_buffer.size());
    for (const auto& drift : online_map_drift_buffer) {
      auto* plf = buffer->Add();
      qcraft::ToProto(drift.first, plf->mutable_time());
      PiecewiseLinearFunctionToProto(drift.second, plf->mutable_plf());
    }
  }
  if (!object_history_map.empty()) {
    auto& buffer = *proto->mutable_objects_motion_state_buffer();
    for (const auto& [id, objects_buffer] : object_history_map) {
      buffer[id].mutable_states()->Reserve(objects_buffer.size());
      for (const auto& [time, object_buffer] : objects_buffer) {
        auto* object_motion_state = buffer[id].mutable_states()->Add();
        object_motion_state->set_time(absl::ToUnixMicros(time));
        object_motion_state->set_heading(object_buffer.heading);
        object_buffer.accel.ToProto(object_motion_state->mutable_accel());
        object_buffer.vel.ToProto(object_motion_state->mutable_vel());
        object_buffer.pos.ToProto(object_motion_state->mutable_pos());
      }
    }
  }

  if (hd_map_state.has_value()) {
    auto* mutable_hd_map_state = proto->mutable_hd_map_state();
    mutable_hd_map_state->Clear();
    mutable_hd_map_state->set_load_distance(hd_map_state->load_distance);
    mutable_hd_map_state->set_has_destination(hd_map_state->has_destination);
  }
  proto->set_prev_collision_warning_request(prev_collision_warning_request);
}

bool PlannerState::operator==(const PlannerState& other) const {
  if (previous_trajectory != other.previous_trajectory ||
      last_audio_alert_time != other.last_audio_alert_time ||
      parking_brake_release_time != other.parking_brake_release_time ||
      planner_skip_counter != other.planner_skip_counter ||
      !ProtoEquals(lane_change_state, other.lane_change_state) ||
      !ProtoEquals(input_seq_num, other.input_seq_num) ||
      previous_autonomy_state != other.previous_autonomy_state ||
      previous_trajectory_plan_counter !=
          other.previous_trajectory_plan_counter ||
      last_door_override_time != other.last_door_override_time ||
      prev_lane_path_before_lc != other.prev_lane_path_before_lc ||
      !ProtoEquals(initializer_state, other.initializer_state) ||
      !ProtoEquals(st_planner_object_trajectories,
                   other.st_planner_object_trajectories) ||
      !ProtoEquals(freespace_planner_state, other.freespace_planner_state) ||
      prev_target_lane_path != other.prev_target_lane_path ||
      prev_length_along_route != other.prev_length_along_route ||
      station_anchor != other.station_anchor ||
      preferred_lane_path != other.preferred_lane_path ||
      prev_route_sections != other.prev_route_sections ||
      prev_smooth_state != other.prev_smooth_state ||
      current_time != other.current_time ||
      async_planner_state != other.async_planner_state ||
      previously_triggered_aeb != other.previously_triggered_aeb ||
      !ProtoEquals(history_transform, other.history_transform) ||
      prev_collision_warning_request != other.prev_collision_warning_request) {
    return false;
  }

  return true;
}

bool PlannerState::Upgrade() {
  // trajectory_start_timestamp is set improperly, compatible for old runs";
  if (previous_trajectory.trajectory_start_timestamp() < 1E-6) {
    previous_trajectory.set_trajectory_start_timestamp(header.timestamp() /
                                                       1E6);
  }
  return true;
}

std::string PlannerState::DebugString() const {
  PlannerStateProto proto;
  ToProto(&proto);
  return proto.DebugString();
}

}  // namespace qcraft::planner
