#include "onboard/planner/assist/tja_state.h"

#include <algorithm>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"

namespace qcraft {
namespace planner {

void TjaState::FromProto(const TjaStateProto& proto) {
  Reset();
  center_line.reserve(proto.center_line_size());
  for (int i = 0; i < proto.center_line_size(); ++i) {
    const auto& point = proto.center_line(i);
    center_line.emplace_back(point.x(), point.y());
  }

  target_obs_ids.reserve(proto.target_obs_ids_size());
  for (int i = 0; i < proto.target_obs_ids_size(); ++i) {
    const auto& id = proto.target_obs_ids(i);
    target_obs_ids.emplace_back(id);
  }

  potential_obs_ids.reserve(proto.potential_obs_ids_size());
  for (int i = 0; i < proto.potential_obs_ids_size(); ++i) {
    const auto& id = proto.potential_obs_ids(i);
    potential_obs_ids.emplace_back(id);
  }

  planner_use_tja_map = proto.planner_use_tja_map();

  planner_center_line.reserve(proto.planner_center_line_size());
  for (int i = 0; i < proto.planner_center_line_size(); ++i) {
    const auto& point = proto.planner_center_line(i);
    planner_center_line.emplace_back(point.x(), point.y());
  }
  exit_lane_ids.reserve(proto.exit_lane_ids_size());
  for (int i = 0; i < proto.exit_lane_ids_size(); ++i) {
    exit_lane_ids.push_back(mapping::ElementId(proto.exit_lane_ids(i)));
  }

  exit_counter = proto.exit_counter();
  update_id = proto.update_id();
}

void TjaState::ToProto(TjaStateProto* proto) const {
  proto->Clear();
  proto->mutable_center_line()->Reserve(center_line.size());
  for (const auto& point : center_line) {
    Vec2dToProto(point, proto->add_center_line());
  }
  proto->mutable_target_obs_ids()->Reserve(target_obs_ids.size());
  for (const auto& obs_id : target_obs_ids) {
    *proto->add_target_obs_ids() = obs_id;
  }
  proto->mutable_potential_obs_ids()->Reserve(potential_obs_ids.size());
  for (const auto& obs_id : potential_obs_ids) {
    *proto->add_potential_obs_ids() = obs_id;
  }
  proto->mutable_planner_center_line()->Reserve(planner_center_line.size());
  for (const auto& point : planner_center_line) {
    Vec2dToProto(point, proto->add_planner_center_line());
  }
  proto->mutable_exit_lane_ids()->Reserve(exit_lane_ids.size());
  for (const auto& lane_id : exit_lane_ids) {
    proto->add_exit_lane_ids(lane_id.value());
  }
  proto->set_exit_counter(exit_counter);
  proto->set_planner_use_tja_map(planner_use_tja_map);
  proto->set_update_id(update_id);
}

void TjaState::Reset() {
  center_line.clear();
  obs_history_pos.clear();
  target_obs_ids.clear();
  left_boundary.clear();
  right_boundary.clear();
  planner_center_line.clear();
  potential_obs_ids.clear();
  exit_lane_ids.clear();
  planner_use_tja_map = false;
  exit_counter = 0;
  update_id = 0;
}

}  // namespace planner
}  // namespace qcraft
