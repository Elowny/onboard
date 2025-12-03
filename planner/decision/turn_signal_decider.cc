#include "onboard/planner/decision/turn_signal_decider.h"

#include <float.h>

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kPreviewDirectionDist = 15.0;     // m.
constexpr double kPreviewMergeForkTime = 2.5;      // s.
constexpr double kPreviewMergeForkMinDist = 30.0;  // m.

bool HasMapDictatedTurnSignal(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePath& lane_path,
                              TurnSignal* signal) {
  const mapping::ElementId cur_lane_id = lane_path.front().lane_id();
  const double cur_lane_fraction = lane_path.front().fraction();
  const auto* cur_lane_proto_ptr = psmm.FindLaneByIdOrNull(cur_lane_id);
  if (cur_lane_proto_ptr == nullptr) return false;

  const bool left_on =
      cur_lane_proto_ptr->has_require_left_turn_signal() &&
      (!cur_lane_proto_ptr->require_left_turn_signal().has_start_fraction() ||
       cur_lane_fraction >=
           cur_lane_proto_ptr->require_left_turn_signal().start_fraction()) &&
      (!cur_lane_proto_ptr->require_left_turn_signal().has_end_fraction() ||
       cur_lane_fraction <=
           cur_lane_proto_ptr->require_left_turn_signal().end_fraction());
  const bool right_on =
      cur_lane_proto_ptr->has_require_right_turn_signal() &&
      (!cur_lane_proto_ptr->require_right_turn_signal().has_start_fraction() ||
       cur_lane_fraction >=
           cur_lane_proto_ptr->require_right_turn_signal().start_fraction()) &&
      (!cur_lane_proto_ptr->require_right_turn_signal().has_end_fraction() ||
       cur_lane_fraction <=
           cur_lane_proto_ptr->require_right_turn_signal().end_fraction());

  if (left_on && right_on) {
    *signal = TURN_SIGNAL_EMERGENCY;
    return true;
  } else if (left_on) {
    *signal = TURN_SIGNAL_LEFT;
    return true;
  } else if (right_on) {
    *signal = TURN_SIGNAL_RIGHT;
    return true;
  }

  return false;
}

std::optional<TurnSignal> ComputeDirectionSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  const auto preview_lane_id =
      lane_path.ArclengthToLanePoint(kPreviewDirectionDist).lane_id();
  const auto* preview_lane_proto_ptr = psmm.FindLaneByIdOrNull(preview_lane_id);
  // There is no direction signal.
  if (preview_lane_proto_ptr == nullptr ||
      !preview_lane_proto_ptr->has_direction()) {
    return std::nullopt;
  }

  const auto& direction = preview_lane_proto_ptr->direction();

  switch (direction) {
    case mapping::LaneProto::STRAIGHT:
      return std::nullopt;
    case mapping::LaneProto::LEFT_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
    case mapping::LaneProto::RIGHT_TURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_RIGHT);
    case mapping::LaneProto::UTURN:
      return std::make_optional<TurnSignal>(TURN_SIGNAL_LEFT);
  }
  return std::nullopt;
}

TurnSignal DecideLaneChangingTurnSignal(const FrenetBox& ego_sl_box,
                                        const DrivePassage& drive_passage) {
  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(ego_sl_box.center_s());
  const double lane_boundary_right_offset =
      std::max(boundaries.right->lat_offset, -kMaxHalfLaneWidth);
  const double lane_boundary_left_offset =
      std::min(boundaries.left->lat_offset, kMaxHalfLaneWidth);

  const double ego_center_l = ego_sl_box.center_l();
  if (lane_boundary_right_offset < ego_center_l &&
      ego_center_l < lane_boundary_left_offset) {
    return TURN_SIGNAL_NONE;
  }

  return ego_center_l <= lane_boundary_right_offset ? TURN_SIGNAL_LEFT
                                                    : TURN_SIGNAL_RIGHT;
}

absl::StatusOr<TurnSignal> SignalFromRealLaneAside(
    const PlannerSemanticMapManager& psmm, const mapping::LaneInfo& lane_info,
    const mapping::LaneInfo& target_lane_info) {
  const auto target_sec_id = target_lane_info.section_id;
  const Vec2d target_end_point = target_lane_info.LerpPointFromFraction(1.0);

  double min_sqr_dist = DBL_MAX;
  Vec2d nearest_end_point;
  for (const auto next_lane_id : lane_info.outgoing_lanes()) {
    if (next_lane_id == target_lane_info.id) continue;
    SMM_ASSIGN_LANE_OR_ERROR(next_lane_info, psmm, next_lane_id);
    if (next_lane_info.section_id != target_sec_id) continue;

    if (!next_lane_info.IsVirtual()) {
      const Vec2d next_end_point = next_lane_info.LerpPointFromFraction(1.0);
      const double sqr_dist = next_end_point.DistanceSquareTo(target_end_point);
      if (sqr_dist < min_sqr_dist) {
        min_sqr_dist = sqr_dist;
        nearest_end_point = next_end_point;
      }
    }
  }
  if (min_sqr_dist < DBL_MAX) {
    const Vec2d fork_point = lane_info.LerpPointFromFraction(1.0);
    return (nearest_end_point - fork_point)
                       .CrossProd(target_end_point - fork_point) > 0.0
               ? TURN_SIGNAL_LEFT
               : TURN_SIGNAL_RIGHT;
  }
  return absl::NotFoundError("No neighboring real lane found.");
}

absl::StatusOr<TurnSignal> ComputeForkSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id, double ego_v) {
  const double preview_dist =
      std::max(kPreviewMergeForkMinDist, ego_v * kPreviewMergeForkTime);
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();
  const auto stop_lane_id = redlight_lane_id.has_value()
                                ? *redlight_lane_id
                                : mapping::kInvalidElementId;

  for (int i = 0; i < lane_path.lane_ids().size(); ++i) {
    const auto lane_id = lane_path.lane_id(i);
    if (lane_id == far_lane_id) break;

    const auto next_lane_id = lane_path.lane_id(i + 1);
    if (next_lane_id == stop_lane_id) break;

    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.outgoing_lanes().size() < 2) continue;

    SMM_ASSIGN_LANE_OR_ERROR(next_lane_info, psmm, next_lane_id);
    if (next_lane_info.IsVirtual()) {
      const auto real_lane_or =
          SignalFromRealLaneAside(psmm, lane_info, next_lane_info);
      if (real_lane_or.ok()) {
        return *real_lane_or;
      }
    }

    constexpr double kQueryDistAfterForkPoint = 50.0;  // m.
    constexpr double kForkLatDistThreshold = 1.0;      // m.
    const Vec2d fork_point = lane_info.LerpPointFromFraction(1.0);
    const Vec2d fork_point_tangent = lane_info.GetTangent(1.0);
    absl::flat_hash_map<mapping::ElementId, double> lat_dist_map;
    bool is_rest_lanes_virtual = true;
    for (const auto outgoing_id : lane_info.outgoing_lanes()) {
      SMM_ASSIGN_LANE_OR_ERROR(outgoing_lane_info, psmm, outgoing_id);
      const double query_fraction =
          std::min(1.0, kQueryDistAfterForkPoint / outgoing_lane_info.length());
      const Vec2d outgoing_query_point =
          outgoing_lane_info.LerpPointFromFraction(query_fraction);
      lat_dist_map[outgoing_id] =
          fork_point_tangent.CrossProd(outgoing_query_point - fork_point);
      if (outgoing_id != next_lane_id && !outgoing_lane_info.IsVirtual()) {
        is_rest_lanes_virtual = false;
      }
    }
    if (is_rest_lanes_virtual) break;
    if (std::abs(lat_dist_map[next_lane_id]) < kForkLatDistThreshold) break;

    auto outgoing_lane_ids = lane_info.outgoing_lanes();
    std::stable_sort(outgoing_lane_ids.begin(), outgoing_lane_ids.end(),
                     [&lat_dist_map](const mapping::ElementId id1,
                                     const mapping::ElementId id2) {
                       return lat_dist_map[id1] < lat_dist_map[id2];
                     });
    for (int j = 0; j < outgoing_lane_ids.size(); ++j) {
      if (outgoing_lane_ids[j] != next_lane_id) continue;

      if (j * 2 + 1 < outgoing_lane_ids.size()) return TURN_SIGNAL_RIGHT;
      if (j * 2 + 1 > outgoing_lane_ids.size()) return TURN_SIGNAL_LEFT;
      break;
    }
    break;
  }

  return absl::CancelledError("No need to turn fork signal.");
}

absl::StatusOr<TurnSignal> TurnOnSignalForEntranceLane(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::optional<double> distance_to_merge_triangle_area = std::nullopt;
  double accumulate_s = 0.0;
  for (const auto& seg : lane_path) {
    const auto& lane_id = seg.lane_id;
    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.proto->type() == mapping::LaneProto::ENTRANCE) {
      if (!distance_to_merge_triangle_area.has_value()) {
        distance_to_merge_triangle_area = accumulate_s;
      }
      if (lane_info.IsMerging()) break;
    } else {
      distance_to_merge_triangle_area.reset();
    }
    accumulate_s += seg.length();
  }
  if (accumulate_s == lane_path.length()) {
    return absl::NotFoundError("Can not find MERGE & ENTRANCE lane.");
  }
  constexpr double kTurnLightDistanceThresholdToEntrance = 50.0;  // m.
  if (distance_to_merge_triangle_area.has_value() &&
      *distance_to_merge_triangle_area <
          kTurnLightDistanceThresholdToEntrance) {
    return TURN_SIGNAL_LEFT;
  }
  return absl::NotFoundError("Far from merge triangle area, not turn signal.");
}

absl::StatusOr<TurnSignal> TurnOnMergeSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LaneInfo& lane_info) {
  mapping::ElementId other_id = mapping::kInvalidElementId;
  for (const auto& interaction : lane_info.proto->interactions()) {
    if (interaction.geometric_configuration() !=
        mapping::LaneInteractionProto::MERGE) {
      continue;
    }
    if (interaction.reaction_rule() ==
            mapping::LaneInteractionProto::YIELD_MERGE ||
        interaction.reaction_rule() == mapping::LaneInteractionProto::YIELD) {
      other_id = mapping::ElementId(interaction.other_lane_id());
      break;
    }
  }
  if (other_id == mapping::kInvalidElementId) {
    return absl::NotFoundError(
        "Can not find the merging lane, please check the map.");
  }

  constexpr double kQueryDistBeforeMergePoint = 10.0;  // m.
  const double query_fraction =
      std::max(0.0, (lane_info.length() - kQueryDistBeforeMergePoint) /
                        lane_info.length());
  const Vec2d this_lane_pos = lane_info.LerpPointFromFraction(query_fraction);

  SMM_ASSIGN_LANE_OR_ERROR(other_lane_info, psmm, other_id);
  const Vec2d other_lane_tangent = other_lane_info.GetTangent(query_fraction);
  const Vec2d other_lane_pos =
      other_lane_info.LerpPointFromFraction(query_fraction);

  return other_lane_tangent.CrossProd(this_lane_pos - other_lane_pos) < 0.0
             ? TURN_SIGNAL_LEFT
             : TURN_SIGNAL_RIGHT;
}

absl::StatusOr<TurnSignal> ComputeMergeSignal(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double ego_v) {
  const auto turn_signal_for_entrance_lane_or =
      TurnOnSignalForEntranceLane(psmm, lane_path);
  if (turn_signal_for_entrance_lane_or.ok()) {
    return *turn_signal_for_entrance_lane_or;
  }

  const double preview_dist =
      std::max(kPreviewMergeForkMinDist, ego_v * kPreviewMergeForkTime);
  const auto far_lane_id =
      lane_path.ArclengthToLanePoint(preview_dist).lane_id();

  for (const auto lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_ERROR(lane_info, psmm, lane_id);
    if (lane_info.IsMerging()) {
      return TurnOnMergeSignal(psmm, lane_info);
    }
    if (lane_id == far_lane_id) break;
  }
  return absl::CancelledError("No need to turn on merge signal.");
}

}  // namespace

// Planner 3.0 turn signal decider
TurnSignalResult DecideTurnSignal(
    const PlannerSemanticMapManager& psmm, TurnSignal route_signal,
    TurnSignal pre_lane_change_signal,
    const mapping::LanePath& current_lane_path,
    const std::optional<mapping::ElementId>& redlight_lane_id,
    const LaneChangeStateProto& lc_state,
    const ExternalCommandStatus& ext_cmd_status,
    const DrivePassage& drive_passage, const FrenetBox& ego_sl_box,
    const TurnSignalResult& planned_result, const PoseProto& ego_pose) {
  SCOPED_QTRACE("DecideTurnSignal");

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

  if (route_signal != TURN_SIGNAL_NONE) {
    return {route_signal, route_signal == TURN_SIGNAL_LEFT
                              ? STARTING_TURN_SIGNAL
                              : PULL_OVER_TURN_SIGNAL};
  }

  // Map dictated turn signal
  TurnSignal map_signal;
  if (HasMapDictatedTurnSignal(psmm, current_lane_path, &map_signal)) {
    return {map_signal, MAP_DICTATED_TURN_SIGNAL};
  }

  // Direction signal
  std::optional<TurnSignal> direction_signal = std::nullopt;
  direction_signal = ComputeDirectionSignal(psmm, current_lane_path);
  if (direction_signal.has_value()) {
    return {direction_signal.value(), TURNING_TURN_SIGNAL};
  }

  // Lane change signal
  if (lc_state.stage() == LCS_PAUSE) {
    return {lc_state.lc_left() ? TURN_SIGNAL_LEFT : TURN_SIGNAL_RIGHT,
            LANE_CHANGE_TURN_SIGNAL};
  }
  const auto lane_changing_signal =
      DecideLaneChangingTurnSignal(ego_sl_box, drive_passage);
  if (lane_changing_signal != TURN_SIGNAL_NONE) {
    return {lane_changing_signal, LANE_CHANGE_TURN_SIGNAL};
  }

  // Planned signal
  // TODO(weijun): Merge with lane change signal.
  if (planned_result.signal != TurnSignal::TURN_SIGNAL_NONE) {
    return planned_result;
  }

  // Merge or fork preview signal
  const double ego_v =
      Hypot(ego_pose.vel_smooth().x(), ego_pose.vel_smooth().y());
  const auto merge_signal = ComputeMergeSignal(psmm, current_lane_path, ego_v);
  if (merge_signal.ok()) {
    return {merge_signal.value(), MERGE_TURN_SIGNAL};
  }
  const auto fork_signal =
      ComputeForkSignal(psmm, current_lane_path, redlight_lane_id, ego_v);
  if (fork_signal.ok()) {
    return {fork_signal.value(), FORK_TURN_SIGNAL};
  }

  // Prepare lane change signal
  if (pre_lane_change_signal != TurnSignal::TURN_SIGNAL_NONE) {
    return {pre_lane_change_signal, PREPARE_LANE_CHANGE_TURN_SIGNAL};
  }

  return {TURN_SIGNAL_NONE, TURN_SIGNAL_OFF};
}

}  // namespace planner
}  // namespace qcraft
