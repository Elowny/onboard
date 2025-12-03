#include "onboard/planner/selector/selector_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <float.h>

#include <algorithm>
#include <deque>
#include <limits>
#include <optional>
#include <ostream>
#include <string_view>
#include <tuple>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

#define btoa(x) ((x) ? "true" : "false")
#define btof(x) ((x) ? 1.0 : 0.0)
namespace qcraft {
namespace planner {
namespace {

constexpr double kContinuousBoundaryMaxLatOffset = kDefaultHalfLaneWidth;
constexpr double kOccupiedCrossBoundaryFactor = 1.1;
constexpr double kFullCrossBoundaryFactor = 1.0;
constexpr double kDefaultCrossBoundaryFactor = 0.8;
constexpr double kCheckIsLeaderObjectStartTimeForLc = 2.5;  // s
constexpr double kCheckIsLeaderObjectStartTimeForLk = 1.5;  // s
constexpr double kCheckIsLeaderObjectEndTimeForLc = 4.5;    // s
constexpr double kCheckIsLeaderObjectEndTimeForLk = 4.0;    // s

struct PreFilterStats {
  int max_lc_num = 0;
  double min_driving_dist = DBL_MAX;
  double left_blocked_rate = 0.0;
  double right_blocked_rate = 0.0;
};

bool IsBranchBlockedByObs(const DrivePassage& passage, const FrenetBox& ego_box,
                          const FrenetBox& box, const double lat_buffer,
                          bool is_stationary, std::optional<bool> lc_left) {
  const auto [right_boundary, left_boundary] =
      passage.QueryEnclosingLaneBoundariesAtS(box.center_s());
  double boundary_right_l =
      right_boundary.has_value()
          ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
          : -kMaxHalfLaneWidth;
  double boundary_left_l =
      left_boundary.has_value()
          ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
          : kMaxHalfLaneWidth;
  if (lc_left.has_value() && is_stationary) {
    if (*lc_left) {
      boundary_right_l = std::min(boundary_right_l, ego_box.l_max - lat_buffer);
    } else {
      boundary_left_l = std::max(boundary_left_l, ego_box.l_min + lat_buffer);
    }
  }
  const double l_max = std::clamp(box.l_max, boundary_right_l, boundary_left_l);
  const double l_min = std::clamp(box.l_min, boundary_right_l, boundary_left_l);

  if (std::min(boundary_left_l - l_min, l_max - boundary_right_l) <
      lat_buffer) {
    return false;
  }
  return true;
}

int FindBestLaneKeepTrajectory(
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<mapping::ElementId, int>& lane_id_idx_map,
    const absl::flat_hash_map<mapping::ElementId, FeatureCostSum>&
        lane_id_cost_map) {
  int best_lane_keep_idx = -1;
  double best_lane_keep_cost = std::numeric_limits<double>::max();
  for (const auto& [lane_id, cost] : lane_id_cost_map) {
    const int idx = lane_id_idx_map.at(lane_id);
    if (IsPerformLaneChange(
            results[idx].scheduler_output.lane_change_state.stage())) {
      continue;
    }
    if (cost.cost_common < best_lane_keep_cost) {
      best_lane_keep_idx = idx;
      best_lane_keep_cost = cost.cost_common;
    }
  }
  return best_lane_keep_idx;
}

void ClearRouteTtcSettingInHighway(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state) {
  // Clear route ttc config in highway when ego is far away from last ramp;
  int lane_change_lane_idx = -1;
  int lane_keep_lane_idx = -1;
  double min_driving_dis = DBL_MAX;
  for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
    min_driving_dis =
        std::min(min_driving_dis, traj_feature_output.driving_dist);
    if (traj_feature_output.is_perform_lane_change) {
      lane_change_lane_idx = idx;
    } else {
      lane_keep_lane_idx = idx;
    }
  }
  if (lane_change_lane_idx == -1 || lane_keep_lane_idx == -1) {
    return;
  }
  constexpr double kClearRouteTtcConfigDistance = 1600.0;
  if (min_driving_dis > kClearRouteTtcConfigDistance) {
    // Clear route ttc config.
    selector_state->route_ttc_setting.set_route_request_state(
        RouteTtcRequestState::NO_REQUEST);
    selector_state->route_ttc_setting.set_prepare_request_frame(0);
  }
}

bool NeedLaneChangeConfirmationForTrickyScenario(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    int last_selected_idx) {
  if (idx_traj_feature_output_map.size() != 2) {
    return false;
  }
  if (last_selected_idx == -1 ||
      !idx_traj_feature_output_map.contains(last_selected_idx) ||
      idx_traj_feature_output_map.at(last_selected_idx)
          .is_perform_lane_change) {
    // Only need lane change confirmation when lane keep.
    return false;
  }
  int route_target_lane_idx = -1;
  int not_route_target_lane_idx = -1;
  int lane_change_lane_idx = -1;
  int lane_keep_lane_idx = -1;
  bool in_highway = true;
  for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
    in_highway &= traj_feature_output.in_highway;
    if (traj_feature_output.in_route_target_lane) {
      route_target_lane_idx = idx;
    } else {
      not_route_target_lane_idx = idx;
    }
    if (traj_feature_output.is_perform_lane_change) {
      lane_change_lane_idx = idx;
    } else {
      lane_keep_lane_idx = idx;
    }
  }
  if (route_target_lane_idx == -1 || not_route_target_lane_idx == -1 ||
      !in_highway || lane_change_lane_idx == -1 || lane_keep_lane_idx == -1) {
    return false;
  }

  if (idx_traj_feature_output_map.at(lane_change_lane_idx)
          .cross_solid_boundary) {
    // There is solid boundary in lane change lane.
    return false;
  }
  constexpr double kSlowFactorThreshold = 0.1;
  if (idx_traj_feature_output_map.at(not_route_target_lane_idx)
          .reach_need_confirmation_distance &&
      (idx_traj_feature_output_map.at(not_route_target_lane_idx)
           .progress_factor >
       idx_traj_feature_output_map.at(route_target_lane_idx).progress_factor +
           kSlowFactorThreshold)) {
    // There is slow leader in route target lane.
    return true;
  }
  return false;
}

void SendRouteTtcLaneChangeRequest(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state) {
  int route_target_lane_idx = -1;
  int not_route_target_lane_idx = -1;
  int lane_change_lane_idx = -1;
  for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
    if (traj_feature_output.in_route_target_lane) {
      route_target_lane_idx = idx;
    } else {
      not_route_target_lane_idx = idx;
    }
    if (traj_feature_output.is_perform_lane_change) {
      lane_change_lane_idx = idx;
    }
  }
  bool route_lc_left = false;
  const double driving_dist =
      idx_traj_feature_output_map.at(not_route_target_lane_idx).driving_dist;
  if (route_target_lane_idx == lane_change_lane_idx) {
    // Lane change to route target lane.
    route_lc_left =
        idx_traj_feature_output_map.at(lane_change_lane_idx).lane_change_left;
    selector_state->selector_lane_change_request.set_lane_change_type(
        LaneChangeType::DEFAULT_ROUTE_CHANGE);
    selector_state->selector_lane_change_request.set_lc_left(route_lc_left);
    selector_state->route_ttc_setting.set_request_config(
        RouteTtcConfig::CONSERVATIVE);
  } else {
    route_lc_left =
        !idx_traj_feature_output_map.at(lane_change_lane_idx).lane_change_left;
    selector_state->selector_lane_change_request.set_lane_change_type(
        LaneChangeType::OVERTAKE_CHANGE);
    selector_state->selector_lane_change_request.set_lc_left(!route_lc_left);
    selector_state->route_ttc_setting.set_request_config(
        RouteTtcConfig::RADICAL);
  }

  // Send route ttc lane change request.
  QLOG(INFO) << "Send route ttc lane change request, driving dist: "
             << driving_dist << ", route lc left: " << route_lc_left
             << " request lc left: "
             << selector_state->selector_lane_change_request.lc_left();
  QRunEvent::RouteTtcLaneChangeRequestProto route_ttc_lane_change_request;
  route_ttc_lane_change_request.set_route_lc_left(route_lc_left);
  route_ttc_lane_change_request.set_request_lc_left(
      selector_state->selector_lane_change_request.lc_left());
  route_ttc_lane_change_request.set_driving_dist(driving_dist);
  QRUNEVENT_WITH_PROTO_NOTICE(
      QRunEvent::KEY_QEVENT_ROUTE_TTC_LANE_CHANGE_REQUEST,
      route_ttc_lane_change_request);
}

std::optional<bool> IsGoingToForceRouteChangeLeft(
    int final_selected_idx, const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map) {
  int lane_keep_lane_idx = -1;
  bool lane_change_left = false;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok() &&
        est_status[idx].status_code() !=
            PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      continue;
    }
    if (IsPerformLaneChange(
            results[idx].scheduler_output.lane_change_state.stage())) {
      lane_change_left =
          results[idx].scheduler_output.lane_change_state.lc_left();
    } else {
      lane_keep_lane_idx = idx;
    }
  }

  if (lane_keep_lane_idx != final_selected_idx) {
    // already in lane change stage.
    return std::nullopt;
  }
  if (!idx_traj_feature_output_map.at(lane_keep_lane_idx)
           .has_obvious_route_cost) {
    // not in obvious route cost.
    return std::nullopt;
  }
  return lane_change_left;
}

bool IsLaneChangeForObstacleFail(
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map) {
  int lane_keep_lane_idx = -1;
  int lane_change_lane_idx = -1;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok() &&
        est_status[idx].status_code() !=
            PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      continue;
    }
    if (IsPerformLaneChange(
            results[idx].scheduler_output.lane_change_state.stage())) {
      lane_change_lane_idx = idx;
    } else {
      lane_keep_lane_idx = idx;
    }
  }
  if (lane_keep_lane_idx == -1 || lane_change_lane_idx == -1) {
    return false;
  }

  if (!idx_traj_feature_output_map.contains(lane_keep_lane_idx) ||
      !idx_traj_feature_output_map.at(lane_keep_lane_idx)
           .lane_change_for_stationary_obj) {
    return false;
  }

  return est_status[lane_change_lane_idx].status_code() ==
         PlannerStatusProto::LC_SAFETY_CHECK_FAILED;
}

bool IsAllTrajectoryBlocked(const absl::flat_hash_map<int, TrajFeatureOutput>&
                                idx_traj_feature_output_map) {
  for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
    if (!traj_feature_output.is_blocked_by_stalled_obj) {
      return false;
    }
  }
  return true;
}

int PrepareLaneChangeBranch(const PlannerSemanticMapManager& psmm,
                            const std::vector<EstPlannerOutput>& results,
                            const SelectorState& selector_state,
                            int left_branch_idx, int right_branch_idx) {
  if (!selector_state.best_target_lane_state.has_successive_count()) {
    return -1;
  }
  const auto& prev_target_lane_state = selector_state.best_target_lane_state;
  if (IsSameTargetLane(prev_target_lane_state,
                       GenerateTargetLaneState(
                           psmm, results[left_branch_idx].scheduler_output))) {
    return left_branch_idx;
  }
  if (IsSameTargetLane(prev_target_lane_state,
                       GenerateTargetLaneState(
                           psmm, results[right_branch_idx].scheduler_output))) {
    return right_branch_idx;
  }
  return -1;
}

void ClearPrefilterState(SelectorState* selector_state) {
  selector_state->prefilter_state.Clear();
  selector_state->prefilter_history_infos.clear();
}

double ComputeBranchCost(const SelectorCommonFeature& common_feature,
                         const PreFilterStats& prefilter_stats,
                         const std::vector<EstPlannerOutput>& results,
                         int branch_idx, bool is_left_side,
                         PrefilterCostDebugProto* cost_debug,
                         SelectorState* selector_state) {
  constexpr double kDistanceEpsilon = 1.0;
  constexpr double kRouteWeight = 20.0;
  constexpr double kCrossSolidBoundaryWeight = 100.0;
  constexpr double kEncourageLeftWeight = 5.0;
  constexpr double kBlockOrDangerWeight = 7;
  constexpr double kSwitchSideWeight = 1.0;
  constexpr double kEncourageRouteChangeDist = 1000.0;
  constexpr double kEncourageRouteChangeDistInHighway = 2500.0;

  const auto& scheduler_output = results[branch_idx].scheduler_output;
  const auto& lane_feature_info = FindOrDieNoPrint(
      common_feature.lane_feature_infos, scheduler_output.Hash());
  const auto start_lane_id =
      scheduler_output.drive_passage.lane_path().front().lane_id();
  const bool in_highway = common_feature.in_high_way;
  double branch_cost = 0.0;

  // Compute route cost.
  const bool is_max_lc_num =
      lane_feature_info.lc_num_to_targets == prefilter_stats.max_lc_num;
  const bool is_min_driving_dist =
      lane_feature_info.driving_dist <
      prefilter_stats.min_driving_dist + kDistanceEpsilon;
  const double encourage_route_change_dist =
      in_highway ? kEncourageRouteChangeDistInHighway
                 : kEncourageRouteChangeDist;
  const double route_cost =
      btof(is_max_lc_num && is_min_driving_dist &&
           lane_feature_info.driving_dist < encourage_route_change_dist) *
      kRouteWeight;

  // Compute cross solid boundary cost.
  const double cross_solid_boundary_cost =
      btof(scheduler_output.is_solid_lane_change) * kCrossSolidBoundaryWeight;

  // Compute left encourage cost.
  const double encourage_left_cost = btof(!is_left_side) * kEncourageLeftWeight;

  // Compute lane change safety cost and lc blocked cost.
  const double block_rate = is_left_side ? prefilter_stats.left_blocked_rate
                                         : prefilter_stats.right_blocked_rate;
  const double blocked_or_danger_cost = block_rate * kBlockOrDangerWeight;
  // Compute switch side cost.
  double switch_side_cost = 0.0;
  if (selector_state->prefilter_state.has_choose_left_branch()) {
    switch_side_cost =
        btof(selector_state->prefilter_state.choose_left_branch() !=
             is_left_side) *
        kSwitchSideWeight;
  }

  // Compute branch cost.
  branch_cost = route_cost + cross_solid_boundary_cost + encourage_left_cost +
                blocked_or_danger_cost + switch_side_cost;
  cost_debug->set_lc_left(is_left_side);
  cost_debug->set_start_lane_id(start_lane_id.value());
  cost_debug->set_sum_cost(branch_cost);
  cost_debug->add_extra_info(absl::StrFormat("Route cost: %.2f", route_cost));
  cost_debug->add_extra_info(absl::StrFormat("Cross solid boundary cost: %.2f",
                                             cross_solid_boundary_cost));
  cost_debug->add_extra_info(
      absl::StrFormat("Encourage left cost: %.2f", encourage_left_cost));
  cost_debug->add_extra_info(
      absl::StrFormat("Block or danger cost: %.2f", blocked_or_danger_cost));
  cost_debug->add_extra_info(
      absl::StrFormat("Switch side cost: %.2f", switch_side_cost));
  cost_debug->add_extra_info(absl::StrFormat(
      "Lc num: %d, min driving dist: %.2f", lane_feature_info.lc_num_to_targets,
      lane_feature_info.driving_dist));
  cost_debug->add_extra_info(
      absl::StrFormat("Block or danger rate: %.2f", block_rate));

  return branch_cost;
}

void UpdateBranchPrefilterHistoryInfo(
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const SelectorCommonFeature& common_feature, int branch_idx,
    bool* is_blocked_by_stalled_obj, bool* safty_check_failed) {
  if (branch_idx >= est_status.size() || branch_idx < 0) {
    return;
  }
  *safty_check_failed = est_status.at(branch_idx).status_code() ==
                        PlannerStatusProto::LC_SAFETY_CHECK_FAILED;
  const auto& scheduler_output = results[branch_idx].scheduler_output;
  const auto& lane_feature_info = FindOrDieNoPrint(
      common_feature.lane_feature_infos, scheduler_output.Hash());
  *is_blocked_by_stalled_obj = false;
  for (const auto& block_id : lane_feature_info.block_obj_ids) {
    if (stalled_object_ids.contains(block_id)) {
      *is_blocked_by_stalled_obj = true;
      break;
    }
  }
}

int PreFilterEstResultsForSide(
    absl::Time plan_time, const PlannerSemanticMapManager& psmm,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const SelectorCommonFeature& common_feature, int left_branch_idx,
    int right_branch_idx, SelectorDebugProto* selector_debug,
    SelectorState* selector_state, std::string* prefilter_reason) {
  // Choose left or right side for selector.

  // Update prefilter history info.
  PrefilterHistoryInfo prefilter_history_info;
  UpdateBranchPrefilterHistoryInfo(
      stalled_object_ids, results, est_status, common_feature, left_branch_idx,
      &prefilter_history_info.left_is_blocked_by_stalled,
      &prefilter_history_info.left_safty_check_failed);
  UpdateBranchPrefilterHistoryInfo(
      stalled_object_ids, results, est_status, common_feature, right_branch_idx,
      &prefilter_history_info.right_is_blocked_by_stalled,
      &prefilter_history_info.right_safty_check_failed);

  constexpr double kHistoryBufferTime = 20.0;      // s.
  constexpr double kMinConsiderBufferTime = 16.0;  // s.
  selector_state->prefilter_history_infos.PushBackAndClearStale(
      plan_time, prefilter_history_info, absl::Seconds(kHistoryBufferTime));

  // Compute prefilter stats.
  PreFilterStats prefilter_stats;
  const std::vector<int> branch_idxs = {left_branch_idx, right_branch_idx};
  for (int branch_idx : branch_idxs) {
    const auto& scheduler_output = results[branch_idx].scheduler_output;
    const auto& lane_feature_info = FindOrDieNoPrint(
        common_feature.lane_feature_infos, scheduler_output.Hash());
    prefilter_stats.max_lc_num = std::max(prefilter_stats.max_lc_num,
                                          lane_feature_info.lc_num_to_targets);
    prefilter_stats.min_driving_dist = std::min(
        prefilter_stats.min_driving_dist, lane_feature_info.driving_dist);
  }
  if (selector_state->prefilter_history_infos.duration() >
      absl::Seconds(kMinConsiderBufferTime)) {
    // Compute blocked rate.
    for (const auto& [_, prefilter_info] :
         selector_state->prefilter_history_infos) {
      prefilter_stats.left_blocked_rate +=
          btof(prefilter_info.left_is_blocked_by_stalled ||
               prefilter_info.left_safty_check_failed);
      prefilter_stats.right_blocked_rate +=
          btof(prefilter_info.right_is_blocked_by_stalled ||
               prefilter_info.right_safty_check_failed);
    }
    prefilter_stats.left_blocked_rate /=
        selector_state->prefilter_history_infos.size();
    prefilter_stats.right_blocked_rate /=
        selector_state->prefilter_history_infos.size();
  }

  // Compute branch cost.
  std::vector<double> branch_costs(branch_idxs.size(), 0.0);
  for (int i = 0; i < branch_idxs.size(); ++i) {
    auto* cost_debug =
        selector_debug->mutable_prefilter_debug()->add_prefilter_costs();
    branch_costs[i] = ComputeBranchCost(
        common_feature, prefilter_stats, results, branch_idxs[i],
        branch_idxs[i] == left_branch_idx, cost_debug, selector_state);
  }
  // Choose the same branch as last time.
  int last_prepare_lane_change_branch_idx = PrepareLaneChangeBranch(
      psmm, results, *selector_state, left_branch_idx, right_branch_idx);
  if (last_prepare_lane_change_branch_idx != -1) {
    *prefilter_reason = "Choose the same best branch as last frame.";
    return last_prepare_lane_change_branch_idx == left_branch_idx
               ? right_branch_idx
               : left_branch_idx;
  }
  // Choose the branch with lower cost.
  *prefilter_reason = "Choose the branch with lower cost.";
  return branch_costs[0] < branch_costs[1] ? right_branch_idx : left_branch_idx;
}
}  // namespace

LaneChangeGeneralType ConvertLaneChangeTypeToGeneralType(
    LaneChangeType lane_change_type) {
  switch (lane_change_type) {
    case LaneChangeType::NO_CHANGE:
      return LaneChangeGeneralType::LCGT_NO_CHANGE;
    case LaneChangeType::OVERTAKE_CHANGE:
    case LaneChangeType::STALLED_VEHICLE_CHANGE:
      return LaneChangeGeneralType::LCGT_OVERTAKE_CHANGE;
    case LaneChangeType::ROAD_SPEED_LIMIT_CHANGE:
    case LaneChangeType::MAINROAD_EXIT_CHANGE:
    case LaneChangeType::ENTER_MAINROAD_CHANGE:
    case LaneChangeType::DEFAULT_ROUTE_CHANGE:
      return LaneChangeGeneralType::LCGT_ROUTE_CHANGE;
    case LaneChangeType::DEFAULT_CHANGE:
    case LaneChangeType::OBSTACLE_CHANGE:
    case LaneChangeType::CONSTRUCTION_ZONE_CHANGE:
      return LaneChangeGeneralType::LCGT_DEFAULT_CHANGE;
    case LaneChangeType::PADDLE_CHANGE:
      return LaneChangeGeneralType::LCGT_PADDLE_CHANGE;
  }
}

bool IsSameTargetLane(const TargetLaneStateProto& prev,
                      const TargetLaneStateProto& curr) {
  // TODO(chengyang): use center point instead of lane id.
  if (prev.lane_ids_size() == 0 || curr.lane_ids_size() == 0) return false;
  auto start_it = std::find(prev.lane_ids().begin(), prev.lane_ids().end(),
                            curr.lane_ids().at(0));
  if (start_it == prev.lane_ids().end()) return false;
  for (int i = start_it - prev.lane_ids().begin(), j = 0;
       i < prev.lane_ids_size() && j < curr.lane_ids_size(); ++i, ++j) {
    if (prev.lane_ids().at(i) != curr.lane_ids().at(j)) {
      return false;
    }
  }
  return prev.is_borrow() == curr.is_borrow();
}

void UpdateSelectorStateBeforeSelection(absl::Time plan_time,
                                        SelectorState* selector_state) {
  if (!selector_state->best_target_lane_state.has_successive_count()) {
    selector_state->activate_selector_time = plan_time;
  }
}

void UpdateSelectorStateAfterSelection(
    absl::Time plan_time, const SelectorFlags& selector_flags,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<PlannerStatus>& est_status,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    int final_selected_idx, int best_traj_idx, bool is_paddle_lane_change,
    SelectorState* selector_state) {
  if (final_selected_idx < 0) return;
  selector_state->turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  if (selector_state->pre_turn_signal != TurnSignal::TURN_SIGNAL_NONE) {
    selector_state->lane_change_type = AnalyzeLaneChangeType(
        idx_traj_feature_output_map, is_paddle_lane_change, best_traj_idx,
        selector_state->lane_change_type);
    selector_state->turn_signal = selector_state->pre_turn_signal;
  } else {
    selector_state->lane_change_type = AnalyzeLaneChangeType(
        idx_traj_feature_output_map, is_paddle_lane_change, final_selected_idx,
        selector_state->lane_change_type);
  }
  selector_state->lane_change_general_type =
      ConvertLaneChangeTypeToGeneralType(selector_state->lane_change_type);

  const auto& final_scheduler = results[final_selected_idx].scheduler_output;
  if (IsPerformLaneChange(final_scheduler.lane_change_state.stage())) {
    // Store last lane change time and direction.
    selector_state->last_lc_info.set_lc_left(
        final_scheduler.lane_change_state.lc_left());
    qcraft::ToProto(plan_time,
                    selector_state->last_lc_info.mutable_lane_change_time());
    selector_state->last_lc_info.set_lane_change_type(
        selector_state->lane_change_type);
    selector_state->turn_signal = final_scheduler.lane_change_state.lc_left()
                                      ? TURN_SIGNAL_LEFT
                                      : TURN_SIGNAL_RIGHT;
    // Store lane change start time.
    if (!selector_state->start_lane_change_time.has_value()) {
      selector_state->start_lane_change_time = plan_time;
      if (idx_traj_feature_output_map.at(final_selected_idx).in_highway) {
        QEVENT_EVERY_N_SECONDS("chengyang", "highway_auto_lane_change",
                               /*seconds=*/5.0, [&](QEvent*) {});
      } else {
        QEVENT_EVERY_N_SECONDS("chengyang", "urban_auto_lane_change",
                               /*seconds=*/5.0, [&](QEvent*) {});
      }
    }

    // Store lane change start give up time.
    const double time_after_lc_start = absl::ToDoubleSeconds(
        plan_time - *selector_state->start_lane_change_time);
    if (time_after_lc_start >
        selector_flags.planner_max_allow_lc_time_before_give_up) {
      selector_state->give_up_lane_change_time = plan_time;
    }
  } else {
    selector_state->start_lane_change_time.reset();
  }

  // Update red light stop time.
  bool has_red_light_stop_s = true;
  for (int idx = 0; idx < est_status.size(); ++idx) {
    if (!est_status[idx].ok()) continue;
    has_red_light_stop_s =
        has_red_light_stop_s && results[idx].redlight_lane_id.has_value();
  }
  if (has_red_light_stop_s) {
    selector_state->last_redlight_stop_time = plan_time;
  }
}

void UpdateSelectorOutput(const std::vector<EstPlannerOutput>& results,
                          const std::vector<PlannerStatus>& est_status,
                          const SelectorState& selector_state,
                          const absl::flat_hash_map<int, TrajFeatureOutput>&
                              idx_traj_feature_output_map,
                          bool in_high_way, int final_selected_idx,
                          int last_selected_idx,
                          SelectorOutput* selector_output) {
  selector_output->is_going_force_route_change_left =
      IsGoingToForceRouteChangeLeft(final_selected_idx, results, est_status,
                                    idx_traj_feature_output_map);
  selector_output->lane_change_for_obstacle_fail = IsLaneChangeForObstacleFail(
      results, est_status, idx_traj_feature_output_map);
  selector_output->selected_idx = final_selected_idx;
  selector_output->last_selected_idx = last_selected_idx;
  selector_output->all_trajectories_blocked =
      IsAllTrajectoryBlocked(idx_traj_feature_output_map);
  selector_output->turn_signal = selector_state.turn_signal;
  selector_output->in_high_way = in_high_way;
  selector_output->merge_point =
      idx_traj_feature_output_map.at(final_selected_idx).merge_point;
}

void UpdateTrajectoryCostForEachLane(
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const absl::flat_hash_map<int, FeatureCostSum>& all_trajectory_cost,
    absl::flat_hash_map<mapping::ElementId, FeatureCostSum>* lane_id_cost_map,
    absl::flat_hash_map<mapping::ElementId, int>* lane_id_idx_map,
    absl::flat_hash_map<int, int>* idx_selector_debug_map) {
  int valid_traj_count = 0;
  for (int idx = 0; idx < results.size(); ++idx) {
    if (!est_status[idx].ok()) continue;
    valid_traj_count++;
    const auto start_id = results[idx]
                              .scheduler_output.drive_passage.lane_path()
                              .front()
                              .lane_id();
    if (!lane_id_cost_map->contains(start_id) ||
        all_trajectory_cost.at(idx) < lane_id_cost_map->at(start_id)) {
      // Keep only one best trajectory for each start lane.
      (*lane_id_cost_map)[start_id] = all_trajectory_cost.at(idx);
      (*lane_id_idx_map)[start_id] = idx;
    }
    (*idx_selector_debug_map)[idx] = valid_traj_count - 1;
  }
}

void UpdateRouteTtcSettingInHighway(
    std::optional<bool> alc_confirmation, int last_selected_idx,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state) {
  if (selector_flags.planner_is_l4_mode) {
    // Only valid in noa mode.
    return;
  }
  if (!selector_flags.planner_enable_lc_request_in_tricky_scenario) {
    // Open after hmi adapt new lane change request.
    return;
  }

  ClearRouteTtcSettingInHighway(idx_traj_feature_output_map, selector_state);
  const bool already_send_request =
      selector_state->selector_lane_change_request.lane_change_type() !=
      LaneChangeType::NO_CHANGE;
  const bool is_in_tricky_scenario =
      NeedLaneChangeConfirmationForTrickyScenario(idx_traj_feature_output_map,
                                                  last_selected_idx);
  switch (selector_state->route_ttc_setting.route_request_state()) {
    case RouteTtcRequestState::NO_REQUEST:
      if (is_in_tricky_scenario) {
        selector_state->route_ttc_setting.set_prepare_request_frame(
            selector_state->route_ttc_setting.prepare_request_frame() + 1);
      } else {
        selector_state->route_ttc_setting.set_prepare_request_frame(0);
      }
      if (!already_send_request &&
          selector_state->route_ttc_setting.prepare_request_frame() >=
              selector_flags
                  .planner_lc_begin_request_frame_in_tricky_scenario) {
        selector_state->route_ttc_setting.set_route_request_state(
            RouteTtcRequestState::WAITING_RESPONSE);
        SendRouteTtcLaneChangeRequest(idx_traj_feature_output_map,
                                      selector_state);
      }
      break;
    case RouteTtcRequestState::WAITING_RESPONSE:
      if (alc_confirmation.has_value()) {
        selector_state->route_ttc_setting.set_route_request_state(
            RouteTtcRequestState::RECEIVED_RESPONSE);
        if (*alc_confirmation) {
          selector_state->route_ttc_setting.set_response_config(
              selector_state->route_ttc_setting.request_config());
        } else {
          switch (selector_state->route_ttc_setting.request_config()) {
            case RouteTtcConfig::CONSERVATIVE:
              selector_state->route_ttc_setting.set_response_config(
                  RouteTtcConfig::RADICAL);
              break;
            case RouteTtcConfig::RADICAL:
              selector_state->route_ttc_setting.set_response_config(
                  RouteTtcConfig::CONSERVATIVE);
              break;
          }
        }
      }
      break;
    case RouteTtcRequestState::RECEIVED_RESPONSE:
      break;
  }
}

void HandleAlcConfirmation(absl::Time plan_time,
                           std::optional<bool> alc_confirmation,
                           SelectorState* selector_state) {
  if (alc_confirmation.has_value()) {
    if (!*alc_confirmation) {
      selector_state->last_user_reject_alc_type =
          selector_state->selector_lane_change_request.lane_change_type();
      selector_state->last_user_reject_alc_time = plan_time;
    }
    selector_state->selector_lane_change_request.set_lane_change_type(
        LaneChangeType::NO_CHANGE);
  }
}

int DecideBeginLaneChangeFrame(
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map) {
  int lane_keep_lane_idx = -1;
  int default_begin_lane_change_frame =
      selector_flags.planner_begin_lane_change_frame;
  int fast_begin_lane_change_frame =
      selector_flags.planner_begin_radical_lane_change_frame;
  for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
    if (!traj_feature_output.is_perform_lane_change) {
      lane_keep_lane_idx = idx;
    }
  }
  if (lane_keep_lane_idx != -1 &&
      idx_traj_feature_output_map.at(lane_keep_lane_idx)
          .has_obvious_route_cost) {
    return fast_begin_lane_change_frame;
  }
  return default_begin_lane_change_frame;
}

std::vector<PlannerStatus> PreFilterEstResults(
    absl::Time plan_time, const PlannerSemanticMapManager& psmm,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const SelectorCommonFeature& common_feature, int last_selected_idx,
    SelectorDebugProto* selector_debug, SelectorState* selector_state) {
  std::vector<PlannerStatus> filtered_est_status = est_status;
  if (!selector_flags.planner_enable_prefilter_for_selector) {
    return filtered_est_status;
  }
  // We can change branch idx to branch set and add more filter when there are
  // more than 3 branches.
  int left_branch_idx = -1;
  int right_branch_idx = -1;
  for (int i = 0; i < results.size(); ++i) {
    if (results[i].scheduler_output.drive_passage.lane_path().IsEmpty()) {
      continue;
    }
    const auto& target_lane_change_state =
        results[i].scheduler_output.lane_change_state;
    if (IsPerformLaneChange(target_lane_change_state.stage())) {
      if (target_lane_change_state.lc_left()) {
        left_branch_idx = i;
      } else {
        right_branch_idx = i;
      }
    }
  }

  if (left_branch_idx == -1 || right_branch_idx == -1) {
    ClearPrefilterState(selector_state);
    return filtered_est_status;
  }
  std::string prefilter_reason;
  // Get last frame lane change direction.
  std::optional<bool> last_lc_left = std::nullopt;
  if (last_selected_idx != -1 &&
      IsPerformLaneChange(results[last_selected_idx]
                              .scheduler_output.lane_change_state.stage())) {
    last_lc_left =
        results[last_selected_idx].scheduler_output.lane_change_state.lc_left();
  }
  if (!last_lc_left.has_value() &&
      selector_state->pre_turn_signal != TurnSignal::TURN_SIGNAL_NONE) {
    last_lc_left =
        selector_state->pre_turn_signal == TurnSignal::TURN_SIGNAL_LEFT;
  }

  int drop_branch_idx = -1;
  if (last_lc_left.has_value()) {
    drop_branch_idx = last_lc_left.value() ? right_branch_idx : left_branch_idx;
    prefilter_reason = "Choose the same lane change branch as last frame.";
    ClearPrefilterState(selector_state);
  } else {
    // Choose one branch from left and right branch.
    drop_branch_idx = PreFilterEstResultsForSide(
        plan_time, psmm, stalled_object_ids, results, filtered_est_status,
        common_feature, left_branch_idx, right_branch_idx, selector_debug,
        selector_state, &prefilter_reason);
    selector_state->prefilter_state.set_choose_left_branch(drop_branch_idx !=
                                                           left_branch_idx);
  }
  selector_debug->mutable_prefilter_debug()->set_prefilter_reason(
      prefilter_reason);
  if (drop_branch_idx != -1) {
    filtered_est_status[drop_branch_idx] = PlannerStatus(
        PlannerStatusProto::BRANCH_RESULT_IGNORED, prefilter_reason);
    selector_debug->mutable_prefilter_debug()->set_choose_left(
        drop_branch_idx != left_branch_idx);
  }

  return filtered_est_status;
}

double LinearInterpolate(double x0, double x1, double t0, double t1, double t) {
  if (std::fabs(t1 - t0) < 1e-6) {
    return x0;
  }
  return x0 + (x1 - x0) * (t - t0) / (t1 - t0);
}

bool IsInTlControlledIntersection(const PlannerSemanticMapManager& psmm,
                                  const DrivePassage& drive_passage, double s) {
  const auto& station = drive_passage.FindNearestStationAtS(s);
  if (!station.is_in_intersection()) return false;

  const auto lane_pt = station.GetLanePoint();
  SMM_ASSIGN_LANE_OR_RETURN(lane_info, psmm, lane_pt.lane_id(), false);
  for (const auto& [id, frac] : lane_info.Intersections()) {
    const auto* intersection_ptr = psmm.FindIntersectionByIdOrNull(id);
    if (intersection_ptr != nullptr &&
        intersection_ptr->proto->traffic_light_controlled() &&
        frac[0] <= lane_pt.fraction() && lane_pt.fraction() <= frac[1]) {
      return true;
    }
  }
  return false;
}

void AddBoundariesToIntervals(const DrivePassage& drive_passage,
                              std::vector<IndexedStationBoundary> boundaries,
                              std::vector<BoundaryInterval>* intervals) {
  if (boundaries.size() < 2) return;

  BoundaryInterval interval;
  interval.type = boundaries.front().second.type;
  interval.points.reserve(boundaries.size());
  for (const auto& boundary : boundaries) {
    interval.points.emplace_back(drive_passage.station(boundary.first)
                                     .lat_point(boundary.second.lat_offset));
  }
  intervals->emplace_back(std::move(interval));
}

std::vector<double> MultiplyVector(const std::vector<double>& vec1,
                                   const std::vector<double>& vec2) {
  QCHECK_EQ(vec1.size(), vec2.size());
  std::vector<double> result;
  result.reserve(vec1.size());
  for (int i = 0; i < vec1.size(); ++i) {
    result.push_back(vec1[i] * vec2[i]);
  }
  return result;
}

std::vector<BoundaryInterval> FindSolidBoundaryIntervals(
    const DrivePassage& drive_passage, const FrenetCoordinate& first_point_sl,
    double cutoff_s) {
  std::vector<BoundaryInterval> intervals;
  std::vector<std::vector<IndexedStationBoundary>> active_intervals;
  for (const auto index : drive_passage.stations().index_range()) {
    const auto& station = drive_passage.station(index);
    if (station.accumulated_s() < -kRouteStationUnitStep) continue;
    if (station.accumulated_s() > cutoff_s) break;

    std::vector<StationBoundary> new_boundaries;
    for (const auto& boundary : station.boundaries()) {
      if (!boundary.IsSolid(first_point_sl.l) ||
          boundary.type == StationBoundaryType::VIRTUAL_CURB) {
        // Only consider real boundaries here, not the virtual curbs.
        continue;
      }

      double match_dist = kContinuousBoundaryMaxLatOffset;
      int match_idx = -1;
      for (int j = 0; j < active_intervals.size(); ++j) {
        const auto& interval_back = active_intervals[j].back();
        if (interval_back.first.value() + 1 == index.value() &&
            interval_back.second.type == boundary.type) {
          const double lat_dist =
              std::abs(interval_back.second.lat_offset - boundary.lat_offset);
          if (lat_dist < match_dist) {
            match_dist = lat_dist;
            match_idx = j;
          }
        }
      }
      if (match_idx == -1) {
        new_boundaries.push_back(boundary);
      } else {
        active_intervals[match_idx].emplace_back(index, boundary);
      }
    }
    for (auto it = active_intervals.begin(); it != active_intervals.end();) {
      if (it->back().first != index) {
        AddBoundariesToIntervals(drive_passage, std::move(*it), &intervals);
        it = active_intervals.erase(it);
      } else {
        ++it;
      }
    }
    for (auto& new_boundary : new_boundaries) {
      active_intervals.emplace_back().emplace_back(index, new_boundary);
    }
  }
  for (auto& interval : active_intervals) {
    AddBoundariesToIntervals(drive_passage, std::move(interval), &intervals);
  }

  return intervals;
}

double CalculateCrossingBoundary(
    const DrivePassage& drive_passage,
    const std::vector<BoundaryInterval>& solid_boundaries,
    const std::vector<Box2d>& ego_boxes, LaneChangeStage stage,
    const absl::flat_hash_set<StationBoundaryType>& type_set,
    const absl::StatusOr<double>& start_l_or,
    const absl::StatusOr<double>& end_l_or, const double ego_half_width) {
  double crossing_factor = 0;

  for (const auto& boundary : solid_boundaries) {
    if (!type_set.contains(boundary.type)) continue;

    const auto& boundary_pts = boundary.points;
    for (const auto& ego_box : ego_boxes) {
      bool has_overlap = false;
      absl::StatusOr<double> cross_l_or;
      for (int j = 1; j < boundary_pts.size(); ++j) {
        const Segment2d boundary_seg(boundary_pts[j - 1], boundary_pts[j]);
        if (ego_box.HasOverlap(boundary_seg)) {
          cross_l_or = drive_passage.QueryFrenetLatOffsetAt(boundary_pts.at(j));
          has_overlap = true;
          break;
        }
      }
      if (has_overlap) {
        double factor = kDefaultCrossBoundaryFactor;
        if (cross_l_or.ok() && start_l_or.ok() && end_l_or.ok()) {
          const double cross_l = cross_l_or.value();
          const double start_l = start_l_or.value();
          const double end_l = end_l_or.value();
          if (stage == LaneChangeStage::LCS_PAUSE) {
            if (std::fabs(end_l - cross_l) < ego_half_width) {
              // Occupy solid line.
              factor = kOccupiedCrossBoundaryFactor;
            }
          } else {
            if ((start_l - cross_l) * (end_l - cross_l) < 0.0) {
              // Crossed the solid line.
              factor = kFullCrossBoundaryFactor;
            }
          }
        }
        crossing_factor = std::max(factor, crossing_factor);
        break;
      }
    }
  }
  return crossing_factor;
}

absl::flat_hash_set<std::string> FindFrontNonBlockObjectIds(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const VehicleGeometryParamsProto& vehicle_geom,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::flat_hash_set<std::string>& block_obj_ids) {
  absl::flat_hash_set<std::string> front_non_block_obj_ids;
  const auto& passage = planner_output.scheduler_output.drive_passage;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geom);
  ASSIGN_OR_RETURN(const auto ego_frenet_box, passage.QueryFrenetBoxAt(ego_box),
                   front_non_block_obj_ids);
  absl::flat_hash_set<std::string> has_checked_set;
  absl::flat_hash_set<std::string> speed_consider_set;
  constexpr double kMaxConsiderFrontDist = 80.0;  // m.
  constexpr double kMinConsiderFrontDist = 40.0;  // m.
  constexpr double kConsiderFrontTime = 3.0;      // s.
  constexpr double kCheckFrontLonBuffer = 1.0;    // m.
  constexpr double kObsLatInLaneBuffer = 0.5;     // m.
  const double consider_front_dist = std::clamp(
      ego_frenet_box.s_max + kConsiderFrontTime * plan_start_point.v(),
      kMinConsiderFrontDist, kMaxConsiderFrontDist);
  std::optional<bool> lc_left =
      IsPerformLaneChange(
          planner_output.scheduler_output.lane_change_state.stage())
          ? std::optional<bool>(
                planner_output.scheduler_output.lane_change_state.lc_left())
          : std::nullopt;

  for (const auto& traj : planner_output.considered_st_objects) {
    speed_consider_set.emplace(traj.st_traj().object_id());
  }

  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Remove duplicated object id
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());
    // Remove block object id.
    if (block_obj_ids.contains(traj.object_id())) continue;
    // Remove obstacle considered by speed.
    if (speed_consider_set.contains(traj.object_id())) continue;
    // Remove object id behind ego.
    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (aabbox.s_max < ego_frenet_box.s_min - kCheckFrontLonBuffer) {
      continue;
    }
    if (aabbox.s_min > consider_front_dist) {
      continue;
    }
    if (!IsBranchBlockedByObs(passage, ego_frenet_box, aabbox,
                              kObsLatInLaneBuffer, traj.is_stationary(),
                              lc_left)) {
      continue;
    }
    front_non_block_obj_ids.emplace(traj.object_id());
  }
  return front_non_block_obj_ids;
}

absl::flat_hash_set<std::string> FindBlockObjectIds(
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const EstPlannerOutput& planner_output,
    const VehicleGeometryParamsProto& vehicle_geom) {
  absl::flat_hash_set<std::string> block_obj_ids;
  constexpr double kEgoPassingLatBuffer = 0.4;  // m.
  constexpr double kObsLatInLaneBuffer = 0.4;   // m.
  const auto allow_width = vehicle_geom.width() + kEgoPassingLatBuffer;
  const auto& passage = planner_output.scheduler_output.drive_passage;
  double speed_max_consider_distance = 0.0;

  if (!planner_output.traj_points.empty()) {
    const auto last_point_sl_or = passage.QueryFrenetCoordinateAt(
        Vec2dFromApolloTrajectoryPointProto(planner_output.traj_points.back()));
    if (last_point_sl_or.ok()) {
      speed_max_consider_distance =
          std::max(last_point_sl_or->s, speed_max_consider_distance);
    }
  }

  absl::flat_hash_map<std::string_view, const PartialSpacetimeObjectTrajectory*>
      speed_consider_objects_map;
  for (const auto& traj : planner_output.considered_st_objects) {
    speed_consider_objects_map[traj.st_traj().object_id()] = &traj;
    const auto init_decision_type = traj.GetDecisionTypeAtTime(0.0);
    if (init_decision_type.has_value()) {
      ASSIGN_OR_CONTINUE(const auto aabbox, passage.QueryFrenetBoxAtContour(
                                                traj.st_traj().contour()));
      speed_max_consider_distance =
          std::max(speed_max_consider_distance, aabbox.s_min);
    }
  }

  absl::flat_hash_set<std::string> has_checked_set;
  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Remove duplicated object id
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());

    // Ignore objects just passing through the current lane path.
    const auto& obj_pose = *traj.states().front().traj_point;
    const auto* st_object_ptr =
        FindOrNull(speed_consider_objects_map, traj.object_id());
    ASSIGN_OR_CONTINUE(const auto dp_tan,
                       passage.QueryTangentAt(obj_pose.pos()));
    if (!traj.is_stationary() &&
        std::abs(NormalizeAngle(dp_tan.FastAngle() - obj_pose.theta())) >
            M_PI_4) {
      continue;
    }
    // Ignore objects not in front.
    ASSIGN_OR_CONTINUE(const auto aabbox,
                       passage.QueryFrenetBoxAtContour(traj.contour()));
    if (aabbox.s_min < 0.0) continue;

    // Ignore stationary object ignored by speed
    if (traj.is_stationary() && aabbox.s_min < speed_max_consider_distance &&
        st_object_ptr == nullptr) {
      continue;
    }
    double boundary_right_l, boundary_left_l;
    if (planner_output.scheduler_output.borrow_lane) {
      // Use target boundary for lane borrow
      std::tie(boundary_right_l, boundary_left_l) =
          planner_output.scheduler_output.sl_boundary.QueryTargetBoundaryL(
              aabbox.center_s());
    } else {
      const auto [right_boundary, left_boundary] =
          passage.QueryEnclosingLaneBoundariesAtS(aabbox.center_s());
      boundary_right_l =
          right_boundary.has_value()
              ? std::max(right_boundary->lat_offset, -kMaxHalfLaneWidth)
              : -kMaxHalfLaneWidth;
      boundary_left_l =
          left_boundary.has_value()
              ? std::min(left_boundary->lat_offset, kMaxHalfLaneWidth)
              : kMaxHalfLaneWidth;
    }
    const double l_max =
        std::clamp(aabbox.l_max, boundary_right_l, boundary_left_l);
    const double l_min =
        std::clamp(aabbox.l_min, boundary_right_l, boundary_left_l);

    if (std::min(boundary_left_l - l_min, l_max - boundary_right_l) <
        kObsLatInLaneBuffer) {
      continue;
    }

    if (st_object_ptr != nullptr) {
      // for object considered in speed
      const bool is_lane_change = IsPerformLaneChange(
          planner_output.scheduler_output.lane_change_state.stage());
      const double start_check_time = is_lane_change
                                          ? kCheckIsLeaderObjectStartTimeForLc
                                          : kCheckIsLeaderObjectStartTimeForLk;
      const double end_check_time = is_lane_change
                                        ? kCheckIsLeaderObjectEndTimeForLc
                                        : kCheckIsLeaderObjectEndTimeForLk;
      const auto init_decision_type =
          (*st_object_ptr)->GetDecisionTypeAtTime(start_check_time);
      const auto future_decision_type =
          (*st_object_ptr)->GetDecisionTypeAtTime(end_check_time);
      if (!init_decision_type.has_value() ||
          *init_decision_type !=
              PartialSpacetimeObjectTrajectory::DecisionType::FOLLOW ||
          !future_decision_type.has_value() ||
          *future_decision_type !=
              PartialSpacetimeObjectTrajectory::DecisionType::FOLLOW) {
        continue;
      }
    } else {
      if (std::max(boundary_left_l - l_max, l_min - boundary_right_l) >
          allow_width) {
        // Out of target lane path or too small to block the ego vehicle.
        continue;
      }
    }
    block_obj_ids.emplace(traj.object_id());
  }
  return block_obj_ids;
}

LeaderInfo FindNearestLeader(
    const absl::flat_hash_set<std::string>& block_ids,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const EstPlannerOutput& planner_output) {
  LeaderInfo nearest_leader;
  nearest_leader.obj_s = DBL_MAX;
  absl::flat_hash_set<std::string> has_checked_set;
  for (const auto& traj : st_traj_mgr.trajectories()) {
    // Remove duplicated object id
    if (has_checked_set.contains(traj.object_id())) continue;
    has_checked_set.emplace(traj.object_id());
    if (!block_ids.contains(traj.object_id())) continue;

    ASSIGN_OR_CONTINUE(
        const auto aabbox,
        planner_output.scheduler_output.drive_passage.QueryFrenetBoxAtContour(
            traj.contour()));
    if (aabbox.center_s() < nearest_leader.obj_s) {
      nearest_leader.obj_s = aabbox.center_s();
      nearest_leader.obj_id = traj.object_id();
      nearest_leader.obj_v = traj.states().front().traj_point->v();
      nearest_leader.obj_type = traj.object_type();
      nearest_leader.is_stationary = traj.is_stationary();
      nearest_leader.is_stalled = stalled_objects.contains(traj.object_id());
    }
  }
  return nearest_leader;
}

LaneChangeType AnalyzeLaneChangeType(
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    bool is_paddle_lane_change, int final_chosen_idx,
    LaneChangeType last_lane_change_type) {
  // Not in lane change.
  if (final_chosen_idx < 0 ||
      !idx_traj_feature_output_map.contains(final_chosen_idx) ||
      !idx_traj_feature_output_map.at(final_chosen_idx)
           .is_perform_lane_change) {
    return LaneChangeType::NO_CHANGE;
  }
  // If the last lane change reason is not NO_CHANGE, return it directly.
  if (last_lane_change_type != LaneChangeType::NO_CHANGE) {
    return last_lane_change_type;
  }
  if (is_paddle_lane_change) {
    return LaneChangeType::PADDLE_CHANGE;
  }
  // Find lane keep idx.
  int lane_keep_idx = -1;
  for (const auto& [idx, traj_feature_output] : idx_traj_feature_output_map) {
    if (!traj_feature_output.is_perform_lane_change) {
      lane_keep_idx = idx;
      break;
    }
  }
  if (lane_keep_idx < 0) {
    return LaneChangeType::DEFAULT_CHANGE;
  }

  const auto& traj_feature_output =
      idx_traj_feature_output_map.at(lane_keep_idx);

  // Route lane change.
  if (traj_feature_output.lane_change_for_road_speed_limit) {
    return LaneChangeType::ROAD_SPEED_LIMIT_CHANGE;
  }
  if (traj_feature_output.lane_change_for_right_most_lane) {
    return LaneChangeType::ROAD_SPEED_LIMIT_CHANGE;
  }
  if (traj_feature_output.lane_change_for_route_cost) {
    return LaneChangeType::DEFAULT_ROUTE_CHANGE;
  }

  // Obstacle lane change.
  if (traj_feature_output.lane_change_for_moving_obj ||
      traj_feature_output.lane_change_for_stationary_vehicle) {
    return LaneChangeType::OVERTAKE_CHANGE;
  }
  if (traj_feature_output.lane_change_for_stalled_vehicle) {
    return LaneChangeType::STALLED_VEHICLE_CHANGE;
  }
  if (traj_feature_output.lane_change_for_stationary_obj) {
    return LaneChangeType::OBSTACLE_CHANGE;
  }

  return LaneChangeType::DEFAULT_CHANGE;
}

void ProcessAutoLaneChangeRequest(
    absl::Time plan_time, int best_traj_idx,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorState* selector_state) {
  const bool already_send_request =
      selector_state->selector_lane_change_request.lane_change_type() !=
      LaneChangeType::NO_CHANGE;
  if (already_send_request) return;
  const auto lane_change_request_type = AnalyzeLaneChangeType(
      idx_traj_feature_output_map, /*is_paddle_lane_change=*/false,
      best_traj_idx,
      selector_state->selector_lane_change_request.lane_change_type());
  const double time_since_last_reject_time =
      selector_state->last_user_reject_alc_time.has_value()
          ? absl::ToDoubleSeconds(plan_time -
                                  *selector_state->last_user_reject_alc_time)
          : DBL_MAX;
  const bool reach_cool_down_time =
      time_since_last_reject_time >
      selector_flags.planner_alc_request_reject_cool_down_time;
  // Avoid the same lane change request after being rejectd.
  if (!selector_flags.planner_need_to_lane_change_confirmation ||
      ConvertLaneChangeTypeToGeneralType(lane_change_request_type) !=
          ConvertLaneChangeTypeToGeneralType(
              selector_state->last_user_reject_alc_type) ||
      reach_cool_down_time) {
    selector_state->selector_lane_change_request.set_lane_change_type(
        lane_change_request_type);
    selector_state->selector_lane_change_request.set_lc_left(
        idx_traj_feature_output_map.at(best_traj_idx).lane_change_left);
    SendAutoLaneChangeRequestEvent(
        selector_state->selector_lane_change_request);
  }

  // Reset lane change request.
  if (!selector_flags.planner_need_to_lane_change_confirmation) {
    selector_state->selector_lane_change_request.set_lane_change_type(
        LaneChangeType::NO_CHANGE);
  }
}

void SendAutoLaneChangeRequestEvent(
    const SelectorLaneChangeRequestProto& selector_lane_change_request) {
  QLOG(INFO) << "Send auto lane change request event: "
             << LaneChangeType_Name(
                    selector_lane_change_request.lane_change_type());
  QRunEvent::AutoLaneChangeRequestProto auto_lane_change_request;
  auto_lane_change_request.set_lc_left(selector_lane_change_request.lc_left());
  auto_lane_change_request.set_lane_change_type(
      selector_lane_change_request.lane_change_type());
  auto_lane_change_request.set_lane_change_general_type(
      ConvertLaneChangeTypeToGeneralType(
          selector_lane_change_request.lane_change_type()));
  QRUNEVENT_WITH_PROTO_NOTICE(QRunEvent::KEY_QEVENT_AUTO_LANE_CHANGE_REQUEST,
                              auto_lane_change_request);
}

bool IsPerformLaneChange(const LaneChangeStage& lc_stage) {
  return lc_stage == LaneChangeStage::LCS_PAUSE ||
         lc_stage == LaneChangeStage::LCS_EXECUTING;
}

TargetLaneStateProto GenerateTargetLaneState(
    const PlannerSemanticMapManager& psmm, const SchedulerOutput& output) {
  TargetLaneStateProto target_lane_state;
  target_lane_state.set_is_borrow(output.borrow_lane);
  target_lane_state.set_is_fallback(output.is_fallback);
  target_lane_state.mutable_lane_ids()->Reserve(
      output.drive_passage.lane_path().lane_ids().size());
  for (const auto& lane_id : output.drive_passage.lane_path().lane_ids()) {
    target_lane_state.add_lane_ids(lane_id.value());
  }
  constexpr double kPreviewLanePathLength = 50.0;  // .m
  double distance = 0.0;
  const auto sample_center_points =
      SampleLanePathPoints(psmm, output.drive_passage.lane_path());
  target_lane_state.mutable_center_points()->Reserve(
      sample_center_points.size());
  for (int i = 0; i < sample_center_points.size(); ++i) {
    Vec2dToProto(sample_center_points[i],
                 target_lane_state.add_center_points());
    if (i > 0) {
      distance +=
          sample_center_points[i].DistanceTo(sample_center_points[i - 1]);
    }
    if (distance > kPreviewLanePathLength) {
      break;
    }
  }
  target_lane_state.set_successive_count(1);
  return target_lane_state;
}

int FindLastSelectedTrjectory(const PlannerSemanticMapManager& psmm,
                              const std::vector<PlannerStatus>& est_status,
                              const std::vector<EstPlannerOutput>& results,
                              const SelectorState& selector_state) {
  int last_selected_idx = -1;
  for (int idx = 0; idx < results.size(); ++idx) {
    if (!est_status[idx].ok()) continue;
    const auto lane_state =
        GenerateTargetLaneState(psmm, results[idx].scheduler_output);
    if (selector_state.best_target_lane_state.has_successive_count() &&
        IsSameTargetLane(selector_state.selected_target_lane_state,
                         lane_state)) {
      last_selected_idx = idx;
      break;
    }
  }
  return last_selected_idx;
}

int ChooseLaneKeepTrajDirectly(
    bool planner_is_l4_mode, absl::Time plan_time, int last_selected_idx,
    const SelectorState& selector_state, const SelectorFlags& selector_flags,
    const absl::flat_hash_map<mapping::ElementId, int>& lane_id_idx_map,
    const absl::flat_hash_map<mapping::ElementId, FeatureCostSum>&
        lane_id_cost_map,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    const std::vector<EstPlannerOutput>& results) {
  int best_lane_keep_idx =
      FindBestLaneKeepTrajectory(results, lane_id_idx_map, lane_id_cost_map);
  if (best_lane_keep_idx == -1) {
    return -1;
  }

  // For the first frame in noa, we need to choose lane keep trajectory.
  const auto activate_selector_time =
      selector_state.activate_selector_time.has_value()
          ? *selector_state.activate_selector_time
          : absl::InfinitePast();
  const double time_after_activate_selector =
      absl::ToDoubleSeconds(plan_time - activate_selector_time);
  if (!planner_is_l4_mode &&
      time_after_activate_selector <
          selector_flags.planner_allow_lc_time_after_activate_selector) {
    QLOG(INFO) << "Choose lane keep trajectory directly in noa, because "
                  "selector was just activated "
               << time_after_activate_selector << " seconds ago.";
    return best_lane_keep_idx;
  }

  // If there is a lane change request waitting, we need to choose lane keep
  // trajectory.
  if (selector_state.selector_lane_change_request.lane_change_type() !=
      LaneChangeType::NO_CHANGE) {
    QLOG(INFO) << "Choose lane keep trajectory directly in noa, because "
                  "there is a lane change request waitting.";
    return best_lane_keep_idx;
  }

  // In noa mode and no need to force route change
  // when lane change was just given up, we need to choose lane keep.
  const auto give_up_lane_change_time =
      selector_state.give_up_lane_change_time.has_value()
          ? *selector_state.give_up_lane_change_time
          : absl::InfinitePast();
  const double time_after_give_up_lane_change =
      absl::ToDoubleSeconds(plan_time - give_up_lane_change_time);
  const bool need_to_force_route_change =
      idx_traj_feature_output_map.at(best_lane_keep_idx).has_obvious_route_cost;
  if (!planner_is_l4_mode && !need_to_force_route_change &&
      time_after_give_up_lane_change <
          selector_flags.planner_allow_lc_time_after_give_up_lc) {
    if (last_selected_idx != -1 &&
        results[last_selected_idx].scheduler_output.lane_change_state.stage() ==
            LaneChangeStage::LCS_EXECUTING) {
      QLOG(INFO) << "We can continue lane change";
    } else {
      QLOG(INFO) << "Choose lane keep trajectory directly in noa, because "
                    "lane change was just given up "
                 << time_after_give_up_lane_change << " seconds ago.";
      return best_lane_keep_idx;
    }
  }

  // In noa mode disable opposite lane change after paddle lane change.
  const auto paddle_lane_change_time =
      selector_state.last_lc_info.has_lane_change_time() &&
              selector_state.last_lc_info.lane_change_type() ==
                  LaneChangeType::PADDLE_CHANGE
          ? qcraft::FromProto(selector_state.last_lc_info.lane_change_time())
          : absl::InfinitePast();
  const auto time_after_paddle_lane_change =
      absl::ToDoubleSeconds(plan_time - paddle_lane_change_time);
  if (!planner_is_l4_mode &&
      time_after_paddle_lane_change <
          selector_flags.planner_allow_opposite_lc_time_after_paddle_lc) {
    bool is_opposite_lc = false;
    for (const auto& [_, traj_feature_output] : idx_traj_feature_output_map) {
      if (traj_feature_output.is_perform_lane_change &&
          traj_feature_output.lane_change_left !=
              selector_state.last_lc_info.lc_left()) {
        is_opposite_lc = true;
        break;
      }
    }
    if (is_opposite_lc) {
      QLOG(INFO) << "Choose lane keep trajectory directly in noa, because "
                    "opposite paddle lane change was just finished "
                 << time_after_paddle_lane_change << " seconds ago.";
      return best_lane_keep_idx;
    }
  }
  return -1;
}

void FillSelectorOutputToDebug(const SelectorOutput& selector_output,
                               SelectorDebugProto* selector_debug) {
  selector_debug->mutable_selector_output()->set_selected_idx(
      selector_output.selected_idx);
  selector_debug->mutable_selector_output()->set_best_traj_idx(
      selector_output.best_traj_idx);
  selector_debug->mutable_selector_output()->set_last_selected_idx(
      selector_output.last_selected_idx);
  selector_debug->mutable_selector_output()->set_turn_signal(
      selector_output.turn_signal);
  selector_debug->mutable_selector_output()->set_all_trajectories_blocked(
      selector_output.all_trajectories_blocked);
  if (selector_output.is_going_force_route_change_left.has_value()) {
    selector_debug->mutable_selector_output()
        ->set_is_going_force_route_change_left(
            *selector_output.is_going_force_route_change_left);
  }
  selector_debug->mutable_selector_output()->set_lane_change_for_obstacle_fail(
      selector_output.lane_change_for_obstacle_fail);
}

}  // namespace planner
}  // namespace qcraft
