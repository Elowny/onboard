#include "onboard/planner/selector/selector.h"

#include <float.h>

#include <algorithm>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/candidate_stats.h"
#include "onboard/planner/selector/common_feature.h"
#include "onboard/planner/selector/cost_feature_base.h"
#include "onboard/planner/selector/proto/selector_params.pb.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector_defs.h"
#include "onboard/planner/selector/selector_util.h"
#include "onboard/planner/selector/traj_cost_features.h"
#include "onboard/proto/route.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
SelectorCommonFeature BuildSelectorCommonFeature(
    const SelectorInput& input,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results) {
  SelectorCommonFeature common_feature;
  const double motion_speed_limit =
      Mph2Mps(input.motion_constraints->default_speed_limit());
  const auto& lane_navi_info_map = input.route_navi_info->route_lane_info_map;
  constexpr double kConsiderHighWaySpeedLimit = 20.83;  // m/s.
  constexpr double kPreviewDistance = 320.0;            // m.
  constexpr double kPreviewSpeedLimitInterval = 100.0;  // m.
  for (int idx = 0; idx < results.size(); ++idx) {
    if (results[idx].scheduler_output.drive_passage.lane_path().IsEmpty()) {
      continue;
    }
    LaneFeatureInfo lane_feature;
    const auto& result = results[idx];
    const auto& passage = result.scheduler_output.drive_passage;
    const auto start_lane_id = passage.lane_path().front().lane_id();
    lane_feature.speed_limit = motion_speed_limit;
    for (double preview_distance = 0.0; preview_distance < kPreviewDistance;
         preview_distance += kPreviewSpeedLimitInterval) {
      ASSIGN_OR_CONTINUE(const double preview_speed_limit,
                         passage.QuerySpeedLimitAtS(preview_distance));
      lane_feature.speed_limit =
          std::min(lane_feature.speed_limit, preview_speed_limit);
    }
    common_feature.in_high_way |=
        lane_feature.speed_limit >= kConsiderHighWaySpeedLimit;
    // Lane route info.
    const auto* lane_navi_info_ptr =
        FindOrNull(lane_navi_info_map, start_lane_id);
    if (lane_navi_info_ptr != nullptr) {
      lane_feature.lc_num_to_targets = lane_navi_info_ptr->min_lc_num_to_target;
      lane_feature.lc_num_within_driving_dist =
          lane_navi_info_ptr->lc_num_within_driving_dist;
      lane_feature.driving_dist = lane_navi_info_ptr->max_reach_length;
      lane_feature.len_before_merge_lane =
          lane_navi_info_ptr->len_before_merge_lane;
      lane_feature.merge_targets = lane_navi_info_ptr->merge_targets;
    }
    // Obstacle info.
    lane_feature.block_obj_ids =
        FindBlockObjectIds(st_traj_mgr_list[idx], result, *input.vehicle_geom);
    lane_feature.front_non_block_obj_ids = FindFrontNonBlockObjectIds(
        st_traj_mgr_list[idx], result, *input.vehicle_geom,
        *input.plan_start_point, lane_feature.block_obj_ids);
    if (lane_feature.block_obj_ids.size() > 0) {
      lane_feature.nearest_leader =
          FindNearestLeader(lane_feature.block_obj_ids, st_traj_mgr_list[idx],
                            *input.stalled_objects, result);
    }
    common_feature.lane_feature_infos[result.scheduler_output.Hash()] =
        std::move(lane_feature);
  }

  // Update common feature for all branch.
  common_feature.in_high_way |= input.route_navi_info->in_highway;

  const auto& front_sec_id = input.sections_info->front().id;
  const auto* navi_section_info_ptr =
      FindOrNull(input.route_navi_info->navi_section_info_map, front_sec_id);
  if (navi_section_info_ptr == nullptr) {
    common_feature.length_before_intersection = 0.0;
    common_feature.is_left_turn = false;
    common_feature.is_right_turn = false;
  } else {
    common_feature.length_before_intersection =
        navi_section_info_ptr->length_before_intersection;
    common_feature.is_left_turn =
        navi_section_info_ptr->intersection_direction ==
        NaviSectionInfoProto::LEFT_TURN;
    common_feature.is_right_turn =
        navi_section_info_ptr->intersection_direction ==
        NaviSectionInfoProto::RIGHT_TURN;
  }

  return common_feature;
}

CostFeatures BuildCostFeatures(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const SelectorState& selector_state,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results) {
  QCHECK(input.config->has_cost_config());

  CostFeatures cost_features;
  const auto& cost_config = input.config->cost_config();
  if (cost_config.enable_progress_cost()) {
    cost_features.emplace_back(std::make_unique<TrajProgressCost>(
        &common_feature, input.selector_flags->planner_lane_change_style,
        input.selector_flags->planner_enable_obstacle_lane_change,
        ProgressStats(common_feature, *input.plan_start_point,
                      *input.vehicle_geom, est_status, st_traj_mgr_list,
                      results)));
  }
  if (cost_config.enable_max_jerk_cost()) {
    cost_features.emplace_back(std::make_unique<TrajMaxJerkCost>(
        &common_feature, *input.motion_constraints));
  }
  if (cost_config.enable_lane_change_cost()) {
    double time_since_last_red_light = DBL_MAX;
    double time_since_last_lane_change = DBL_MAX;
    if (input.selector_state != nullptr) {
      if (input.selector_state->last_redlight_stop_time.has_value()) {
        time_since_last_red_light = absl::ToDoubleSeconds(
            input.plan_time - *input.selector_state->last_redlight_stop_time);
      }
      if (input.selector_state->last_lc_info.has_lane_change_time()) {
        time_since_last_lane_change = absl::ToDoubleSeconds(
            input.plan_time -
            qcraft::FromProto(
                input.selector_state->last_lc_info.lane_change_time()));
      }
    }
    cost_features.emplace_back(std::make_unique<TrajLaneChangeCost>(
        &common_feature, input.psmm, input.prev_lane_path_from_current,
        input.prev_traj, *input.plan_start_point,
        input.selector_state->last_lc_info, time_since_last_red_light,
        time_since_last_lane_change,
        input.selector_flags->planner_enable_lane_change_in_intersection,
        selector_state.turn_signal != TurnSignal::TURN_SIGNAL_NONE));
  }
  if (cost_config.enable_solid_boundary_cost()) {
    cost_features.emplace_back(std::make_unique<TrajCrossSolidBoundaryCost>(
        &common_feature, *input.vehicle_geom,
        input.selector_flags->planner_enable_cross_solid_boundary));
  }
  if (cost_config.enable_route_look_ahead_cost()) {
    bool use_conservative_ttc = false;
    if (selector_state.route_ttc_setting.route_request_state() ==
        RouteTtcRequestState::RECEIVED_RESPONSE) {
      use_conservative_ttc =
          selector_state.route_ttc_setting.response_config() ==
          RouteTtcConfig::CONSERVATIVE;
    }
    RouteLookAheadStats route_stats(
        common_feature, *input.psmm, *input.sections_info,
        *input.plan_start_point, *input.stalled_objects, est_status,
        st_traj_mgr_list, results);
    cost_features.emplace_back(std::make_unique<TrajRouteLookAheadCost>(
        &common_feature, route_stats,
        input.selector_flags->planner_is_bus_model, use_conservative_ttc));
  }
  if (cost_config.enable_boundary_expansion_cost()) {
    cost_features.emplace_back(std::make_unique<TrajBoundaryExpansionCost>(
        &common_feature, *input.plan_start_point));
  }
  return cost_features;
}

WeightTable BuildWeightTable(
    const SelectorParamsProto::TrajCostWeights& weight_config,
    const CostFeatures& cost_features) {
  WeightTable weight_table;
  const auto* reflection = weight_config.GetReflection();
  const auto* descriptor = weight_config.GetDescriptor();
  for (const auto& feature : cost_features) {
    auto* feature_desc = descriptor->FindFieldByName(feature->name());
    const auto& feature_conf =
        reflection->GetMessage(weight_config, feature_desc);
    const auto* feature_ref = feature_conf.GetReflection();
    std::vector<const google::protobuf::FieldDescriptor*> feature_weights_desc;
    feature_ref->ListFields(feature_conf, &feature_weights_desc);
    auto& weight_vec = weight_table[feature->name()];
    weight_vec.resize(feature->size());
    int idx = 0;
    for (const auto* desc : feature_weights_desc) {
      weight_vec[idx++] = feature_ref->GetDouble(feature_conf, desc);
    }
  }
  return weight_table;
}

FeatureCostSum ComputeTrajCost(const CostFeatures& cost_features,
                               const WeightTable& weights,
                               const EstPlannerOutput& planner_output,
                               TrajFeatureOutput* traj_feature_output,
                               TrajectoryCost* traj_cost) {
  FeatureCostSum cost_res{0.0, 0.0};
  constexpr double kInvalidCost = 1e5;
  for (const auto& feature : cost_features) {
    std::vector<std::string> extra_info;
    const auto cost_vec_or =
        feature->ComputeCost(planner_output, &extra_info, traj_feature_output);
    if (!cost_vec_or.ok()) {
      const double feat_cost = kInvalidCost;
      const std::string error_msg =
          absl::StrFormat("%s when calculate %s cost.",
                          cost_vec_or.status().message(), feature->name());
      auto& feat_debug = *traj_cost->add_features();
      feat_debug.set_name(feature->name());
      feat_debug.set_cost(feat_cost);
      feat_debug.add_extra_info(error_msg);
      cost_res.add(feat_cost, /*is_common=*/true);
      QLOG(WARNING) << error_msg;
    } else {
      const auto& cost_vec = *cost_vec_or;
      const auto weighted_cost =
          MultiplyVector(FindOrDie(weights, feature->name()), cost_vec);
      const double feat_cost =
          std::accumulate(weighted_cost.begin(), weighted_cost.end(), 0.0);
      auto& feat_debug = *traj_cost->add_features();
      feat_debug.set_name(feature->name());
      feat_debug.set_cost(feat_cost);
      for (auto& info : extra_info) {
        *feat_debug.add_extra_info() = std::move(info);
      }
      for (int i = 0; i < feature->size(); ++i) {
        feat_debug.add_cost_value_info(absl::StrFormat(
            "%s, value: %.2f, weighted: %.2f", feature->sub_names()[i],
            cost_vec[i], weighted_cost[i]));
      }
      cost_res.add(feat_cost, feature->is_common());
    }
  }
  traj_cost->set_start_lane_id(
      planner_output.scheduler_output.drive_passage.lane_path()
          .front()
          .lane_id()
          .value());
  traj_cost->set_sum_common(cost_res.cost_common);
  traj_cost->set_sum(cost_res.cost_sum());
  traj_cost->set_lc_stage(
      planner_output.scheduler_output.lane_change_state.stage());

  return cost_res;
}

absl::flat_hash_map<int, FeatureCostSum> CalculateTrajectoryCost(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    const SelectorCommonFeature& selector_common_feature,
    const SelectorState& selector_state,
    absl::flat_hash_map<int, TrajFeatureOutput>* idx_traj_feature_output_map,
    SelectorDebugProto* selector_debug) {
  absl::flat_hash_map<int, FeatureCostSum> lane_id_cost_map;

  const auto cost_features =
      BuildCostFeatures(input, selector_common_feature, selector_state,
                        est_status, st_traj_mgr_list, results);
  const auto weights =
      BuildWeightTable(input.config->cost_weights(), cost_features);

  lane_id_cost_map.reserve(results.size());
  idx_traj_feature_output_map->reserve(results.size());
  for (int idx = 0; idx < results.size(); ++idx) {
    if (!est_status[idx].ok()) continue;

    TrajFeatureOutput traj_feature_output;
    auto* traj_cost_debug = selector_debug->add_traj_costs();
    auto cur_cost = ComputeTrajCost(cost_features, weights, results[idx],
                                    &traj_feature_output, traj_cost_debug);
    lane_id_cost_map[idx] = cur_cost;
    idx_traj_feature_output_map->emplace(idx, traj_feature_output);
  }
  return lane_id_cost_map;
}

int FindBestTrajectory(
    absl::Time plan_time, int last_selected_idx,
    const PlannerSemanticMapManager& psmm,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results,
    const SelectorFlags& selector_flags,
    const absl::flat_hash_map<mapping::ElementId, int>& lane_id_idx_map,
    const absl::flat_hash_map<int, int>& idx_selector_debug_map,
    const absl::flat_hash_map<mapping::ElementId, FeatureCostSum>&
        lane_id_cost_map,
    const absl::flat_hash_map<int, TrajFeatureOutput>&
        idx_traj_feature_output_map,
    SelectorDebugProto* selector_debug, SelectorState* selector_state,
    SelectorOutput* selector_output) {
  int best_traj_idx = ChooseLaneKeepTrajDirectly(
      selector_flags.planner_is_l4_mode, plan_time, last_selected_idx,
      *selector_state, selector_flags, lane_id_idx_map, lane_id_cost_map,
      idx_traj_feature_output_map, results);
  int final_selected_idx = -1;

  if (best_traj_idx == -1) {
    // Choose best trajectory.
    double best_cost = DBL_MAX;
    // Only compare common cost features here.
    for (const auto& [lane_id, cost] : lane_id_cost_map) {
      if (cost.cost_common < best_cost) {
        best_traj_idx = lane_id_idx_map.at(lane_id);
        best_cost = cost.cost_common;
      }
    }
  }
  auto best_target_lane_state =
      GenerateTargetLaneState(psmm, results[best_traj_idx].scheduler_output);
  int successive_count = 1;
  if (IsSameTargetLane(selector_state->best_target_lane_state,
                       best_target_lane_state)) {
    successive_count =
        selector_state->best_target_lane_state.successive_count() + 1;
  }
  best_target_lane_state.set_successive_count(successive_count);
  selector_state->best_target_lane_state = std::move(best_target_lane_state);

  // Update selector state and find final chosen traj.
  const auto last_pre_turn_signal = selector_state->pre_turn_signal;
  const int begin_lane_change_frame =
      DecideBeginLaneChangeFrame(selector_flags, idx_traj_feature_output_map);
  selector_state->pre_turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  selector_state->lc_prepare_stage_lane_path = std::nullopt;
  selector_output->best_traj_idx = best_traj_idx;
  final_selected_idx = best_traj_idx;
  bool need_to_send_lane_change_request = false;
  if (last_selected_idx != -1 && last_selected_idx != best_traj_idx) {
    // Need to choose a new policy.
    const bool is_ready = successive_count >= begin_lane_change_frame;
    const auto& target_lane_change_state =
        results[best_traj_idx].scheduler_output.lane_change_state;
    const bool is_going_lane_change =
        IsPerformLaneChange(target_lane_change_state.stage());
    if (selector_flags.planner_need_to_lane_change_confirmation) {
      if (!is_going_lane_change) {
        final_selected_idx = is_ready ? best_traj_idx : last_selected_idx;
      } else {
        final_selected_idx = last_selected_idx;
        need_to_send_lane_change_request = is_ready;
      }
    } else {
      final_selected_idx = is_ready ? best_traj_idx : last_selected_idx;
      // Set pre turn signal before lane change.
      if (!is_ready &&
          successive_count >= selector_flags.planner_begin_signal_frame) {
        if (is_going_lane_change) {
          selector_state->pre_turn_signal = target_lane_change_state.lc_left()
                                                ? TURN_SIGNAL_LEFT
                                                : TURN_SIGNAL_RIGHT;
          selector_state->lc_prepare_stage_lane_path =
              results[best_traj_idx].scheduler_output.drive_passage.lane_path();
        }
      }
      // Send lane change request in noa when lane change or pre lane change.
      if (is_going_lane_change && !selector_flags.planner_is_l4_mode &&
          last_pre_turn_signal == TURN_SIGNAL_NONE) {
        need_to_send_lane_change_request =
            successive_count >=
            std::min(selector_flags.planner_begin_signal_frame,
                     begin_lane_change_frame);
      }
    }
  }

  selector_state->selected_target_lane_state = GenerateTargetLaneState(
      psmm, results[final_selected_idx].scheduler_output);

  // Update selector debug.
  for (int idx = 0; idx < results.size(); ++idx) {
    if (!est_status[idx].ok()) continue;

    const int selector_debug_idx = idx_selector_debug_map.at(idx);
    auto& selector_state_debug =
        *(selector_debug->mutable_traj_costs(selector_debug_idx)
              ->mutable_selector_state());
    selector_state_debug.set_best_traj_time(
        idx == best_traj_idx ? successive_count : 0);
    selector_state_debug.set_best_threshold(begin_lane_change_frame);
    selector_state_debug.set_is_last_traj(idx == last_selected_idx);
    selector_state_debug.set_is_final_traj(idx == final_selected_idx);
  }

  if (need_to_send_lane_change_request) {
    ProcessAutoLaneChangeRequest(plan_time, best_traj_idx, selector_flags,
                                 idx_traj_feature_output_map, selector_state);
  }

  return final_selected_idx;
}

}  // namespace

absl::StatusOr<SelectorOutput> SelectTrajectory(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    SelectorDebugProto* selector_debug, SelectorState* selector_state) {
  SCOPED_QTRACE("SelectTrajectory");
  // 0. Check selector input.
  if (selector_debug == nullptr || selector_state == nullptr) {
    return absl::InvalidArgumentError(
        "Input selector_debug or selector_state is null!");
  }
  bool has_valid_trajectory = false;
  for (int i = 0; i < results.size(); ++i) {
    if (est_status[i].ok()) {
      has_valid_trajectory = true;
      break;
    }
  }
  if (!has_valid_trajectory) {
    std::stringstream fail_reason;
    fail_reason << "No valid trajectory added for selection:\n";
    for (int idx = 0; idx < est_status.size(); ++idx) {
      fail_reason << "Task " << idx << ": " << est_status[idx].message();
    }
    return absl::NotFoundError(fail_reason.str());
  }
  if (input.selector_state != nullptr) {
    *selector_state = *input.selector_state;
  }

  // 1.Calculate traj cost for each lane.
  SelectorOutput selector_output;
  absl::flat_hash_map<mapping::ElementId, int> lane_id_idx_map;
  absl::flat_hash_map<int, int> idx_selector_debug_map;
  absl::flat_hash_map<mapping::ElementId, FeatureCostSum> lane_id_cost_map;
  absl::flat_hash_map<int, TrajFeatureOutput> idx_traj_feature_output_map;
  UpdateSelectorStateBeforeSelection(input.plan_time, selector_state);
  const int last_selected_idx = FindLastSelectedTrjectory(
      *input.psmm, est_status, results, *selector_state);
  const auto selector_common_feature =
      BuildSelectorCommonFeature(input, st_traj_mgr_list, results);
  const auto prefilter_est_status = PreFilterEstResults(
      input.plan_time, *input.psmm, *input.selector_flags,
      *input.stalled_objects, est_status, results, selector_common_feature,
      last_selected_idx, selector_debug, selector_state);
  const auto all_trajectory_cost =
      CalculateTrajectoryCost(input, prefilter_est_status, st_traj_mgr_list,
                              results, selector_common_feature, *selector_state,
                              &idx_traj_feature_output_map, selector_debug);
  UpdateTrajectoryCostForEachLane(prefilter_est_status, results,
                                  all_trajectory_cost, &lane_id_cost_map,
                                  &lane_id_idx_map, &idx_selector_debug_map);
  UpdateRouteTtcSettingInHighway(input.alc_confirmation, last_selected_idx,
                                 *input.selector_flags,
                                 idx_traj_feature_output_map, selector_state);
  HandleAlcConfirmation(input.plan_time, input.alc_confirmation,
                        selector_state);

  // 2. Find best trajectory.
  const int final_selected_idx = FindBestTrajectory(
      input.plan_time, last_selected_idx, *input.psmm, prefilter_est_status,
      results, *input.selector_flags, lane_id_idx_map, idx_selector_debug_map,
      lane_id_cost_map, idx_traj_feature_output_map, selector_debug,
      selector_state, &selector_output);

  if (final_selected_idx < 0) {
    return absl::NotFoundError("No valid trajectory selected!");
  }

  // 3. Update selector state and selector output.
  const bool is_paddle_lane_change = input.preferred_lane_path != nullptr &&
                                     !input.preferred_lane_path->IsEmpty();
  UpdateSelectorStateAfterSelection(
      input.plan_time, *input.selector_flags, results, prefilter_est_status,
      idx_traj_feature_output_map, final_selected_idx,
      selector_output.best_traj_idx, is_paddle_lane_change, selector_state);
  UpdateSelectorOutput(results, prefilter_est_status, *selector_state,
                       idx_traj_feature_output_map,
                       selector_common_feature.in_high_way, final_selected_idx,
                       last_selected_idx, &selector_output);
  FillSelectorOutputToDebug(selector_output, selector_debug);
  return selector_output;
}

}  // namespace qcraft::planner
