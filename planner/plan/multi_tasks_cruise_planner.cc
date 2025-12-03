#include "onboard/planner/plan/multi_tasks_cruise_planner.h"

// IWYU pragma: no_include <cxxabi.h>  // for __forced_unwind
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include "onboard/lite/qissue_trans.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/qalc.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/run_context.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/hmi_util.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/captain_net/captain_net_generator.h"
#include "onboard/planner/ml/expert/expert_planner.h"
#include "onboard/planner/ml/expert/expert_planner_util.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/planner_object_util.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/plan/fallback_planner.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/multi_tasks_scheduler.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_input.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/scheduler/target_lane_path_filter.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/planner/selector/proto/selector_params.pb.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector.h"
#include "onboard/planner/selector/selector_input.h"
#include "onboard/planner/selector/selector_output.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/lane_change_type.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

namespace {

inline bool IsGoingLaneChangeStage(LaneChangeStage stage) {
  switch (stage) {
    case LCS_NONE:
    case LCS_WAITING:
    case LCS_RETURN:
      return false;
    case LCS_EXECUTING:
    case LCS_PAUSE:
      return true;
  }
}
absl::StatusOr<int> FindRouteTargetIndex(
    absl::Span<const EstPlannerOutput> est_outputs,
    const mapping::LanePath& preferred_lane_path,
    std::optional<bool> is_going_lc_left) {
  if (!preferred_lane_path.IsEmpty()) {
    absl::flat_hash_set<mapping::ElementId> preferred_lanes(
        preferred_lane_path.lane_ids().begin(),
        preferred_lane_path.lane_ids().end());
    for (int i = 0; i < est_outputs.size(); ++i) {
      if (preferred_lanes.contains(
              est_outputs[i]
                  .scheduler_output.drive_passage.lane_path()
                  .front()
                  .lane_id())) {
        return i;
      }
    }
  }

  if (is_going_lc_left.has_value()) {
    for (auto it = est_outputs.begin(); it != est_outputs.end(); ++it) {
      const auto& lc_state = it->scheduler_output.lane_change_state;
      if (IsGoingLaneChangeStage(lc_state.stage()) &&
          lc_state.lc_left() == *is_going_lc_left) {
        return std::distance(est_outputs.begin(), it);
      }
    }
  }

  return absl::NotFoundError("Can not found target route.");
}

std::optional<RouteTargetInfo> FindRouteTargetInfo(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& preferred_lane_path, const Box2d& ego_box,
    absl::Span<const EstPlannerOutput> est_outputs,
    absl::Span<const SpacetimeTrajectoryManager> st_traj_mgr_list,
    std::optional<bool> is_going_lc_left, std::optional<Vec2d> merge_point) {
  ASSIGN_OR_RETURN(
      const auto route_target_index,
      FindRouteTargetIndex(est_outputs, preferred_lane_path, is_going_lc_left),
      std::nullopt);

  const auto& target_scheduler =
      est_outputs[route_target_index].scheduler_output;
  const auto target_lane_path_ext = BackwardExtendLanePath(
      psmm,
      target_scheduler.drive_passage.extend_lane_path().BeforeArclength(
          kLaneChangeCheckForwardLength),
      kLaneChangeCheckBackwardLength);
  auto target_frenet_frame_or =
      BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                             /*down_sample_raw_points=*/true);
  if (!target_frenet_frame_or.ok()) return std::nullopt;

  ASSIGN_OR_RETURN(const auto ego_frenet_box,
                   target_frenet_frame_or->QueryFrenetBoxAt(ego_box),
                   std::nullopt);

  return RouteTargetInfo{
      .plan_id = route_target_index + 1,
      .frenet_frame = std::move(target_frenet_frame_or).value(),
      .ego_frenet_box = ego_frenet_box,
      .drive_passage = target_scheduler.drive_passage,
      .sl_boundary = target_scheduler.sl_boundary,
      .st_traj_mgr = st_traj_mgr_list[route_target_index],
      .merge_point = std::move(merge_point)};
}

void AppendFallbackToResultList(
    const PlannerStatus& fallback_status, FallbackPlannerOutput fallback_result,
    EstPlannerDebug fallback_debug,
    vis::vantage::ChartDataBundleProto fallback_chart_data,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    std::vector<PlannerStatus>* status_list,
    std::vector<EstPlannerOutput>* results,
    std::vector<EstPlannerDebug>* debug_list,
    std::vector<vis::vantage::ChartDataBundleProto>* chart_data_bundles) {
  st_traj_mgr_list->push_back(std::move(fallback_result.filtered_traj_mgr));
  status_list->push_back(fallback_status);
  debug_list->push_back(std::move(fallback_debug));
  chart_data_bundles->push_back(std::move(fallback_chart_data));
  results->emplace_back(EstPlannerOutput{
      .scheduler_output = std::move(fallback_result.scheduler_output),
      .path = std::move(fallback_result.path),
      .traj_points = std::move(fallback_result.trajectory_points),
      .st_path_points = std::move(fallback_result.st_path_points),
      .decider_state = std::move(fallback_result.decider_state),
      .considered_st_objects = std::move(fallback_result.considered_st_objects),
      .trajectory_end_info = std::move(fallback_result.trajectory_end_info)});
}

void PreFilterByPreferred(const PlannerSemanticMapManager& psmm,
                          const QALCState alc_state,
                          const mapping::LanePath& preferred_lane_path,
                          const std::vector<EstPlannerOutput>& est_results,
                          const VehicleGeometryParamsProto& vehicle_geom,
                          PlcInternalResult* plc_result,
                          std::vector<PlannerStatus>* status_list) {
  const auto& original_status_list = *status_list;
  plc_result->status = PlcInternalStatus::OK;
  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path.lane_ids().begin(),
      preferred_lane_path.lane_ids().end());
  int preferred_idx = -1;

  bool disable_all_others = false;
  const absl::Cleanup disable_other_status = [alc_state, &preferred_idx,
                                              &disable_all_others,
                                              &preferred_lanes, status_list]() {
    if (!disable_all_others &&
        !(alc_state == ALC_CROSSING_LANE || alc_state == ALC_RETURNING)) {
      return;
    }
    const std::string err_msg = absl::StrCat(
        "Planner branch ignored: teleop to lanes (",
        absl::StrJoin(preferred_lanes, ", "), ")",
        alc_state == ALC_CROSSING_LANE || alc_state == ALC_RETURNING
            ? absl::StrCat(" with state ", QALCState_Name(alc_state), ".")
            : ".");
    for (int i = 0; i < status_list->size(); ++i) {
      if (i == preferred_idx) continue;
      (*status_list)[i] =
          PlannerStatus(PlannerStatusProto::BRANCH_RESULT_IGNORED, err_msg);
    }
  };

  for (int i = 0; i < original_status_list.size(); ++i) {
    if (!preferred_lanes.contains(
            est_results[i]
                .scheduler_output.drive_passage.lane_path()
                .front()
                .lane_id())) {
      continue;
    }
    if (preferred_idx == -1 ||
        (original_status_list[i].ok() &&
         (!original_status_list[preferred_idx].ok() ||
          est_results[preferred_idx].scheduler_output.is_fallback))) {
      preferred_idx = i;
    }
  }
  if (preferred_idx == -1) {
    plc_result->status = PlcInternalStatus::BRANCH_NOT_FOUND;
    return;
  }

  const auto& preferred_status = original_status_list[preferred_idx];
  const auto& preferred_result = est_results[preferred_idx];
  const auto& preferred_scheduler = preferred_result.scheduler_output;
  if (!preferred_status.ok()) {
    if (preferred_status.status_code() !=
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
      plc_result->status = PlcInternalStatus::BRANCH_FAILED_INTERNAL;
    } else {
      plc_result->status = PlcInternalStatus::UNSAFE_OBJECT;
      plc_result->unsafe_object_ids = {
          preferred_result.unsafe_object_ids.begin(),
          preferred_result.unsafe_object_ids.end()};
      // Also find possible solid boundary points since we have no trajectory
      // here to judge whether they also affect plc.
      plc_result->left_solid_boundary =
          preferred_scheduler.lane_change_state.lc_left();
    }
    return;
  }
  if (!IsRunModeL4()) {
    const auto preferred_crossed_or = HasTrajectoryCrossedSolidBoundary(
        preferred_scheduler.drive_passage, preferred_scheduler.sl_boundary,
        preferred_result.traj_points, vehicle_geom,
        preferred_scheduler.lane_change_state.stage() ==
            LaneChangeStage::LCS_PAUSE,
        psmm.IsOnVisionMap());
    if (preferred_crossed_or.ok() && *preferred_crossed_or) {
      bool all_crossed_solid = true;
      for (int i = 0; i < original_status_list.size(); ++i) {
        if (i == preferred_idx || !original_status_list[i].ok()) continue;

        const auto& branch_scheduler = est_results[i].scheduler_output;
        const auto branch_crossed_or = HasTrajectoryCrossedSolidBoundary(
            branch_scheduler.drive_passage, branch_scheduler.sl_boundary,
            est_results[i].traj_points, vehicle_geom,
            branch_scheduler.lane_change_state.stage() ==
                LaneChangeStage::LCS_PAUSE,
            psmm.IsOnVisionMap());
        if (branch_crossed_or.ok() && !(*branch_crossed_or)) {
          all_crossed_solid = false;
          break;
        }
      }
      if (!all_crossed_solid) {
        plc_result->status = PlcInternalStatus::SOLID_BOUNDARY;
        plc_result->left_solid_boundary =
            preferred_scheduler.lane_change_state.lc_left();
        (*status_list)[preferred_idx] = PlannerStatus(
            PlannerStatusProto::BRANCH_RESULT_IGNORED,
            absl::StrCat("Planner branch ignored: crossing solid boundary."));
        return;
      }
    }
  }
  disable_all_others = true;
}

void PreFilterRedundant(const std::vector<EstPlannerOutput>& est_results,
                        std::vector<PlannerStatus>* status_list) {
  // If a normal branch (scheduled in this iteration) has ok status:
  //  - its fallback correspondence (if any) will not be considered,
  //  - its lc pause correspondence (if any) will not be considered,
  // otherwise:
  //  - if it has a lc pause correspondence, use it,
  //  - otherwise if it has a fallback correspondence, use it.

  const auto& original_status_list = *status_list;
  // Map succeeded normal branches' start lane id to whether executing lc.
  absl::flat_hash_map<mapping::ElementId, bool> id_stage_map;
  for (int i = 0; i < original_status_list.size(); ++i) {
    if (!original_status_list[i].ok() ||
        est_results[i].scheduler_output.is_fallback) {
      continue;
    }
    const auto start_lane_id = est_results[i]
                                   .scheduler_output.drive_passage.lane_path()
                                   .front()
                                   .lane_id();
    const auto [lc_executing, _] = id_stage_map.insert({start_lane_id, false});
    if (est_results[i].scheduler_output.lane_change_state.stage() ==
        LaneChangeStage::LCS_EXECUTING) {
      lc_executing->second = true;
    }
  }

  auto& considered_status_list = *status_list;
  for (int i = 0; i < considered_status_list.size(); ++i) {
    if (!considered_status_list[i].ok()) continue;

    const auto& scheduler_output = est_results[i].scheduler_output;
    const auto start_lane_id =
        scheduler_output.drive_passage.lane_path().front().lane_id();
    if (!id_stage_map.contains(start_lane_id)) continue;

    if (scheduler_output.is_fallback) {
      considered_status_list[i] = PlannerStatus(
          PlannerStatusProto::BRANCH_RESULT_IGNORED,
          "Fallback branch ignored: latest correspondence succeeded.");
    }
    if (scheduler_output.lane_change_state.stage() ==
            LaneChangeStage::LCS_PAUSE &&
        FindOrDie(id_stage_map, start_lane_id)) {
      considered_status_list[i] = PlannerStatus(
          PlannerStatusProto::BRANCH_RESULT_IGNORED,
          "LC pause branch ignored: lane change branch passed safety check.");
    }
  }
}

void FillFallbackPlannerResult(
    const PlannerStatus& fallback_status,
    const mapping::LanePath& updated_preferred_lane_path,
    FallbackPlannerOutput fallback_result, EstPlannerDebug fallback_debug,
    vis::vantage::ChartDataBundleProto fallback_chart_data,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    std::vector<PlannerStatus>* status_list,
    std::vector<EstPlannerOutput>* results,
    std::vector<EstPlannerDebug>* debug_list,
    std::vector<vis::vantage::ChartDataBundleProto>* chart_data_bundles) {
  // Fill the fallback result to be selected together if:
  // - fallback planner succeeds, and
  // - no teleop-required lane change is activated, and
  //   - it matches the start id of some est planner, or
  //   - no est planner succeeds.
  if (!fallback_status.ok() || !updated_preferred_lane_path.IsEmpty()) {
    return;
  }
  const bool est_any_success =
      std::any_of(status_list->begin(), status_list->end(),
                  [](const PlannerStatus& status) { return status.ok(); });
  if (!est_any_success) {
    AppendFallbackToResultList(
        fallback_status, std::move(fallback_result), std::move(fallback_debug),
        std::move(fallback_chart_data), st_traj_mgr_list, status_list, results,
        debug_list, chart_data_bundles);
  } else {
    const auto fallback_start_id =
        fallback_result.scheduler_output.drive_passage.lane_path()
            .front()
            .lane_id();
    for (const auto& result : *results) {
      if (!result.scheduler_output.drive_passage.empty() &&
          result.scheduler_output.drive_passage.lane_path().front().lane_id() ==
              fallback_start_id) {
        AppendFallbackToResultList(fallback_status, std::move(fallback_result),
                                   std::move(fallback_debug),
                                   std::move(fallback_chart_data),
                                   st_traj_mgr_list, status_list, results,
                                   debug_list, chart_data_bundles);
        return;
      }
    }
  }
}

absl::StatusOr<mapping::LanePath> AlignLanePathWithRouteSections(
    const RouteSectionsInfo& sections_info, const mapping::LanePath& lane_path,
    double proj_range) {
  const auto& front_id_map = sections_info.front().id_idx_map;
  for (const auto& lane_seg : lane_path.BeforeArclength(proj_range)) {
    if (front_id_map.contains(lane_seg.lane_id)) {
      return lane_path.AfterFirstOccurrenceOfLanePoint(mapping::LanePoint(
          lane_seg.lane_id, std::min(std::max(lane_seg.start_fraction,
                                              sections_info.start_fraction()),
                                     lane_seg.end_fraction)));
    }
  }
  return absl::OutOfRangeError(
      absl::StrCat("Prev target lane path: ", lane_path.DebugString(),
                   "has no overlap with the current route sections: ",
                   sections_info.route_sections()->DebugString(), "."));
}

PlannerStatus UpdatePlcRelevantStateByPlcStatus(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    std::optional<absl::Time> plc_prepare_start_time, absl::Time plan_time,
    QALCState init_alc_state, PlcInternalStatus plc_status,
    double planner_paddle_lane_change_max_prepare_time,
    mapping::LanePath* updated_preferred_lane_path, QALCState* new_alc_state,
    DriverAction::LaneChangeCommand* new_lc_cmd_state) {
  if (plc_status == PlcInternalStatus::BRANCH_NOT_FOUND ||
      plc_status == PlcInternalStatus::BRANCH_FAILED_INTERNAL) {
    if (plc_status == PlcInternalStatus::BRANCH_NOT_FOUND) {
      QLOG(ERROR) << "Teleop lane change failed: target branch not found.";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_REJECT_TARGET_NOT_FOUND);
    } else {
      QLOG(ERROR) << "Teleop lane change failed: branch internal failure.";
      if (*new_alc_state == ALC_ONGOING) {
        QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                   QRunEvent::PLC_FAIL_INTERNAL);
      } else {
        QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                   QRunEvent::PLC_FAIL_ALL_BRANCHES);
      }
    }

    updated_preferred_lane_path->Clear();
    *new_alc_state = ALC_STANDBY_ENABLE;
    *new_lc_cmd_state = DriverAction::LC_CMD_NONE;
  } else if (plc_status == PlcInternalStatus::SOLID_BOUNDARY ||
             plc_status == PlcInternalStatus::UNSAFE_OBJECT) {
    ReportPlcEventSignal(init_alc_state,
                         /*new_state=*/ALC_PREPARE, *new_lc_cmd_state,
                         plc_status);
    *new_alc_state = ALC_PREPARE;
  } else {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        *new_alc_state,
        UpdateAlcState(
            *new_alc_state == ALC_PREPARE ? ALC_ONGOING : *new_alc_state,
            drive_passage, {plan_start_point}, *new_lc_cmd_state,
            /*lane_change_target_point=*/std::nullopt,
            /*preview_duration=*/0.0, /*preview_length=*/0.0),
        PlannerStatusProto::SCHEDULER_UNAVAILABLE);
    if (*new_alc_state == ALC_STANDBY_ENABLE) {
      updated_preferred_lane_path->Clear();
      *new_lc_cmd_state = DriverAction::LC_CMD_NONE;
    }
    ReportPlcEventSignal(init_alc_state, *new_alc_state, *new_lc_cmd_state,
                         plc_status);
  }
  // Check if having been prepared for too long.
  if (*new_alc_state == ALC_PREPARE && plc_prepare_start_time.has_value() &&
      absl::ToDoubleSeconds(plan_time - *plc_prepare_start_time) >=
          planner_paddle_lane_change_max_prepare_time) {
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_FAIL_WAIT_TIMEOUT);
    updated_preferred_lane_path->Clear();
    *new_alc_state = ALC_STANDBY_ENABLE;
    *new_lc_cmd_state = DriverAction::LC_CMD_NONE;
  }
  return OkPlannerStatus();
}

RouteNaviInfo AlignRouteNaviInfoWithCurrentSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteSections& current_sections,
    const RouteNaviInfo& route_navi_info) {
  const int back_sec_size = route_navi_info.back_extend_sections.size();
  const int sec_size = back_sec_size + route_sections.size();

  auto aligned_navi_info = route_navi_info;
  const auto cur_front_sec_id = current_sections.section_id(0);
  for (int i = 0; i < sec_size; ++i) {
    const auto route_sec_id =
        i < back_sec_size ? route_navi_info.back_extend_sections.section_id(i)
                          : route_sections.section_id(i - back_sec_size);
    if (route_sec_id == cur_front_sec_id) {
      SMM_ASSIGN_SECTION_OR_BREAK(sec_info, psmm, route_sec_id);
      const double end_frac = current_sections.start_fraction();
      const double curr_offset = sec_info.average_length * end_frac;

      auto& route_lane_infos = aligned_navi_info.route_lane_info_map;
      auto& navi_section_infos = aligned_navi_info.navi_section_info_map;
      navi_section_infos[sec_info.id].length_before_intersection = std::max(
          0.0, navi_section_infos[sec_info.id].length_before_intersection -
                   curr_offset);
      for (const auto& lane_id : sec_info.lane_ids) {
        route_lane_infos[lane_id].max_driving_distance = std::max(
            0.0, route_lane_infos[lane_id].max_driving_distance - curr_offset);
        route_lane_infos[lane_id].max_reach_length = std::max(
            0.0, route_lane_infos[lane_id].max_reach_length - curr_offset);
        route_lane_infos[lane_id].recommend_reach_length =
            std::max(0.0, route_lane_infos[lane_id].recommend_reach_length -
                              curr_offset);
        route_lane_infos[lane_id].len_before_merge_lane = std::max(
            0.0, route_lane_infos[lane_id].len_before_merge_lane - curr_offset);
      }

      break;
    }
  }

  return aligned_navi_info;
}

PlannerStatus RunOptimizerAutoTuning(
    const std::vector<EstPlannerDebug>& est_debugs,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const PlannerSemanticMapManager& psmm,
    const Future<PlannerStatus>& future_expert_status,
    bool optimizer_data_cleaning, bool auto_tuning_mode,
    bool dumping_selector_features, bool filter_selector_intention,
    std::vector<EstPlannerOutput>* results,
    std::vector<PlannerStatus>* status_list,
    AutoTuningDataProto* auto_tuning_data,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    SelectorMLData* ml_data, ExpertPlannerOutput* expert_result) {
  if (optimizer_data_cleaning) {
    QCHECK((*results)[0].expert_auto_tuning_traj_proto.has_trajectory());
    if (auto_tuning_mode) {
      OptimizerCheckEachIntentionSameAsExpert(
          (*results)[0].expert_auto_tuning_traj_proto.trajectory(),
          *status_list, est_debugs, vehicle_geometry, psmm, results);
    } else {
      if (OptimizerHasNoIntentionSameAsExpert(
              (*results)[0].expert_auto_tuning_traj_proto.trajectory(),
              *status_list, *results, est_debugs, vehicle_geometry, psmm)) {
        return PlannerStatus(
            PlannerStatusProto::OPTIMIZER_DIFF_INTENTION_EXPERT,
            "Optimizer has no intention same as expert.");
      }
    }
  }
  *(auto_tuning_data->mutable_expert_auto_tuning_traj()) =
      std::move((*results)[0].expert_auto_tuning_traj_proto);
  for (auto& result : *results) {
    if (!result.scheduler_output.is_fallback) {
      *(auto_tuning_data->add_candidate_auto_tuning_trajs()) =
          std::move(result.candidate_auto_tuning_traj_proto);
    }
  }

  if (dumping_selector_features) {
    auto expert_status = future_expert_status.Get();
    if (!expert_status.ok()) return expert_status;

    if (filter_selector_intention) {
      ml_data->set_is_filtered(!SelectorIntentionSameAsExpert(
          expert_result->trajectory_points, *status_list, *results,
          vehicle_geometry, psmm));
    }
    AppendExpertToResultList(expert_status, std::move(*expert_result),
                             st_traj_mgr_list, status_list, results);
  }
  return OkPlannerStatus();
}

bool IsEnableCaptainNet(bool is_l4_mode, bool planner_enable_captain_net_j5,
                        bool planner_enable_captain_net_onnx_trt) {
  return is_l4_mode &&
         (planner_enable_captain_net_j5 || planner_enable_captain_net_onnx_trt);
}

bool IsEnableCaptainNetTrajectory(bool enable_captain_net,
                                  bool planner_use_ml_trajectory_end_to_end) {
  return enable_captain_net && planner_use_ml_trajectory_end_to_end;
}

bool IsFarAwayFromMergePoint(const RouteNaviInfo& route_navi_info,
                             const mapping::LanePath& lane_path, double v) {
  const auto& lane_navi_info_map = route_navi_info.route_lane_info_map;
  const auto* lane_navi_info_ptr =
      FindOrNull(lane_navi_info_map, lane_path.lane_ids().front());
  constexpr double kEnableTrafficGapFinderHeadwayTime = 5.0;   // s
  constexpr double kMinEnableTrafficGapFinderDistance = 30.0;  // m
  if (lane_navi_info_ptr != nullptr) {
    const double len_before_merge_lane =
        lane_navi_info_ptr->len_before_merge_lane;
    const double distance_threshold =
        std::max(kEnableTrafficGapFinderHeadwayTime * v,
                 kMinEnableTrafficGapFinderDistance);
    if (len_before_merge_lane < distance_threshold) {
      return false;
    }
  }
  return true;
}

bool IsEnableCalcRouteTargetInfo(bool is_going_force_lane_change,
                                 bool is_prepare_lane_change,
                                 bool is_far_away_from_merge) {
  return (is_going_force_lane_change || is_prepare_lane_change) &&
         is_far_away_from_merge;
}

PlannerStatus ProjectPlanStartPoint(
    const RouteSectionsInfo& route_sections_info_from_start,
    const mapping::LanePath& prev_target_lane_path,
    const mapping::LanePath& prev_lane_path_before_lc,
    const mapping::LanePath& prev_lc_prepare_lane_path,
    mapping::LanePath* prev_target_lane_path_from_start,
    mapping::LanePath* prev_lane_path_before_lc_from_start,
    mapping::LanePath* prev_lc_prepare_lane_path_from_start) {
  if (!route_sections_info_from_start.IsValid()) {
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         "Input RouteSectionsInfo is empty!");
  }

  // Guarantee that the lane path starts from the current section.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      *prev_target_lane_path_from_start,
      AlignLanePathWithRouteSections(
          route_sections_info_from_start, prev_target_lane_path,
          kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength),
      PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);

  if (!prev_lane_path_before_lc.IsEmpty()) {
    if (auto prev_lp_before_lc_or = AlignLanePathWithRouteSections(
            route_sections_info_from_start, prev_lane_path_before_lc,
            kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength);
        prev_lp_before_lc_or.ok()) {
      *prev_lane_path_before_lc_from_start =
          std::move(prev_lp_before_lc_or).value();
    }
  }

  if (!prev_lc_prepare_lane_path.IsEmpty()) {
    if (auto prev_lc_prepare_lane_path_or = AlignLanePathWithRouteSections(
            route_sections_info_from_start, prev_lc_prepare_lane_path,
            kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength);
        prev_lc_prepare_lane_path_or.ok()) {
      *prev_lc_prepare_lane_path_from_start =
          std::move(prev_lc_prepare_lane_path_or).value();
    }
  }

  return OkPlannerStatus();
}

void RunSingleEstPlannerOrFallback(
    int /*worker index*/ i, const RouteNaviInfo& aligned_route_navi_info,
    bool enable_captain_net, const MultiTasksCruisePlannerInput& input,
    const ExternalCommandStatus& ext_cmd_status,
    const RouteSectionsInfo& route_sections_info_from_start,
    const DrivingMapTopo& driving_map_topo,
    const StPathPlanStartPointInfo& path_start_point_info,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    LaneChangeStyle lane_change_style,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::optional<double>& cruising_speed_limit,
    std::vector<SchedulerOutput>* mutable_multi_tasks,
    std::vector<ml::captain_net::CaptainNetOutput>* mutable_captain_net_results,
    std::vector<PlannerStatus>* mutable_status_list,
    std::vector<EstPlannerOutput>* results,
    std::vector<EstPlannerDebug>* est_debugs,
    std::vector<vis::vantage::ChartDataBundleProto>* chart_data_bundles,
    PlannerStatus* fallback_status, FallbackPlannerOutput* fallback_result,
    EstPlannerDebug* fallback_debug,
    vis::vantage::ChartDataBundleProto* fallback_chart_data,
    ThreadPool* thread_pool) {
  std::vector<SchedulerOutput>& multi_tasks = *mutable_multi_tasks;
  std::vector<ml::captain_net::CaptainNetOutput>& captain_net_results =
      *mutable_captain_net_results;
  std::vector<PlannerStatus>& status_list = *mutable_status_list;

  if (i < multi_tasks.size()) {
    ml::captain_net::CaptainNetOutput captain_net_output;
    if (enable_captain_net) {
      // ---------------------------------------------------------
      // ------ Use CaptainNet Trajs as Reference ----------------
      // ---------------------------------------------------------
      if (captain_net_results[i].validation.IsValidAsRef()) {
        captain_net_output = std::move(captain_net_results[i]);
      }
    }
    if (aligned_route_navi_info.in_highway && multi_tasks[i].blocked_abreast) {
      (*results)[i].scheduler_output = std::move(multi_tasks[i]);
      status_list[i] =
          PlannerStatus(PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
                        "Target lane side blocked by obstacle abreast.");
      return;
    }
    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &driving_map_topo,
            .semantic_map_manager =
                input.planner_semantic_map_manager->semantic_map_manager(),
            .planner_semantic_map_manager =
                input.planner_semantic_map_manager.get(),
            .plan_id = i + 1,
            .vehicle_params = input.vehicle_params,
            .is_run_model_l4 = IsRunModeL4(),
            .parking_brake_release_time = input.parking_brake_release_time,
            .decider_state = input.decider_state,
            .initializer_state = input.initializer_state,
            .trajectory_optimizer_state_proto =
                input.selected_trajectory_optimizer_state_proto,
            .st_planner_object_trajectories =
                input.st_planner_object_trajectories,
            .prev_collision_warning_request =
                input.prev_collision_warning_request,
            .obj_mgr = input.object_manager.get(),
            .start_point_info = input.start_point_info,
            .st_path_start_point_info = &path_start_point_info,
            .tl_info_map = input.tl_info_map,
            .smooth_result_map = input.smooth_result_map,
            .stalled_objects = input.stalled_objects,
            .scene_reasoning = input.scene_reasoning,
            .prev_target_lane_path_from_start =
                &prev_target_lane_path_from_start,
            .time_aligned_prev_traj = input.time_aligned_prev_traj,
            .lane_change_style = lane_change_style,
            .enable_pull_over = ext_cmd_status.enable_pull_over,
            .enable_traffic_light_stopping =
                ext_cmd_status.enable_traffic_light_stopping,
            .brake_to_stop = ext_cmd_status.brake_to_stop,
            .enable_force_stop = input.is_standwait,
            .st_traj_mgr = &st_traj_mgr_list[i],
            .log_av_trajectory = input.log_av_trajectory,
            .captain_net_output = &captain_net_output,
            .planner_av_context = input.planner_av_context,
            .real_objects = input.real_objects.get(),
            .virtual_objects = input.virtual_objects.get(),
            .planner_model_pool = input.planner_model_pool,
            .decision_constraint_config =
                &input.planner_params->decision_constraint_config(),
            .initializer_params = &input.planner_params->initializer_params(),
            .trajectory_optimizer_params =
                &input.planner_params->trajectory_optimizer_params(),
            .speed_finder_params = &input.planner_params->speed_finder_params(),
            .motion_constraint_params =
                &input.planner_params->motion_constraint_params(),
            .planner_functions_params =
                &input.planner_params->planner_functions_params(),
            .vehicle_models_params =
                &input.planner_params->vehicle_models_params(),
            .speed_finder_lc_radical_params =
                &input.planner_params->speed_finder_lc_radical_params(),
            .speed_finder_lc_conservative_params =
                &input.planner_params->speed_finder_lc_conservative_params(),
            .trajectory_optimizer_lc_radical_params =
                &input.planner_params->trajectory_optimizer_lc_radical_params(),
            .trajectory_optimizer_lc_normal_params =
                &input.planner_params->trajectory_optimizer_lc_normal_params(),
            .trajectory_optimizer_lc_conservative_params =
                &input.planner_params
                     ->trajectory_optimizer_lc_conservative_params(),
            .spacetime_planner_object_trajectories_params =
                &input.planner_params
                     ->spacetime_planner_object_trajectories_params()},
        std::move(multi_tasks[i]), &(*results)[i], &(*est_debugs)[i],
        &(*chart_data_bundles)[i],
        FLAGS_planner_allow_multi_threads_in_est ? thread_pool : nullptr);
  } else {
    FallbackPlannerInput fallback_input{
        .psmm = input.planner_semantic_map_manager.get(),
        .start_point_info = input.start_point_info,
        .time_aligned_prev_trajectory = input.time_aligned_prev_traj,
        .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
        .prev_length_along_route = input.prev_length_along_route,
        .prev_max_reach_length = input.prev_max_reach_length,
        .station_anchor = input.station_anchor,
        .prev_smooth_state = input.prev_smooth_state,
        .prev_lane_path_before_lc = &prev_lane_path_before_lc_from_start,
        .route_sections_info_from_start = &route_sections_info_from_start,
        .obj_mgr = input.object_manager.get(),
        .st_traj_mgr = input.st_traj_mgr.get(),
        .stalled_objects = input.stalled_objects,
        .scene_reasoning = input.scene_reasoning,
        .prev_lc_state = input.lane_change_state,
        .traffic_light_states = input.traffic_light_states,
        .pre_decider_state = input.decider_state,
        .tl_info_map = input.tl_info_map,
        .smooth_result_map = input.smooth_result_map,
        .parking_brake_release_time = input.parking_brake_release_time,
        .teleop_enable_traffic_light_stop =
            ext_cmd_status.enable_traffic_light_stopping,
        .enable_pull_over = ext_cmd_status.enable_pull_over,
        .brake_to_stop = ext_cmd_status.brake_to_stop,
        .prev_route_sections = input.prev_route_sections,
        .cruising_speed_limit = cruising_speed_limit,
        .enable_force_stop = input.is_standwait};

    *fallback_status =
        RunFallbackPlanner(fallback_input, *input.vehicle_params,
                           input.planner_params->motion_constraint_params(),
                           input.planner_params->decision_constraint_config(),
                           input.planner_params->fallback_planner_params(),
                           fallback_result, fallback_debug, fallback_chart_data,
                           /*thread_pool=*/nullptr);
  }
}

PlannerStatus PostSelectTrajectory(
    const MultiTasksCruisePlannerInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const ExternalCommandStatus& ext_cmd_status, const Box2d& ego_box,
    QALCState init_alc_state,
    const RouteSectionsInfo& route_sections_info_from_start,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const RouteNaviInfo& aligned_route_navi_info,
    LaneChangeStyle lane_change_style,
    mapping::LanePath* updated_preferred_lane_path, QALCState* new_alc_state,
    DriverAction::LaneChangeCommand* new_lc_cmd_state,
    std::vector<PlannerStatus>* mutable_status_list,
    std::vector<SpacetimeTrajectoryManager>* mutable_st_traj_mgr_list,
    std::vector<EstPlannerOutput>* mutable_results,
    std::vector<EstPlannerDebug>* mutable_est_debugs,
    std::vector<vis::vantage::ChartDataBundleProto>* mutable_chart_data_bundles,
    PathBoundedEstPlannerOutput* output) {
  std::vector<PlannerStatus>& status_list = *mutable_status_list;
  std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list =
      *mutable_st_traj_mgr_list;
  std::vector<EstPlannerOutput>& results = *mutable_results;
  std::vector<EstPlannerDebug>& est_debugs = *mutable_est_debugs;
  std::vector<vis::vantage::ChartDataBundleProto>& chart_data_bundles =
      *mutable_chart_data_bundles;

  const auto& plan_start_point = input.start_point_info->start_point;

  // 0. Pre-process status list to filter out est planner results that
  // should not be considered by selector.
  if (!input.preferred_lane_path->IsEmpty() ||
      !updated_preferred_lane_path->IsEmpty()) {
    output->plc_result = PlcInternalResult();

    if (!updated_preferred_lane_path->IsEmpty()) {
      PreFilterByPreferred(*input.planner_semantic_map_manager, *new_alc_state,
                           *updated_preferred_lane_path, results,
                           vehicle_geometry, &output->plc_result.value(),
                           mutable_status_list);
    }
  }

  PreFilterRedundant(results, mutable_status_list);

  // 1. Run trajectory selector.
  SelectorParamsProto tuned_selector_params;
  if (FLAGS_use_tuned_selector_params) {
    tuned_selector_params =
        LoadSelectorParamsFromFile(FLAGS_selector_params_file_address);
  }

  // Get noa need lane change confirmation from hmi.
  const bool noa_need_lane_change_confirmation =
      ext_cmd_status.noa_need_lane_change_confirmation.has_value()
          ? *ext_cmd_status.noa_need_lane_change_confirmation
          : false;
  const bool planner_enable_cross_solid_boundary =
      IsRunModeL4() ? FLAGS_planner_enable_cross_solid_boundary : false;
  SelectorFlags selector_flags{
      .planner_begin_lane_change_frame = FLAGS_planner_begin_lane_change_frame,
      .planner_begin_signal_frame = FLAGS_planner_begin_signal_frame,
      .planner_enable_lane_change_in_intersection =
          FLAGS_planner_enable_lane_change_in_intersection,
      .planner_enable_cross_solid_boundary =
          planner_enable_cross_solid_boundary,
      .planner_lane_change_style = lane_change_style,
      .planner_need_to_lane_change_confirmation =
          noa_need_lane_change_confirmation,
      .planner_is_bus_model =
          input.planner_params->vehicle_models_params().is_vehicle_bus_model(),
      .planner_is_l4_mode = IsRunModeL4(),
      .planner_enable_obstacle_lane_change =
          FLAGS_planner_enable_obstacle_lane_change,
      .planner_enable_lc_request_in_tricky_scenario =
          FLAGS_planner_enable_lc_request_in_tricky_scenario,
      .planner_begin_radical_lane_change_frame =
          FLAGS_planner_begin_radical_lane_change_frame,
      .planner_allow_lc_time_after_activate_selector =
          FLAGS_planner_allow_lc_time_after_activate_selector,
      .planner_max_allow_lc_time_before_give_up =
          FLAGS_planner_max_allow_lc_time_before_give_up,
      .planner_allow_lc_time_after_give_up_lc =
          FLAGS_planner_allow_lc_time_after_give_up_lc,
      .planner_lc_begin_request_frame_in_tricky_scenario =
          FLAGS_planner_lc_begin_request_frame_in_tricky_scenario,
      .planner_enable_prefilter_for_selector =
          FLAGS_planner_enable_prefilter_for_selector,
      .planner_allow_opposite_lc_time_after_paddle_lc =
          FLAGS_planner_allow_opposite_lc_time_after_paddle_lc};
  SelectorInput selector_input{
      .psmm = input.planner_semantic_map_manager.get(),
      .sections_info = &route_sections_info_from_start,
      .prev_lane_path_from_current = &prev_target_lane_path_from_start,
      .prev_traj = input.time_aligned_prev_traj,
      .motion_constraints = &input.planner_params->motion_constraint_params(),
      .vehicle_geom = &vehicle_geometry,
      .plan_start_point = &plan_start_point,
      .stalled_objects = input.stalled_objects,
      .route_navi_info = &aligned_route_navi_info,
      .avoid_lanes = &input.rm_output->avoid_lanes,
      .plan_time = input.start_point_info->plan_time,
      .alc_confirmation = input.alc_confirmation,
      .preferred_lane_path = updated_preferred_lane_path,
      .selector_state = input.selector_state,
      .selector_flags = &selector_flags,
      .config = FLAGS_use_tuned_selector_params
                    ? &tuned_selector_params
                    : &input.planner_params->selector_params()};

  auto selector_output_or =
      SelectTrajectory(selector_input, status_list, st_traj_mgr_list, results,
                       &output->selector_debug, &output->selector_state);

  if (selector_output_or.ok()) {
    const int selected_idx = selector_output_or->selected_idx;
    std::swap(results[0], results[selected_idx]);
    std::swap(status_list[0], status_list[selected_idx]);
    std::swap(est_debugs[0], est_debugs[selected_idx]);
    std::swap(chart_data_bundles[0], chart_data_bundles[selected_idx]);
    std::swap(st_traj_mgr_list[0], st_traj_mgr_list[selected_idx]);

    // Stitch path points and convert to global path points.
    const auto& start_point_info = *input.start_point_info;
    StitchStPathTrajectoryWithPastTrajectory(
        *input.previous_trajectory, start_point_info.start_index_on_prev_traj,
        results[0].st_path_points, &output->st_path_points_including_past);
    output->st_path_points_global_including_past =
        output->st_path_points_including_past;
    ConvertSmoothPathToGlobalCoordinates(
        *input.coordinate_converter,
        &output->st_path_points_global_including_past);

    const auto& scheduler_output = results[0].scheduler_output;

    if (scheduler_output.request_help_lane_change_by_route) {
      QEVENT_EVERY_N_SECONDS("chengyang", "request_help_lane_change_by_route",
                             /*seconds=*/5.0, [](QEvent*) {});
    }

    if (input.lane_change_state->stage() == LaneChangeStage::LCS_NONE &&
        scheduler_output.lane_change_state.stage() ==
            LaneChangeStage::LCS_EXECUTING) {
      QEVENT_EVERY_N_SECONDS("zixuan", "lane_change_initiated",
                             /*seconds=*/5.0, [](QEvent*) {});
    }
    // -----------------------------------------------------------------
    // ---------------------- Update PLC result ------------------------
    // -----------------------------------------------------------------
    if (output->plc_result.has_value()) {
      auto update_status = UpdatePlcRelevantStateByPlcStatus(
          scheduler_output.drive_passage, plan_start_point,
          ext_cmd_status.plc_prepare_start_time,
          input.start_point_info->plan_time, init_alc_state,
          output->plc_result->status,
          FLAGS_planner_paddle_lane_change_max_prepare_time,
          updated_preferred_lane_path, new_alc_state, new_lc_cmd_state);
      if (!update_status.ok()) {
        return update_status;
      }
      output->alc_state = *new_alc_state;
      output->plc_result->preferred_lane_path =
          std::move(*updated_preferred_lane_path);
      output->plc_result->lane_change_command = *new_lc_cmd_state;
    }

    if (IsEnableCalcRouteTargetInfo(
            selector_output_or->is_going_force_route_change_left.has_value(),
            output->alc_state == ALC_PREPARE,
            IsFarAwayFromMergePoint(aligned_route_navi_info,
                                    prev_target_lane_path_from_start,
                                    plan_start_point.v()))) {
      output->route_target_info = FindRouteTargetInfo(
          *input.planner_semantic_map_manager,
          output->plc_result->preferred_lane_path, ego_box, results,
          st_traj_mgr_list,
          selector_output_or->is_going_force_route_change_left,
          selector_output_or->merge_point);
    }

    // Hmi
    output->nudge_object_info = results[0].nudge_object_info;
  }

  if (!selector_output_or.ok()) {
    return PlannerStatus(PlannerStatusProto::SELECTOR_FAILED,
                         selector_output_or.status().message());
  }
  output->selector_output = std::move(selector_output_or).value();

  return OkPlannerStatus();
}

}  // namespace

// NOLINTNEXTLINE
PlannerStatus RunMultiTasksCruisePlanner(
    const MultiTasksCruisePlannerInput& input,
    PathBoundedEstPlannerOutput* output, ThreadPool* thread_pool) {
  SCOPED_QTRACE("MultiTasksEstPlan");

  const auto& plan_start_point = input.start_point_info->start_point;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto& vehicle_geometry =
      input.vehicle_params->vehicle_geometry_params();
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geometry);

  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& ext_cmd_status = *input.ext_cmd_status;
  const auto& cruising_speed_limit = ext_cmd_status.noa_cruising_speed_limit;

  ScopedMultiTimer timer("multi_tasks_est");

  // ----------------------------------------------------------
  // ---------------- Project plan_start_point ----------------
  // ----------------------------------------------------------
  const RouteSectionsInfo route_sections_info_from_start(
      psmm, input.route_sections_from_start);
  mapping::LanePath prev_target_lane_path_from_start;
  mapping::LanePath prev_lane_path_before_lc_from_start;
  mapping::LanePath prev_lc_prepare_lane_path_from_start;
  if (auto status = ProjectPlanStartPoint(
          route_sections_info_from_start, *input.prev_target_lane_path,
          *input.prev_lane_path_before_lc,
          input.selector_state->lc_prepare_stage_lane_path.has_value()
              ? *input.selector_state->lc_prepare_stage_lane_path
              : mapping::LanePath(),
          &prev_target_lane_path_from_start,
          &prev_lane_path_before_lc_from_start,
          &prev_lc_prepare_lane_path_from_start);
      !status.ok()) {
    return status;
  }
  timer.Mark("project_start_point");

  const auto aligned_route_navi_info = AlignRouteNaviInfoWithCurrentSections(
      psmm, input.rm_output->route_sections_from_current,
      *input.route_sections_from_start, input.rm_output->route_navi_info);

  // ----------------------------------------------------------
  // ------------------- Build driving map --------------------
  // ----------------------------------------------------------
  // Build driving map by route sections.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto route_section_in_horizon,
      ClampRouteSectionsBeforeArcLength(
          psmm, *input.route_sections_from_start,
          input.route_sections_from_start->planning_horizon(psmm)),
      PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto driving_map_topo,
      BuildDrivingMapByRouteOnOfflineMap(psmm, route_section_in_horizon),
      PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);

  // ------------------------------------------------------------------
  // ------------------ Preferred lanes from teleop -------------------
  // ------------------------------------------------------------------
  const auto init_alc_state = ext_cmd_status.alc_state;
  auto updated_preferred_lane_path = *input.preferred_lane_path;
  auto new_alc_state = ext_cmd_status.alc_state;
  auto new_lc_cmd_state = ext_cmd_status.lane_change_command;

  UpdatePreferredLanePath(psmm, route_section_in_horizon,
                          aligned_route_navi_info, &updated_preferred_lane_path,
                          &new_alc_state, &new_lc_cmd_state);

  // Ignore driver action when wait alc response.
  const auto& alc_request = input.selector_state->selector_lane_change_request;
  const bool alc_request_waiting =
      alc_request.has_lane_change_type() &&
      alc_request.lane_change_type() != LaneChangeType::NO_CHANGE;
  const auto input_new_lc_cmd =
      alc_request_waiting ? DriverAction::LC_CMD_NONE : input.new_lc_command;
  if (input_new_lc_cmd != input.new_lc_command) {
    QLOG(INFO) << "Ignore driver action: "
               << DriverAction_LaneChangeCommand_Name(input.new_lc_command)
               << ", when alc request waits.";
  }
  HandleNewTeleopCommand(
      psmm, route_section_in_horizon, aligned_route_navi_info,
      input.ego_nearest_lane_id, input_new_lc_cmd,
      prev_target_lane_path_from_start, *input.prev_lane_path_before_lc,
      *input.lane_change_state, input.selector_state->pre_turn_signal,
      &updated_preferred_lane_path, &new_alc_state, &new_lc_cmd_state);

  // Ignore alc confirmation when there is no alc request.
  const std::optional<bool> alc_left =
      alc_request_waiting ? std::optional<bool>(alc_request.lc_left())
                          : std::nullopt;
  HandleAlcUserResponse(psmm, route_section_in_horizon, aligned_route_navi_info,
                        prev_target_lane_path_from_start,
                        *input.lane_change_state, input.alc_confirmation,
                        alc_left, &updated_preferred_lane_path, &new_alc_state,
                        &new_lc_cmd_state);
  const auto lane_change_style = updated_preferred_lane_path.IsEmpty()
                                     ? ext_cmd_status.lane_change_style
                                     : LC_STYLE_RADICAL;

  // --------------------------------------------------------------
  // ----------------- Choose candidates on lane graph ------------
  // --------------------------------------------------------------
  const auto lane_graph =
      BuildLaneGraph(psmm, route_sections_info_from_start,
                     *input.object_manager, *input.stalled_objects,
                     input.rm_output->avoid_lanes, aligned_route_navi_info);
  if (FLAGS_planner_send_lane_graph_to_canvas) {
    SendLaneGraphToCanvas(lane_graph, psmm, route_sections_info_from_start,
                          "planner/lane_graph");
  }

  // Collect candidate lane paths from each start lane. If one or more
  // diverging point exists starting from some lane, the first one would be
  // considered, thus two candidates would be produced.

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto lp_infos,
      FindBestLanePathsFromStart(psmm, route_sections_info_from_start,
                                 aligned_route_navi_info, lane_graph,
                                 thread_pool),
      PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE);
  if (lp_infos.empty()) {
    // In case all lane paths are blocked by stalled objects.
    return PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                         "No viable candidate route found to destination.");
  }

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, route_sections_info_from_start, aligned_route_navi_info,
      prev_target_lane_path_from_start, plan_start_point,
      updated_preferred_lane_path, prev_lc_prepare_lane_path_from_start,
      &lp_infos);
  if (target_lp_infos.empty()) {
    // In case all lane paths are blocked by stalled objects.
    return PlannerStatus(PlannerStatusProto::TARGET_LANE_CANDIDATES_UNAVAILABLE,
                         "No valid target lane path from the current section.");
  }
  timer.Mark("filter_target_lane_paths");

  // ------------------------------------------------------------------
  // --------------------- Optional expert planner --------------------
  // ------------------------------------------------------------------
  ExpertPlannerOutput expert_result;
  Future<PlannerStatus> future_expert_status;
  if (FLAGS_dumping_selector_features) {
    future_expert_status = ScheduleFuture(thread_pool, [&]() {
      ExpertPlannerInput expert_planner_input{
          .psmm = &psmm,
          .start_point_info = input.start_point_info,
          .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
          .prev_lane_path_before_lc_from_start =
              &prev_lane_path_before_lc_from_start,
          .prev_lc_state = input.lane_change_state,
          .station_anchor = input.station_anchor,
          .sections_info_from_start = &route_sections_info_from_start,
          .obj_mgr = input.object_manager.get(),
          .st_traj_mgr = input.st_traj_mgr.get(),
          .stalled_objects = input.stalled_objects,
          .scene_reasoning = input.scene_reasoning,
          .traffic_light_states = input.traffic_light_states,
          .pre_decider_state = input.decider_state,
          .tl_info_map = input.tl_info_map,
          .parking_brake_release_time = input.parking_brake_release_time,
          .teleop_enable_traffic_light_stop =
              ext_cmd_status.enable_traffic_light_stopping,
          .enable_pull_over = ext_cmd_status.enable_pull_over,
          .brake_to_stop = ext_cmd_status.brake_to_stop,
          .lp_infos = &lp_infos,
          .planner_params = input.planner_params,
          .vehicle_params = input.vehicle_params,
          .smooth_result_map = input.smooth_result_map,
          .route_navi_info = &aligned_route_navi_info,
          .prev_smooth_state = input.prev_smooth_state,
          .prev_route_sections = input.prev_route_sections,
          .log_av_trajectory = input.log_av_trajectory,
          .cruising_speed_limit = cruising_speed_limit,
          .enable_force_stop = input.is_standwait,
          .autonomy_state = input.autonomy_state,
      };

      return RunExpertPlanner(expert_planner_input, &expert_result,
                              thread_pool);
    });
  }

  // --------------------------------------------------------------
  // ----------------- Run Scheduler ------------------------------
  // --------------------------------------------------------------
  MultiTasksSchedulerInput scheduler_input{
      .psmm = &psmm,
      .online_semantic_map = input.online_semantic_map.get(),
      .vehicle_geom = &vehicle_geometry,
      .st_traj_mgr = input.st_traj_mgr.get(),
      .lane_path_infos = &lp_infos,
      .sections_info_from_current = &route_sections_info_from_start,
      .tl_info_map = input.tl_info_map,
      .prev_smooth_state = input.prev_smooth_state,
      .plan_start_point = &plan_start_point,
      .station_anchor = input.station_anchor,
      .start_route_s = 0.0,
      .smooth_result_map = input.smooth_result_map,
      .prev_route_sections = input.prev_route_sections,
      .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
      .prev_lane_path_before_lc_from_start =
          &prev_lane_path_before_lc_from_start,
      .prev_lc_state = input.lane_change_state,
      .route_navi_info = &aligned_route_navi_info,
      .cruising_speed_limit = cruising_speed_limit,
      .planner_is_l4_mode = IsRunModeL4(),
      .autonomy_state = input.autonomy_state,
  };

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto multi_tasks,
      ScheduleMultiplePlanTasks(scheduler_input, target_lp_infos, thread_pool),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  timer.Mark("scheduler");

  // TODO(huaiyuan): Get last_st_path_plan_start_time from elsewhere.
  std::optional<absl::Time> last_st_path_plan_start_time;
  if (input.selected_trajectory_optimizer_state_proto != nullptr) {
    last_st_path_plan_start_time =
        qcraft::FromProto(input.selected_trajectory_optimizer_state_proto
                              ->last_plan_start_time());
  }

  // Find st path planner path_plan_start_point with given look ahead
  // duration.
  const auto path_start_point_info = GetStPathPlanStartPointInfo(
      input.min_path_look_ahead_duration, *input.start_point_info,
      *input.previous_trajectory,
      QCHECK_NOTNULL(input.planner_params)
          ->trajectory_optimizer_params()
          .trajectory_time_step(),
      last_st_path_plan_start_time);

  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_list;
  // +2 for possible fallback and expert results.
  st_traj_mgr_list.reserve(multi_tasks.size() + 2);
  st_traj_mgr_list.resize(multi_tasks.size());
  ParallelFor(0, multi_tasks.size(), thread_pool,
              [&multi_tasks, &obj_mgr = *input.object_manager,
               on_vision_map = psmm.IsOnVisionMap(), &st_traj_mgr_list,
               &thread_pool](int i) {
                SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
                    .passage = &multi_tasks[i].drive_passage,
                    .sl_boundary = &multi_tasks[i].sl_boundary,
                    .obj_mgr = &obj_mgr,
                    .on_vision_map = on_vision_map};
                st_traj_mgr_list[i] = BuildSpacetimeTrajectoryManager(
                    st_mgr_builder_input, thread_pool);
              });

  // Select highlight vehicle branch.
  const std::optional<int> highlight_vehicle_branch =
      SelectHighlightFrontVehicleBranchIndex(
          multi_tasks, ego_box, plan_start_point.path_point().theta());

  // --------------------------------------------------------------
  // -------- Run CaptainNet Inference If Configured---------------
  // --------------------------------------------------------------
  // Run CaptainNet for multiple scheduler outputs if configured.
  std::vector<ml::captain_net::CaptainNetOutput> captain_net_results(
      multi_tasks.size());
  std::vector<PlannerStatus> status_list(multi_tasks.size());
  std::vector<EstPlannerOutput> results(multi_tasks.size());
  std::vector<EstPlannerDebug> est_debugs(multi_tasks.size());
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(
      multi_tasks.size());

  PlannerStatus fallback_status(PlannerStatusProto::UNINITIALIZED,
                                "Initial status.");
  FallbackPlannerOutput fallback_result;
  EstPlannerDebug fallback_debug;
  vis::vantage::ChartDataBundleProto fallback_chart_data;

  const bool enable_captain_net =
      IsEnableCaptainNet(IsRunModeL4(), FLAGS_planner_enable_captain_net_j5,
                         FLAGS_planner_enable_captain_net_onnx_trt);
  if (enable_captain_net) {
    const auto start_time = absl::Now();
    auto status = GenerateCaptainNetTrajectory(
        psmm, multi_tasks, *input.context_feature, *input.start_point_info,
        vehicle_geometry, input.vehicle_params->vehicle_drive_params(),
        st_traj_mgr_list, input.planner_model_pool, &captain_net_results,
        &status_list, &est_debugs, thread_pool);

    const auto duration = absl::ToDoubleMilliseconds(absl::Now() - start_time);
    VLOG(1) << "Time(ms) spent for GenerateCaptainNetTrajectory: " << duration;
    QEVENT_EVERY_N_SECONDS(
        "jinyun", "GeneratingCaptainNetTrajectoryTime", 5.0,
        [&](QEvent* qevent) { qevent->AddField("time(ms)", duration); });

    if (!status.ok()) return status;
  }

  const bool enable_captain_net_traj = IsEnableCaptainNetTrajectory(
      enable_captain_net, FLAGS_planner_use_ml_trajectory_end_to_end);
  if (enable_captain_net_traj) {
    // -----------------------------------------------------------------
    // --------------- Use CaptainNet Trajs E2E ------------------------
    // -----------------------------------------------------------------
    for (int i = 0; i < multi_tasks.size(); ++i) {
      constexpr double kRequiredMinPathLength = 20.0;
      constexpr double kCurvatureRelaxFactor = 1.05;
      const double max_curvature = ComputeRelaxedCenterMaxCurvature(
          vehicle_geometry, input.vehicle_params->vehicle_drive_params());
      RETURN_PLANNER_STATUS_OR_ASSIGN(
          auto path_extension_output,
          ExtendPathAndDeleteUnreasonablePart(
              captain_net_results[i].traj_points, kRequiredMinPathLength,
              kCurvatureRelaxFactor * max_curvature),
          PlannerStatusProto::PATH_EXTENSION_FAILED);
      results[i].path = DiscretizedPath::CreateResampledPath(
          std::move(path_extension_output), kPathSampleInterval);
      results[i].traj_points = std::move(captain_net_results[i].traj_points);
      results[i].scheduler_output = multi_tasks[i];
      *est_debugs[i].speed_finder_debug.mutable_trajectory() = {
          results[i].traj_points.begin(), results[i].traj_points.end()};
      est_debugs[i].speed_finder_debug.set_trajectory_start_timestamp(
          ToUnixDoubleSeconds(input.start_point_info->plan_time));
    }
  } else {
    // ---------------------------------------
    // ------ Run Est Planner ----------------
    // ---------------------------------------
    // +1 run fallback
    ParallelFor(0, multi_tasks.size() + 1, thread_pool, [&](int i) {
      RunSingleEstPlannerOrFallback(
          i, aligned_route_navi_info, enable_captain_net, input, ext_cmd_status,
          route_sections_info_from_start, driving_map_topo,
          path_start_point_info, prev_target_lane_path_from_start,
          prev_lane_path_before_lc_from_start, lane_change_style,
          st_traj_mgr_list, cruising_speed_limit, &multi_tasks,
          &captain_net_results, &status_list, &results, &est_debugs,
          &chart_data_bundles, &fallback_status, &fallback_result,
          &fallback_debug, &fallback_chart_data, thread_pool);
    });
  }

  // All est planners use the same alerted vehicle.
  FillSameAlertedFrontVehicle(highlight_vehicle_branch, &results);

  for (int i = 0; i < results.size(); ++i) {
    if (!status_list[i].ok()) {
      QLOG(WARNING) << "Failed task " << i << ": " << status_list[i].message();
    }
  }
  timer.Mark("multi_trajectories");

  // Collect speed-considered object ids by all est planners and fallback
  // planner.
  ObjectsPredictionProto speed_considered_objects_prediction =
      CollectSpeedConsideredObjectsPrediction(
          *input.object_manager,
          CollectSpeedConsiderObjectsPartialStTrajectory(
              results, fallback_result.considered_st_objects),
          FLAGS_planner_export_all_prediction_to_speed_considered);

  FillFallbackPlannerResult(
      fallback_status, updated_preferred_lane_path, std::move(fallback_result),
      std::move(fallback_debug), std::move(fallback_chart_data),
      &st_traj_mgr_list, &status_list, &results, &est_debugs,
      &chart_data_bundles);

  // ----------------------------------------------------------
  // --------------- Optimizer Auto Tuning --------------------
  // ----------------------------------------------------------
  auto optimizer_auto_tuning_status = RunOptimizerAutoTuning(
      est_debugs, vehicle_geometry, psmm, future_expert_status,
      FLAGS_optimizer_data_cleaning, FLAGS_optimizer_data_cleaning,
      FLAGS_dumping_selector_features, FLAGS_filter_selector_intention,
      &results, &status_list, &output->auto_tuning_data, &st_traj_mgr_list,
      output->selector_debug.mutable_ml_data(), &expert_result);
  if (!optimizer_auto_tuning_status.ok()) {
    return optimizer_auto_tuning_status;
  }

  // ---------------------------------------
  // ------ Trajectory Selection ----------------
  // ---------------------------------------
  auto selector_status = PostSelectTrajectory(
      input, vehicle_geometry, ext_cmd_status, ego_box, init_alc_state,
      route_sections_info_from_start, prev_target_lane_path_from_start,
      aligned_route_navi_info, lane_change_style, &updated_preferred_lane_path,
      &new_alc_state, &new_lc_cmd_state, &status_list, &st_traj_mgr_list,
      &results, &est_debugs, &chart_data_bundles, output);

  output->est_status_list = std::move(status_list);
  output->est_planner_output_list = std::move(results);
  output->est_planner_debug_list = std::move(est_debugs);
  output->chart_data_list = std::move(chart_data_bundles);
  output->speed_considered_objects_prediction =
      std::move(speed_considered_objects_prediction);
  output->path_start_relative_index =
      path_start_point_info.relative_index_from_plan_start_point;
  output->low_freq_psmm = input.planner_semantic_map_manager;
  output->driving_map_topo =
      std::make_shared<const DrivingMapTopo>(std::move(driving_map_topo));

  return selector_status;
}  // NOLINT

}  // namespace qcraft::planner
