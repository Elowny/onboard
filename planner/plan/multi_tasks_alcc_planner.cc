#include "onboard/planner/plan/multi_tasks_alcc_planner.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <type_traits>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "common/proto/qalc.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/alcc_scheduler.h"
#include "onboard/planner/assist/alcc_selector.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/lcc_map_builder.h"
#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/assist/vision_lane_path_filter.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/hmi_util.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_util.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

bool NeedOriginLanePath(QALCState prev_alc_state) {
  switch (prev_alc_state) {
    case ALC_STANDBY_ENABLE:
    case ALC_PREPARE:
    case ALC_RETURN_COMPLETED:
    case ALC_RETURNING:
    case ALC_OFF:
    case ALC_STANDBY:
      return true;
    case ALC_ONGOING:
    case ALC_COMPLETED:
    case ALC_CROSSING_LANE:
      return false;
  }
}

mapping::LanePath FindFinalTargetLanePath(
    const std::array<mapping::LanePath, 3>& candidate_lanes,
    LaneChangeDirection lc_direction) {
  switch (lc_direction) {
    case LaneChangeDirection::LCD_NONE:
      return candidate_lanes[1];
    case LaneChangeDirection::LCD_LEFT:
      return candidate_lanes[0];
    case LaneChangeDirection::LCD_RIGHT:
      return candidate_lanes[2];
  }
}

int FindPreferredEstIndex(
    const std::vector<EstPlannerOutput>& est_results,
    const std::array<mapping::LanePath, 3>& candidate_lanes,
    DriverAction::LaneChangeCommand lc_cmd) {
  constexpr int kInvalidEstIdx = -1;

  const mapping::LanePath* preferred_lane_path = nullptr;
  switch (lc_cmd) {
    case DriverAction::LC_CMD_NONE:
    case DriverAction::LC_CMD_STRAIGHT:  // Should not receive this here.
      return kInvalidEstIdx;
    case DriverAction::LC_CMD_CANCEL:
      preferred_lane_path = &candidate_lanes[1];
      break;
    case DriverAction::LC_CMD_RIGHT:
      preferred_lane_path = &candidate_lanes[2];
      break;
    case DriverAction::LC_CMD_LEFT:
      preferred_lane_path = &candidate_lanes[0];
      break;
  }

  if (preferred_lane_path == nullptr || preferred_lane_path->IsEmpty()) {
    return kInvalidEstIdx;
  }

  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path->lane_ids().begin(),
      preferred_lane_path->lane_ids().end());
  for (int i = 0; i < est_results.size(); ++i) {
    if (preferred_lanes.contains(est_results[i]
                                     .scheduler_output.drive_passage.lane_path()
                                     .front()
                                     .lane_id())) {
      return i;
    }
  }

  return kInvalidEstIdx;
}

PlcInternalResult DeterminePlcResultFromEstResults(
    const std::vector<EstPlannerOutput>& est_results,
    const std::vector<PlannerStatus>& status_list, int preferred_idx) {
  PlcInternalResult plc_result;
  if (preferred_idx == -1) {
    plc_result.status = BRANCH_NOT_FOUND;
    return plc_result;
  }

  const auto& preferred_status = status_list[preferred_idx];
  const auto& preferred_result = est_results[preferred_idx];
  const auto& preferred_scheduler = preferred_result.scheduler_output;
  if (preferred_status.ok()) return plc_result;

  if (preferred_status.status_code() !=
      PlannerStatusProto::LC_SAFETY_CHECK_FAILED) {
    plc_result.status = BRANCH_FAILED_INTERNAL;
  } else {
    plc_result.status = UNSAFE_OBJECT;
    plc_result.unsafe_object_ids = {preferred_result.unsafe_object_ids.begin(),
                                    preferred_result.unsafe_object_ids.end()};
    // Also find possible solid boundary points since we have no trajectory
    // here to judge whether they also affect plc.
    plc_result.left_solid_boundary =
        preferred_scheduler.lane_change_state.lc_left();
  }

  return plc_result;
}

DriverAction::LaneChangeCommand HandleNewLaneChangeCommand(
    const mapping::LanePath& origin_lane_path, const QALCState& prev_alc_state,
    DriverAction::LaneChangeCommand lc_cmd) {
  if (lc_cmd == DriverAction::LC_CMD_NONE) return lc_cmd;

  if (lc_cmd == DriverAction::LC_CMD_CANCEL) {
    switch (prev_alc_state) {
      case ALC_PREPARE:
        QLOG(INFO) << "ALCC lane change cancelled before start!";
        QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                   QRunEvent::PLC_CANCEL_BEFORE_START);
        return lc_cmd;
      case ALC_ONGOING: {
        if (!origin_lane_path.IsEmpty()) return lc_cmd;

        QLOG(WARNING) << "ALCC Cannot drive back to the lane path before lane "
                         "change: origin lane path not available!";
        QRUNEVENT_WITH_ENUM_NOTICE(
            QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
            QRunEvent::PLC_REJECT_RETURN_ORIGIN_UNAVAILABLE);
        return DriverAction::LC_CMD_NONE;
      }
      case ALC_CROSSING_LANE: {
        QLOG(WARNING) << "ALCC Cannot drive back to the lane path before lane "
                         "change: already crossed boundary!";
        QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                   QRunEvent::PLC_REJECT_RETURN_CROSSED_LANE);
        return DriverAction::LC_CMD_NONE;
      }
      case ALC_STANDBY_ENABLE:
      case ALC_RETURN_COMPLETED:
      case ALC_RETURNING:
      case ALC_COMPLETED:
      case ALC_OFF:
      case ALC_STANDBY:
        return DriverAction::LC_CMD_NONE;
    }
  }

  switch (prev_alc_state) {
    case ALC_STANDBY_ENABLE:
      return lc_cmd;
    case ALC_PREPARE:
    case ALC_ONGOING:
    case ALC_COMPLETED:
    case ALC_RETURN_COMPLETED:
    case ALC_CROSSING_LANE:
    case ALC_RETURNING: {
      QLOG(WARNING)
          << "ALCC lane change rejected: previous lane change not completed!";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_REJECT_PREV_CMD_INCOMPLETE);
      return DriverAction::LC_CMD_NONE;
    }
    case ALC_OFF:
    case ALC_STANDBY:
      return DriverAction::LC_CMD_NONE;
  }
}

std::optional<RouteTargetInfo> BuildRouteTargetInfo(
    const PlannerSemanticMapManager& psmm, const Box2d& ego_box,
    const AlccSchedulerOutput& target_scheduler,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
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
      .plan_id = 2,
      .frenet_frame = std::move(target_frenet_frame_or).value(),
      .ego_frenet_box = ego_frenet_box,
      .drive_passage = target_scheduler.drive_passage,
      .sl_boundary = target_scheduler.sl_boundary,
      .st_traj_mgr = st_traj_mgr};
}

inline bool ShouldConsiderRouteTarget(
    int index, LaneChangeStage lane_change_stage,
    const std::optional<RouteTargetInfo>& route_target_info) {
  return route_target_info.has_value() &&
         ((index + 1 != route_target_info->plan_id &&
           lane_change_stage == LaneChangeStage::LCS_NONE) ||
          lane_change_stage == LaneChangeStage::LCS_PAUSE);
}

absl::Status UpdatePrevLanePaths(
    const PlannerSemanticMapManager& psmm,
    const PlannerSemanticMapManager& prev_low_freq_psmm,
    const mapping::OnlineSemanticMapProto& online_semantic_map,
    const Vec2d& plan_start_point_xy, double plan_start_point_v,
    bool need_origin_lane_path, mapping::LanePath* origin_lane_path,
    mapping::LanePath* target_lane_path, ThreadPool* thread_pool) {
  Future<mapping::LanePath> future_target_lp =
      ScheduleFuture(thread_pool, [&]() {
        if (target_lane_path->IsEmpty()) return mapping::LanePath();

        auto target_lane_path_or = ProjectLanePathToCurrentOnlineMap(
            psmm, online_semantic_map, prev_low_freq_psmm, *target_lane_path,
            plan_start_point_xy, plan_start_point_v, kAlccPlcPreviewDistance,
            thread_pool);
        return target_lane_path_or.ok() ? *target_lane_path_or
                                        : mapping::LanePath();
      });
  const absl::Cleanup wait_target_lp_result = [&future_target_lp]() {
    future_target_lp.Wait();
  };

  if (!origin_lane_path->IsEmpty()) {
    auto origin_lane_path_or = ProjectLanePathToCurrentOnlineMap(
        psmm, online_semantic_map, prev_low_freq_psmm, *origin_lane_path,
        plan_start_point_xy, plan_start_point_v,
        /*check_preview_length=*/0.0, thread_pool);
    if (!origin_lane_path_or.ok() && need_origin_lane_path) {
      return absl::InternalError(origin_lane_path_or.status().message());
    }
    *origin_lane_path = origin_lane_path_or.ok()
                            ? std::move(origin_lane_path_or).value()
                            : mapping::LanePath();
  }

  *target_lane_path = future_target_lp.Get();

  return absl::OkStatus();
}

absl::Status PostSelectTrajectory(
    const MultiTasksAlccPlannerInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry,
    const ExternalCommandStatus& ext_cmd_status,
    std::optional<RouteTargetInfo> route_target_info, int preferred_idx,
    std::vector<AlccSchedulerOutput>* mutable_alcc_scheduler_results,
    std::vector<PlannerStatus>* mutable_status_list,
    std::vector<EstPlannerOutput>* mutable_est_outputs,
    std::vector<EstPlannerDebug>* mutable_est_debugs,
    std::vector<vis::vantage::ChartDataBundleProto>* mutable_chart_data_bundles,
    PathBoundedEstPlannerOutput* output) {
  std::vector<AlccSchedulerOutput>& alcc_scheduler_results =
      *mutable_alcc_scheduler_results;
  std::vector<PlannerStatus>& status_list = *mutable_status_list;
  std::vector<EstPlannerOutput>& est_outputs = *mutable_est_outputs;
  std::vector<EstPlannerDebug>& est_debugs = *mutable_est_debugs;
  std::vector<vis::vantage::ChartDataBundleProto>& chart_data_bundles =
      *mutable_chart_data_bundles;

  ASSIGN_OR_RETURN(
      const auto selected_idx,
      RunAlccSelector(*input.planner_semantic_map_manager, vehicle_geometry,
                      status_list, est_outputs, preferred_idx,
                      output->plc_result.has_value()
                          ? &output->plc_result.value()
                          : nullptr));

  if (selected_idx != 0) {
    std::swap(alcc_scheduler_results[0], alcc_scheduler_results[selected_idx]);
    std::swap(status_list[0], status_list[selected_idx]);
    std::swap(est_outputs[0], est_outputs[selected_idx]);
    std::swap(est_debugs[0], est_debugs[selected_idx]);
    std::swap(chart_data_bundles[0], chart_data_bundles[selected_idx]);
  }

  // Stitch path points and convert to global path points.
  const auto& start_point_info = *input.plan_start_point_info;
  StitchStPathTrajectoryWithPastTrajectory(
      *input.previous_trajectory, start_point_info.start_index_on_prev_traj,
      est_outputs[0].st_path_points, &output->st_path_points_including_past);

  const auto& scheduler_output = est_outputs[0].scheduler_output;
  if (ShouldConsiderRouteTarget(selected_idx,
                                scheduler_output.lane_change_state.stage(),
                                route_target_info)) {
    // Fill only if the selected branch should consider prepare_lc.
    output->route_target_info = std::move(route_target_info);
  }

  auto& updated_alc_state = alcc_scheduler_results[0].alc_state;
  // Check if having been prepared for too long.
  if (updated_alc_state == ALC_PREPARE &&
      ext_cmd_status.plc_prepare_start_time.has_value() &&
      absl::ToDoubleSeconds(input.plan_time -
                            *ext_cmd_status.plc_prepare_start_time) >=
          FLAGS_planner_paddle_lane_change_max_prepare_time) {
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_FAIL_WAIT_TIMEOUT);
    updated_alc_state = ALC_STANDBY_ENABLE;
    alcc_scheduler_results[0].lc_direction = LCD_NONE;
  }
  if (output->plc_result.has_value() &&
      updated_alc_state == ALC_STANDBY_ENABLE) {
    output->plc_result->lane_change_command = DriverAction::LC_CMD_NONE;
  }

  return absl::OkStatus();
}

}  // namespace

// NOLINTNEXTLINE(readability-function-size)
PlannerStatus RunMultiTasksAlccPlanner(const MultiTasksAlccPlannerInput& input,
                                       PathBoundedEstPlannerOutput* output,
                                       ThreadPool* thread_pool) {
  SCOPED_QTRACE("ALCC Task/MultiTasksPlanner");

  const auto& vehicle_geometry =
      input.vehicle_params->vehicle_geometry_params();
  const auto& ext_cmd_status = *input.ext_cmd_status;
  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& smm = *psmm.semantic_map_manager();
  const auto& plan_start_point = input.plan_start_point_info->start_point;

  auto assist_plan_state = *input.assist_plan_state;
  mapping::LanePath origin_lane_path(&smm,
                                     assist_plan_state.origin_lane_path());
  mapping::LanePath target_lane_path(&smm,
                                     assist_plan_state.target_lane_path());

  const auto plan_start_point_xy =
      Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto plan_start_point_v = plan_start_point.v();

  auto lc_cmd = ext_cmd_status.lane_change_command;

  // Step 1: Intialization or reset
  const bool reset_alc_states =
      (origin_lane_path.IsEmpty() &&
       NeedOriginLanePath(assist_plan_state.alc_state())) ||
      !IsLateralAutonomousDrivingMode(input.autonomy_state->autonomy_state());
  if (reset_alc_states) {
    // locate ego pose on semantic map and find current lane
    // path.
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        origin_lane_path,
        FindInitialLanePathFromEgoPose(psmm, *input.pose, thread_pool),
        PlannerStatusProto::REFERENCE_PATH_UNAVAILABLE);
    origin_lane_path = BackwardExtendLanePath(psmm, origin_lane_path,
                                              kDrivePassageKeepBehindLength);

    assist_plan_state.set_alc_state(ALC_STANDBY_ENABLE);
    assist_plan_state.set_lc_direction(LCD_NONE);
    target_lane_path.Clear();
    lc_cmd = DriverAction::LC_CMD_NONE;

    output->plc_result = PlcInternalResult();
    output->plc_result->lane_change_command = lc_cmd;
  } else if (input.prev_low_freq_psmm != nullptr &&
             input.online_semantic_map != nullptr &&
             input.use_online_semantic_map) {
    const auto status = UpdatePrevLanePaths(
        psmm, *input.prev_low_freq_psmm, *input.online_semantic_map,
        plan_start_point_xy, plan_start_point_v,
        NeedOriginLanePath(assist_plan_state.alc_state()), &origin_lane_path,
        &target_lane_path, thread_pool);
    if (!status.ok()) {
      return PlannerStatus(PlannerStatusProto::PROJECT_TO_ONLINE_MAP_FAILED,
                           status.message());
    }
  }

  // Process new lc cmd
  if (!reset_alc_states) {
    const auto processed_lc_cmd = HandleNewLaneChangeCommand(
        origin_lane_path, assist_plan_state.alc_state(), input.new_lc_command);
    lc_cmd = processed_lc_cmd == DriverAction::LC_CMD_NONE
                 ? ext_cmd_status.lane_change_command
                 : processed_lc_cmd;
  }

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto dm_result,
      input.use_online_semantic_map
          ? UpdateLccDrivingMapByOnlineMap(
                psmm, origin_lane_path, target_lane_path,
                *input.online_semantic_map, plan_start_point_xy)
          : UpdateLccDrivingMapByOfflineMap(
                psmm, origin_lane_path, target_lane_path,
                Vec2d(input.pose->pos_smooth().x(),
                      input.pose->pos_smooth().y())),

      PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);
  auto dm = std::move(dm_result.driving_map);
  origin_lane_path = std::move(dm_result.aligned_origin_lane_path);
  target_lane_path = std::move(dm_result.aligned_target_lane_path);

  // Step 3: Build local map
  // Find serveral lane paths from current section.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = &target_lane_path,
          .alc_state = assist_plan_state.alc_state(),
          .lc_direction = assist_plan_state.lc_direction(),
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}),
      PlannerStatusProto::LOCAL_LANE_MAP_BUILDER_FAILED);
  if (FLAGS_planner_local_lane_map_debug_level) {
    SendLocalLaneMapToCanvas(candidate_lanes, psmm, "local_lane_map");
  }
  // Step 4: Update ALC state manager.

  // Step 5: Run scheduler
  // Schedule a lane change continue branch and a return to lane branch
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto scheduler_results,
      RunAlccScheduler(
          AlccSchedulerInput{
              .psmm = &psmm,
              .vehicle_geom = &vehicle_geometry,
              .plan_start_point = &plan_start_point,
              .st_traj_mgr = input.st_traj_mgr.get(),
              .candidate_lanes = &candidate_lanes,
              .lc_cmd = lc_cmd,
              .prev_alc_state = assist_plan_state.alc_state(),
              .lcc_cruising_speed_mps = ext_cmd_status.lcc_cruising_speed_limit,
              .online_map_drift_buffer = input.online_map_drift_buffer},
          thread_pool),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  if (FLAGS_planner_drive_passage_debug_level) {
    for (int i = 0; i < scheduler_results.size(); ++i) {
      SendDrivePassageToCanvas(
          scheduler_results[i].drive_passage,
          "drive_passage_" + QALCState_Name(scheduler_results[i].alc_state));
    }
  }

  // Step 6: Build spacetime trajectory manager
  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_lists;
  st_traj_mgr_lists.reserve(scheduler_results.size());
  const bool on_vision_map = psmm.IsOnVisionMap();
  for (int i = 0; i < scheduler_results.size(); ++i) {
    const auto& drive_passage = scheduler_results[i].drive_passage;
    const auto& sl_boundary = scheduler_results[i].sl_boundary;
    if (FLAGS_planner_alcc_use_st_traj_cutin_filter) {
      st_traj_mgr_lists.emplace_back(
          BuildSpacetimeTrajectoryManagerWithStTrajCutinFilter(
              sl_boundary, drive_passage, *input.object_manager,
              plan_start_point, vehicle_geometry, on_vision_map, thread_pool));
    } else {
      st_traj_mgr_lists.emplace_back(BuildSpacetimeTrajectoryManager(
          SpacetimeTrajectoryManagerBuilderInput{
              .passage = &drive_passage,
              .sl_boundary = &sl_boundary,
              .obj_mgr = input.object_manager.get(),
              .on_vision_map = on_vision_map},
          thread_pool));
    }
  }

  // Find path plan start point with given look ahead duration.
  const auto path_start_point_info = GetStPathPlanStartPointInfo(
      input.path_look_ahead_time, *input.plan_start_point_info,
      *input.previous_trajectory,
      /*trajectory_optimizer_time_step=*/std::nullopt,
      /*last_st_path_plan_start_time=*/std::nullopt);

  const TrafficLightInfoMap empty_tl_map;
  const SmoothedReferenceLineResultMap empty_refline_map;
  const absl::flat_hash_set<std::string> empty_stalled_objects;
  const SceneOutputProto empty_scene_reasoning;
  const ml::captain_net::CaptainNetOutput empty_captain_net_output;

  // Build route target info
  const Box2d ego_box =
      ComputeAvBox(plan_start_point_xy, plan_start_point.path_point().theta(),
                   vehicle_geometry);
  std::optional<RouteTargetInfo> route_target_info = std::nullopt;
  if (scheduler_results.size() >= 2) {
    route_target_info = BuildRouteTargetInfo(
        psmm, ego_box, scheduler_results[1], st_traj_mgr_lists[1]);
  }

  // Step 7: Parallel for running est planner on both branches
  std::vector<PlannerStatus> status_list(scheduler_results.size());
  std::vector<EstPlannerOutput> est_outputs(scheduler_results.size());
  std::vector<EstPlannerDebug> est_debugs(scheduler_results.size());
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(
      scheduler_results.size());

  // Select highlight vehicle branch.
  const std::optional<int> highlight_vehicle_branch =
      SelectHighlightFrontVehicleBranchIndex(
          scheduler_results, ego_box, plan_start_point.path_point().theta());

  ParallelFor(0, scheduler_results.size(), thread_pool, [&](int i) {
    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &dm,
            .semantic_map_manager = &smm,
            .planner_semantic_map_manager = &psmm,
            .plan_id = 0,
            .vehicle_params = input.vehicle_params,
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
            .start_point_info = input.plan_start_point_info,
            .st_path_start_point_info = &path_start_point_info,
            .tl_info_map = &empty_tl_map,
            .smooth_result_map = &empty_refline_map,
            .stalled_objects = &empty_stalled_objects,
            .scene_reasoning = &empty_scene_reasoning,
            .prev_target_lane_path_from_start = &target_lane_path,
            .time_aligned_prev_traj = input.time_aligned_prev_traj_points,
            .lane_change_style = ext_cmd_status.lane_change_style,
            .enable_pull_over = ext_cmd_status.enable_pull_over,
            .enable_traffic_light_stopping =
                ext_cmd_status.enable_traffic_light_stopping,
            .brake_to_stop = ext_cmd_status.brake_to_stop,
            .enable_force_stop = input.is_standwait,
            .st_traj_mgr = &st_traj_mgr_lists[i],
            .log_av_trajectory = input.log_av_trajectory,
            .captain_net_output = &empty_captain_net_output,
            .decision_constraint_config =
                &input.alcc_params->decision_constraint_config(),
            .initializer_params = &input.alcc_params->initializer_params(),
            .trajectory_optimizer_params =
                &input.alcc_params->trajectory_optimizer_params(),
            .speed_finder_params = &input.alcc_params->speed_finder_params(),
            .motion_constraint_params =
                &input.alcc_params->motion_constraint_params(),
            .planner_functions_params =
                &input.alcc_params->planner_functions_params(),
            .vehicle_models_params =
                &input.alcc_params->vehicle_models_params(),
            .speed_finder_lc_radical_params =
                &input.alcc_params->speed_finder_lc_radical_params(),
            .speed_finder_lc_conservative_params =
                &input.alcc_params->speed_finder_lc_conservative_params(),
            .trajectory_optimizer_lc_radical_params =
                &input.alcc_params->trajectory_optimizer_lc_radical_params(),
            .trajectory_optimizer_lc_normal_params =
                &input.alcc_params->trajectory_optimizer_lc_normal_params(),
            .trajectory_optimizer_lc_conservative_params =
                &input.alcc_params
                     ->trajectory_optimizer_lc_conservative_params(),
            .spacetime_planner_object_trajectories_params =
                &input.alcc_params
                     ->spacetime_planner_object_trajectories_params()},
        SchedulerOutput{
            .drive_passage = std::move(scheduler_results[i].drive_passage),
            .sl_boundary = std::move(scheduler_results[i].sl_boundary),
            .lane_change_state = CalculateLaneChangeState(
                scheduler_results[i].av_frenet_box_on_drive_passage,
                scheduler_results[i].alc_state,
                scheduler_results[i].lc_direction),
            .av_frenet_box_on_drive_passage =
                scheduler_results[i].av_frenet_box_on_drive_passage},
        &est_outputs[i], &est_debugs[i], &chart_data_bundles[i], thread_pool);
  });

  // All est planners use the same alerted vehicle.
  FillSameAlertedFrontVehicle(highlight_vehicle_branch, &est_outputs);

  // Collect speed-considered object ids by all est planners.
  ObjectsPredictionProto speed_considered_objects_prediction =
      CollectSpeedConsideredObjectsPrediction(
          *input.object_manager,
          CollectSpeedConsiderObjectsPartialStTrajectory(
              est_outputs, /*fallback_considered_st_objects=*/{}),
          FLAGS_planner_export_all_prediction_to_speed_considered);

  const auto preferred_idx =
      FindPreferredEstIndex(est_outputs, candidate_lanes, lc_cmd);
  if (lc_cmd != DriverAction::LC_CMD_NONE) {
    output->plc_result = DeterminePlcResultFromEstResults(
        est_outputs, status_list, preferred_idx);
    output->plc_result->lane_change_command = lc_cmd;
  }

  // Step 8: Select the better branch by checking status and lane change safety.
  const auto selecter_status = PostSelectTrajectory(
      input, vehicle_geometry, ext_cmd_status, std::move(route_target_info),
      preferred_idx, &scheduler_results, &status_list, &est_outputs,
      &est_debugs, &chart_data_bundles, output);

  output->est_status_list = std::move(status_list);
  output->est_planner_output_list = std::move(est_outputs);
  output->est_planner_debug_list = std::move(est_debugs);
  output->chart_data_list = std::move(chart_data_bundles);
  output->alc_state = scheduler_results[0].alc_state;
  output->lc_direction = scheduler_results[0].lc_direction;
  output->speed_considered_objects_prediction =
      std::move(speed_considered_objects_prediction);
  output->path_start_relative_index =
      path_start_point_info.relative_index_from_plan_start_point;

  if (input.use_online_semantic_map) {
    output->online_map_id = input.online_semantic_map->update_id();
  }

  output->low_freq_psmm = input.planner_semantic_map_manager;
  output->driving_map_topo =
      std::make_shared<const DrivingMapTopo>(std::move(dm));

  output->origin_lane_path = candidate_lanes[1];
  output->target_lane_path =
      FindFinalTargetLanePath(candidate_lanes, output->lc_direction);

  return selecter_status.ok()
             ? OkPlannerStatus()
             : PlannerStatus(PlannerStatusProto::SELECTOR_FAILED,
                             selecter_status.message());
}

}  // namespace qcraft::planner
