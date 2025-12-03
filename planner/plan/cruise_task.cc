#include "onboard/planner/plan/cruise_task.h"

// IWYU pragma: no_include <cxxabi.h>  // for __forced_unwind
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>  // IWYU pragma: keep
#include <cmath>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "common/proto/qacc.pb.h"
#include "common/proto/qalc.pb.h"
#include "common/proto/qlcc.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/base/macros.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/car_common.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/door_open_decider.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"
#include "onboard/planner/decision/turn_signal_decider.h"
#include "onboard/planner/driving_state.h"
#include "onboard/planner/emergency_stop.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/proto/captain_net_debug.pb.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature_extractor.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature_proto_converter.h"
#include "onboard/planner/ml/context_feature_extractor/proto/context_feature.pb.h"
#include "onboard/planner/ml/context_groundtruth_extractor/context_groundtruth_extractor.h"
#include "onboard/planner/ml/context_groundtruth_extractor/proto/context_groundtruth.pb.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/acc/acc_planner_output.h"
#include "onboard/planner/plan/acc/acc_task_internal.h"
#include "onboard/planner/plan/acc/acc_task_output.h"
#include "onboard/planner/plan/async_cruise_planner.h"
#include "onboard/planner/plan/async_planner_output.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/plan/multi_tasks_cruise_planner_input.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/plan/previous_trajectory_planner.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/proto/route_external_command.pb.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scene/bus_station_stalled_objects_filter.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scene/scene_reasoning_util.h"
#include "onboard/planner/scene/scene_understanding.h"
#include "onboard/planner/scheduler/local_map_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/planner/selector/selector_output.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/proto/assist_driving_system_state.pb.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/source_location.h"

DEFINE_int32(
    prev_traj_planner_max_frame, 20,
    "The max frame number that the previous-trajectory planner is allowed "
    "to be consecutively used. If set to zero, the previous-trajectory planner "
    "is disabled.");

namespace qcraft::planner {

namespace {

struct PlcExtraInfo {
  std::vector<std::string> unsafe_object_ids;
  std::optional<bool> left_solid_boundary = std::nullopt;
  mapping::LanePath* plc_target_lane_path_ptr = nullptr;
};

SceneReasoningOutput RunSceneReasoningAndFillDebug(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const RouteNaviInfo& route_navi_info,
    const TrafficLightInfoMap& tl_info_map, const PlannerObjectManager& obj_mgr,
    const ObjectsPredictionProto& prediction,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SensorFovsProto* sensor_fovs, PlannerDebugProto* debug_proto,
    ThreadPool* thread_pool) {
  const auto lane_paths_or =
      BuildLocalMap(psmm, route_sections, route_navi_info);
  if (!lane_paths_or.ok()) {
    QLOG(WARNING) << "RunSceneReasoningAndFillDebug failed in BuildLocalMap: "
                  << lane_paths_or.status();
    return SceneReasoningOutput();
  }

  auto scene_reasoning_output_or = RunSceneReasoning(
      SceneReasoningInput{.psmm = &psmm,
                          .prediction = &prediction,
                          .tl_info_map = &tl_info_map,
                          .lane_paths = &(*lane_paths_or),
                          .sensor_fovs = sensor_fovs,
                          .route_sections = &route_sections,
                          .plan_start_point = &plan_start_point},
      thread_pool);
  if (!scene_reasoning_output_or.ok()) {
    QLOG(WARNING)
        << "RunSceneReasoningAndFillDebug failed in RunSceneReasoning: "
        << scene_reasoning_output_or.status();
    return SceneReasoningOutput();
  }
  const auto& scene_output_proto =
      scene_reasoning_output_or->scene_output_proto;

  *debug_proto->mutable_scene_understanding_debug() = scene_output_proto;
  // Record stalled objects.
  ParseObjectAnnotationToDebugProto(scene_output_proto.objects_annotation(),
                                    obj_mgr, debug_proto);
  // Record traffic waiting objects.
  ParseTrafficWaitingQueueToDebugProto(
      scene_output_proto.traffic_waiting_queue(), obj_mgr, debug_proto);

  return *scene_reasoning_output_or;
}

void ClearStateInManualDrivingMode(const AutonomyStateProto& autonomy_state,
                                   PlannerState* planner_state) {
  if (IsLateralAutonomousDrivingMode(autonomy_state.autonomy_state())) return;

  planner_state->selector_state.Reset();
}

void FillEstPlannerDebugProto(const PlannerSemanticMapManager& psmm,
                              const PoseProto& pose, TurnSignal route_signal,
                              const ExternalCommandStatus& ext_cmd_status,
                              const PlannerStatus& planner_status,
                              const SchedulerOutput& scheduler_output,
                              const EstPlannerOutput& est_output,
                              EstPlannerDebug est_debug,
                              EstPlannerDebugProto* est_planner_debug) {
  const auto turn_signal_res = DecideTurnSignal(
      psmm, route_signal,
      /*pre_lane_change_signal=*/TurnSignal::TURN_SIGNAL_NONE,
      scheduler_output.drive_passage.lane_path(), est_output.redlight_lane_id,
      scheduler_output.lane_change_state, ext_cmd_status,
      scheduler_output.drive_passage,
      scheduler_output.av_frenet_box_on_drive_passage,
      TurnSignalResult{.signal = scheduler_output.planner_turn_signal,
                       .reason = scheduler_output.turn_signal_reason},
      pose);

  est_planner_debug->set_turn_signal_reason_enum(turn_signal_res.reason);

  planner_status.ToProto(est_planner_debug->mutable_planner_status());

  *est_planner_debug->mutable_filtered() =
      std::move(est_debug.filtered_prediction_trajectories);

  *est_planner_debug->mutable_st_planner_object_trajectories() =
      std::move(est_debug.st_planner_object_trajectories);

  ToSchedulerOutputProto(scheduler_output,
                         est_planner_debug->mutable_scheduler());

  *est_planner_debug->mutable_constraint() =
      std::move(est_debug.decision_constraints);

  *est_planner_debug->mutable_initializer() =
      std::move(est_debug.initializer_debug_proto);

  *est_planner_debug->mutable_trajectory_optimizer() =
      std::move(est_debug.optimizer_debug_proto);

  *est_planner_debug->mutable_lane_change_safety_debug_proto() =
      std::move(est_debug.lane_change_safety_debug_proto);

  *est_planner_debug->mutable_st_path_points() = {
      est_output.st_path_points.begin(), est_output.st_path_points.end()};

  *est_planner_debug->mutable_capnet_traj() =
      std::move(est_debug.capnet_traj_debug);

  *est_planner_debug->mutable_speed_finder() =
      std::move(est_debug.speed_finder_debug);

  *est_planner_debug->mutable_traj_validation() =
      std::move(est_debug.traj_validation_result);
}

bool ShouldResetAlcState(ResetReasonProto::Reason reset_reason) {
  switch (reset_reason) {
    case ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND:
    case ResetReasonProto::PREV_NOW_POINT_NOT_FOUND:
    case ResetReasonProto::NON_AUTONOMY:
    case ResetReasonProto::FIRST_ENGAGE:
    case ResetReasonProto::FULL_STOP:
    case ResetReasonProto::PREVIOUS_AEB:
    case ResetReasonProto::NEW_FREESPACE_PATH:
    case ResetReasonProto::STEER_ONLY_ENGAGE:
    case ResetReasonProto::STEER_ONLY:
    case ResetReasonProto::SPEED_ONLY:
    case ResetReasonProto::SPEED_ONLY_ENGAGE:
    case ResetReasonProto::PREV_PATH_EMPTY:
      return true;
    case ResetReasonProto::NONE:
    case ResetReasonProto::LON_ERROR_TOO_LARGE:
    case ResetReasonProto::LAT_ERROR_TOO_LARGE:
      return false;
  }
}

void ClearPrevEstPlannerState(PlannerState* planner_state) {
  planner_state->prev_target_lane_path.Clear();
  planner_state->station_anchor = mapping::LanePoint();
  planner_state->prev_length_along_route = 0.0;
  planner_state->prev_max_reach_length = 0.0;
  planner_state->prev_smooth_state = false;

  planner_state->decider_state.Clear();
  planner_state->initializer_state.Clear();

  planner_state->lane_change_state.Clear();
  planner_state->prev_lane_path_before_lc.Clear();

  planner_state->st_planner_object_trajectories.Clear();
  planner_state->prev_traj_end_info = std::nullopt;

  planner_state->previously_triggered_aeb = false;
}

void ParseAsyncPlannerOutputToPlannerState(const AsyncPlannerOutput& output,
                                           bool is_planner_async_mode,
                                           PlannerState* planner_state) {
  FUNC_QTRACE();

  // NOTE: Check updated info in high freq frame in AsyncPlannerOuput. If the
  // variable is updated in high freq frame, take it from AsyncPlannerOutput
  // directly rather than from lastest_low_freq_output.

  const auto& est_planner_output = output.latest_low_freq_output->est_output;
  const auto& scheduler_output = est_planner_output.scheduler_output;
  const auto& drive_passage = scheduler_output.drive_passage;

  planner_state->prev_target_lane_path =
      drive_passage.extend_lane_path()
          .BeforeLastOccurrenceOfLanePoint(drive_passage.lane_path().back())
          .AfterArclength(
              scheduler_output.av_frenet_box_on_drive_passage.s_min);

  planner_state->station_anchor =
      drive_passage.FindNearestStationAtS(0.0).GetLanePoint();

  planner_state->prev_length_along_route = scheduler_output.length_along_route;

  planner_state->prev_max_reach_length = scheduler_output.max_reach_length;

  planner_state->prev_smooth_state = scheduler_output.should_smooth;

  // NOTE: The following fields are faked by multi-task scheduler and
  // should be removed in the future.
  planner_state->lane_change_state = scheduler_output.lane_change_state;
  planner_state->prev_lane_path_before_lc =
      scheduler_output.lane_path_before_lc;
  if (is_planner_async_mode) {
    planner_state->decider_state = output.decider_state;
    planner_state->prev_traj_end_info = output.trajectory_end_info;
  } else {
    planner_state->decider_state = est_planner_output.decider_state;
    planner_state->prev_traj_end_info = est_planner_output.trajectory_end_info;
  }
  planner_state->initializer_state = est_planner_output.initializer_state;
  planner_state->st_planner_object_trajectories =
      est_planner_output.st_planner_object_trajectories;
  planner_state->selected_trajectory_optimizer_state_proto =
      est_planner_output.trajectory_optimizer_state_proto;

  planner_state->prev_low_freq_psmm =
      output.latest_low_freq_output->low_freq_psmm;

  planner_state->previous_st_path_global_including_past =
      output.latest_low_freq_output->st_path_points_global_including_past;

  planner_state->selector_state = output.latest_low_freq_output->selector_state;
}

[[maybe_unused]] std::vector<Vec2d> CollectSolidBoundaryPoints(
    const DrivePassage& drive_passage, bool left_solid_boundary) {
  constexpr double kMaxSolidBoundaryDist = 50.0;  // m.
  std::vector<Vec2d> points;
  for (const auto& station : drive_passage.stations()) {
    if (station.accumulated_s() > kMaxSolidBoundaryDist) break;
    const auto boundaries =
        *station.QueryEnclosingLaneBoundariesAt(/*signed_lat=*/0.0);
    const auto& boundary =
        left_solid_boundary ? boundaries.left : boundaries.right;
    if (boundary.has_value() && boundary->IsSolid(/*query_lat_offset=*/0.0) &&
        boundary->type != StationBoundaryType::VIRTUAL_CURB &&
        boundary->lat_offset < kMaxHalfLaneWidth) {
      points.push_back(station.lat_point(boundary->lat_offset));
    }
  }
  return points;
}

HmiContentProto FillHmiContentProto(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& target_lane_path,
    const std::vector<std::string>& unsafe_object_ids,
    const std::vector<ApolloTrajectoryPointProto>& planned_traj_points,
    const PlannerStatus& est_status,
    const SceneReasoningOutput& scene_reasoning_output,
    const mapping::LanePath* plc_target_lane_path_ptr,
    const AsyncPlannerOutput* async_output,
    std::optional<bool> left_solid_boundary, bool is_planner_async_mode,
    double av_half_width) {
  HmiContentInput hmi_content_input{
      .psmm = &psmm,
      .lane_path = &target_lane_path,
      .unsafe_object_ids = &unsafe_object_ids,
      .plc_target_lane_path = plc_target_lane_path_ptr,
      .av_half_width = av_half_width};
  hmi_content_input.distance_to_roadblock =
      scene_reasoning_output.distance_to_roadblock;

  if (!est_status.ok()) {
    return ReportHmiContent(hmi_content_input);
  }

  const auto& scheduler_output =
      async_output->latest_low_freq_output->est_output.scheduler_output;
  hmi_content_input.borrow_lane = scheduler_output.borrow_lane;
  hmi_content_input.request_help_lane_change_by_route =
      scheduler_output.request_help_lane_change_by_route;
  hmi_content_input.drive_passage = &scheduler_output.drive_passage;
  hmi_content_input.traj_points = &planned_traj_points;
  if (is_planner_async_mode) {
    if (async_output->alerted_front_vehicle.has_value()) {
      hmi_content_input.alerted_front_vehicle =
          &async_output->alerted_front_vehicle.value();
    }
    hmi_content_input.distance_to_traffic_light_stop_line =
        async_output->distance_to_traffic_light_stop_line;
    hmi_content_input.collision_warning_request =
        async_output->collision_warning_request;
  } else {
    const auto& est_planner_output =
        async_output->latest_low_freq_output->est_output;
    if (est_planner_output.alerted_front_vehicle.has_value()) {
      hmi_content_input.alerted_front_vehicle =
          &est_planner_output.alerted_front_vehicle.value();
    }
    hmi_content_input.distance_to_traffic_light_stop_line =
        est_planner_output.distance_to_traffic_light_stop_line;
    hmi_content_input.collision_warning_request =
        est_planner_output.collision_warning_request;
  }
  if (async_output->latest_low_freq_output->nudge_object_info.has_value()) {
    hmi_content_input.nudge_object_info =
        &async_output->latest_low_freq_output->nudge_object_info.value();
  }
  if (async_output->latest_low_freq_output->selector_output.has_value()) {
    hmi_content_input.all_trajectories_blocked =
        async_output->latest_low_freq_output->selector_output
            ->all_trajectories_blocked;
    hmi_content_input.lc_left =
        async_output->latest_low_freq_output->selector_output->turn_signal ==
        TurnSignal::TURN_SIGNAL_LEFT;
    hmi_content_input.lane_change_for_obstacle_fail =
        async_output->latest_low_freq_output->selector_output
            ->lane_change_for_obstacle_fail;
  }
  hmi_content_input.lane_change_type =
      async_output->latest_low_freq_output->selector_state.lane_change_type;
  hmi_content_input.lane_change_general_type =
      async_output->latest_low_freq_output->selector_state
          .lane_change_general_type;
  if (async_output->latest_low_freq_output->selector_state.turn_signal !=
      TurnSignal::TURN_SIGNAL_NONE) {
    hmi_content_input.lane_change_stage =
        async_output->latest_low_freq_output->selector_state.pre_turn_signal ==
                TurnSignal::TURN_SIGNAL_NONE
            ? LaneChangeStage::LCS_EXECUTING
            : LaneChangeStage::LCS_WAITING;
  } else {
    hmi_content_input.lane_change_stage = LaneChangeStage::LCS_NONE;
  }

  HmiContentProto hmi_proto = ReportHmiContent(hmi_content_input);

  // TODO(weijun): Merge with function above.
  if (est_status.ok() && left_solid_boundary.has_value()) {
    *hmi_proto.mutable_path_boundary() = ReportBoundaryPointsToHmiContent(
        CollectSolidBoundaryPoints(
            async_output->latest_low_freq_output->est_output.scheduler_output
                .drive_passage,
            *left_solid_boundary),
        *left_solid_boundary, HmiPathBoundaryProto::STYLE_WARN);
  }
  return hmi_proto;
}

void UpdateRequestHelpLaneChangeByRoute(const PlannerStatus& est_status,
                                        const AsyncPlannerOutput* async_output,
                                        ExternalCommandStatus* ext_cmd_status) {
  if (est_status.ok() && !IsRunModeL4() &&
      FLAGS_planner_enable_route_lane_change_fail &&
      async_output->latest_low_freq_output->selector_output.has_value() &&
      async_output->latest_low_freq_output->selector_output->in_high_way) {
    const bool request_help_lane_change_by_route =
        async_output->latest_low_freq_output->est_output.scheduler_output
            .request_help_lane_change_by_route;
    ext_cmd_status->output.is_route_lane_change_fail =
        request_help_lane_change_by_route;

    if (request_help_lane_change_by_route &&
        ext_cmd_status->output.planner_to_router_command ==
            ExternalRouterCommand::NONE) {
      ext_cmd_status->output.planner_to_router_command =
          ExternalRouterCommand::ANOTHER_ROUTE;
    }
  }
}

PlcExtraInfo HandlePlcResult(const PlannerStatus& est_status,
                             absl::Time plan_time,
                             const AsyncPlannerOutput* async_output,
                             ExternalCommandStatus* ext_cmd_status,
                             PlannerState* planner_state,
                             PlannerDebugProto* debug_proto) {
  PlcExtraInfo plc_extra_info;
  if (!planner_state->preferred_lane_path.IsEmpty() && !est_status.ok()) {
    planner_state->preferred_lane_path.Clear();
    ext_cmd_status->alc_state = ALC_STANDBY_ENABLE;
    ext_cmd_status->lane_change_command = DriverAction::LC_CMD_NONE;
    ext_cmd_status->plc_prepare_start_time = std::nullopt;
    QLOG(ERROR) << "Teleop lane change failed: all branches failed.";
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_FAIL_ALL_BRANCHES);
  }

  if (async_output->latest_low_freq_output != nullptr &&
      async_output->latest_low_freq_output->plc_result.has_value()) {
    const auto& plc_result = async_output->latest_low_freq_output->plc_result;
    planner_state->preferred_lane_path = plc_result->preferred_lane_path;
    ext_cmd_status->alc_state = async_output->latest_low_freq_output->alc_state;
    ext_cmd_status->lane_change_command = plc_result->lane_change_command;

    if (ext_cmd_status->alc_state == ALC_PREPARE) {
      if (!ext_cmd_status->plc_prepare_start_time.has_value()) {
        ext_cmd_status->plc_prepare_start_time = plan_time;
      }
    } else if (ext_cmd_status->plc_prepare_start_time.has_value()) {
      ext_cmd_status->plc_prepare_start_time = std::nullopt;
    }

    const auto plc_status = plc_result->status;
    if (plc_status == PlcInternalStatus::SOLID_BOUNDARY ||
        plc_status == PlcInternalStatus::UNSAFE_OBJECT) {
      plc_extra_info.unsafe_object_ids = plc_result->unsafe_object_ids;
      plc_extra_info.left_solid_boundary = plc_result->left_solid_boundary;
      if (plc_status == PlcInternalStatus::UNSAFE_OBJECT) {
        plc_extra_info.plc_target_lane_path_ptr =
            &planner_state->preferred_lane_path;
      }
    }

    debug_proto->set_plc_status(plc_status);
  }

  return plc_extra_info;
}

void UpdateAlternateRoute(const PlannerStatus& est_status,
                          const AsyncPlannerOutput* async_output,
                          const RouteManagerOutput* route_output,
                          ExternalCommandStatus* ext_cmd_status) {
  if (est_status.ok() && route_output->alter_route_msg.has_value() &&
      route_output->alter_route_msg->roundabout_distance <
          kAlternateRouteAllowRoundaboutDist &&
      async_output->latest_low_freq_output->est_output.scheduler_output
          .switch_alternate_route) {
    QEVENT_EVERY_N_SECONDS("zuowei", "trigger_alternate_route",
                           /*every_n_seconds=*/5.0, [&](QEvent* /*qevent*/) {});
    if (ext_cmd_status->output.planner_to_router_command ==
        ExternalRouterCommand::NONE) {
      ext_cmd_status->output.planner_to_router_command =
          ExternalRouterCommand::SWITCH_ROUTE;
    }
  }
}

PlannerStatus UpdateRouteResult(bool rerouted, const Vec2d& ego_pos,
                                const PlannerSemanticMapManager& psmm,
                                const RouteManagerOutput* route_output,
                                PlannerState* planner_state) {
  if (IsPlannerAsync(FLAGS_planner_async_low_freq_cycle_iterations) &&
      rerouted) {
    planner_state->async_planner_state.wait_path_switch_route = true;
    planner_state->async_planner_state.wait_speed_switch_route = true;
  }
  const int cur_counter = (planner_state->async_planner_state.counter + 1) %
                          (FLAGS_planner_async_low_freq_cycle_iterations + 1);
  const bool switch_speed_route_sections =
      (rerouted && HasValidRouteResults(*route_output) &&
       !IsPlannerAsync(FLAGS_planner_async_low_freq_cycle_iterations)) ||
      (!planner_state->async_planner_state.wait_path_switch_route &&
       planner_state->async_planner_state.wait_speed_switch_route &&
       ShouldRetrieveLowFreqResult(
           cur_counter, FLAGS_planner_async_low_freq_cycle_iterations));
  if (switch_speed_route_sections) {
    planner_state->async_planner_state.wait_speed_switch_route = false;
  }

  // Dynamic update route in NOA mode.
  if (!IsRunModeL4() && !rerouted && HasValidRouteResults(*route_output) &&
      !planner_state->prev_route_sections.empty()) {
    auto tailored_sections =
        AppendRouteSectionsToTail(planner_state->prev_route_sections,
                                  route_output->route_sections_from_current);
    if (!tailored_sections.ok()) {
      QLOG(INFO) << "Planner prev route sections does not match route, reset";
      planner_state->prev_route_sections.Clear();
    } else {
      planner_state->prev_route_sections = std::move(tailored_sections).value();
    }
  }

  // Initialize or switch route sections.
  if (planner_state->prev_route_sections.empty() ||
      switch_speed_route_sections) {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        planner_state->prev_route_sections,
        BackwardExtendRouteSectionsFromPos(
            psmm, route_output->route_sections_from_current, ego_pos,
            kDrivePassageKeepBehindLength),
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);
    auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        psmm, planner_state->prev_route_sections, ego_pos,
        route_output->route_navi_info);

    if (target_lane_path_or.ok()) {
      planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
    } else {
      // BANDAID(SCENARIOS-689): Force to recalculate prev target lane path by
      // clearing prev route section.
      planner_state->prev_route_sections.Clear();
      return PlannerStatus(
          PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED,
          target_lane_path_or.status().message());
    }
  }

  if (planner_state->prev_target_lane_path.IsEmpty()) {
    // Should have been filled at the end of the last iteration.
    return PlannerStatus(PlannerStatusProto::PLANNER_STATE_INCOMPLETE,
                         "Prev target lane path empty.");
  }

  return OkPlannerStatus();
}

PlannerStatus UpdateAebStateToOutput(
    const PlannerStatus& est_status, const AutonomyStateProto& autonomy_state,
    bool is_planner_async_mode,
    absl::StatusOr<PreviousTrajectoryPlannerOutput> prev_traj_result,
    const AebPlannerOutput* aeb_output, PlannerState* planner_state,
    AsyncPlannerOutput* async_output, PlannerDebugProto* debug_proto,
    std::vector<ApolloTrajectoryPointProto>* planned_traj_points) {
  PlannerStatus status = OkPlannerStatus();
  if (aeb_output->emergency_stop_info.has_value()) {
    // TODO(bo): Add a qevent.
    debug_proto->set_active_planner(AEB_PLANNER);
    *planned_traj_points = aeb_output->trajectory_points;
    planner_state->previous_trajectory_plan_counter = 0;
    planner_state->previously_triggered_aeb = true;
  } else if (est_status.ok()) {
    *planned_traj_points =
        is_planner_async_mode
            ? std::move(async_output->traj_points)
            : async_output->latest_low_freq_output->est_output.traj_points;

    if (!async_output->latest_low_freq_output->est_output.scheduler_output
             .is_fallback) {
      debug_proto->set_active_planner(EST_PLANNER);
    } else {
      QEVENT("renjie", "use_fallback_planner", [&](QEvent* /*qevent*/) {});
      debug_proto->set_active_planner(FALLBACK_PLANNER);
    }

    planner_state->previous_trajectory_plan_counter = 0;
    ParseAsyncPlannerOutputToPlannerState(*async_output, is_planner_async_mode,
                                          planner_state);

  } else if (prev_traj_result.ok()) {
    QEVENT("renjie", "use_prev_traj_planner", [&](QEvent* /*qevent*/) {});
    QLOG(WARNING) << "Est planner fails: " << est_status.message();
    QLOG(WARNING) << "Use previous-trajectory planner!";
    debug_proto->set_active_planner(PREV_TRAJ_PLANNER);
    *planned_traj_points = std::move(prev_traj_result->trajectory_points);
    planner_state->previous_trajectory_plan_counter++;
  } else {
    // Call AEB planner if no valid plan found.
    QLOG(WARNING) << "Est planner fails: " << est_status.message();
    QLOG(WARNING) << "Previous-trajectory planner fails: "
                  << prev_traj_result.status().ToString();
    QLOG(WARNING) << "Call AEB planner for no valid plan found!";
    debug_proto->set_active_planner(AEB_PLANNER);
    *planned_traj_points = aeb_output->trajectory_points;
    planner_state->previous_trajectory_plan_counter = 0;
    planner_state->previously_triggered_aeb = true;
    if (!IsRunModeL4() || !IS_AUTO_DRIVE(autonomy_state.autonomy_state())) {
      const std::string reason = absl::StrCat(
          "No valid plan.\n  Est planner error: ", est_status.message(),
          "  Previous-trajectory error: ",
          prev_traj_result.status().ToString());
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_PLANNER_PLAN_INVALID,
                        "No valid plan. ", reason);
      status = PlannerStatus(est_status.status_code(), reason);
    }
  }
  return status;
}

void SetNoaCustomerRequirement(
    const NoaReqParamsProto& req_proto, double cur_v,
    SpeedFinderParamsProto* speed_finder_params,
    MotionConstraintParamsProto* motion_constraint_params) {
  const auto max_acc_vel_plf =
      PiecewiseLinearFunctionFromProto(req_proto.max_acc_vel_plf());
  const auto max_jerk_vel_plf =
      PiecewiseLinearFunctionFromProto(req_proto.max_jerk_vel_plf());
  const auto min_decel_vel_plf =
      PiecewiseLinearFunctionFromProto(req_proto.min_decel_vel_plf());
  const double max_jerk = std::fabs(max_jerk_vel_plf(cur_v));

  const double max_deceleration = min_decel_vel_plf(cur_v);
  const double max_acceleration = max_acc_vel_plf(cur_v);

  motion_constraint_params->set_max_deceleration(max_deceleration);
  motion_constraint_params->set_max_acceleration(max_acceleration);
  motion_constraint_params->set_max_decel_jerk(-max_jerk);  // Negative.
  motion_constraint_params->set_max_accel_jerk(max_jerk);   // Positive.
  speed_finder_params->set_follow_standstill_distance(
      req_proto.follow_standstill_distance());
}

// NOTE(jiayu): If enable_obstacle_lane_change is false, av will not perform
// lane change for stalled objects and slow-moving objects; if enable_lc_objects
// is false,av will not perform lane change for stalled objects only. if
// enable_bus_station_stalled_object_filter is true, (LARGE)VEHICLE near bus
// station will not be considered as stalled object.
absl::flat_hash_set<std::string> FillStalledObjects(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const PlannerObjectManager& obj_mgr,
    const ::google::protobuf::RepeatedPtrField<PlannerDebugProto::ObjectInfo>&
        raw_stalled_objects,
    bool enable_lc_objects, bool enable_obstacle_lane_change,
    bool enable_bus_station_stalled_object_filter) {
  absl::flat_hash_set<std::string> stalled_objects;
  if (!enable_lc_objects || !enable_obstacle_lane_change) {
    return stalled_objects;
  }
  std::optional<BusStationStalledObjectsFilter> filter = std::nullopt;
  if (enable_bus_station_stalled_object_filter == true) {
    filter = BusStationStalledObjectsFilter(psmm, route_sections);
  }
  for (const auto& stalled_object : raw_stalled_objects) {
    if (filter.has_value()) {
      const auto* obj_ptr = obj_mgr.FindObjectById(stalled_object.id());
      if (UNLIKELY(obj_ptr == nullptr)) continue;
      if (filter->IsFiltered(psmm, obj_ptr->pose().pos(), obj_ptr->type())) {
        continue;
      }
    }
    stalled_objects.insert(stalled_object.id());
  }
  return stalled_objects;
}

PlannerParamsProto RecreatePlannerParam(
    const CruiseTaskInput& input, const ExternalCommandStatus& ext_cmd_status) {
  auto planner_params = *QCHECK_NOTNULL(input.planner_params);

  UpdateFollowHeadwayTimeAccordHmi(planner_params.mutable_speed_finder_params(),
                                   ext_cmd_status.following_distance_level);
  // Noa specific custom settings.
  if (!IsRunModeL4()) {
    SetNoaCustomerRequirement(
        planner_params.noa_params().noa_req_params(),
        input.plan_start_point_info->start_point.v(),
        planner_params.mutable_speed_finder_params(),
        planner_params.mutable_motion_constraint_params());
  }

  return planner_params;
}

void UpdateResultByAcc(const PlannerStatus& est_status, bool is_standwait,
                       bool prev_collision_warning_request, double plan_start_v,
                       double follow_time_headway, const absl::Time& plan_time,
                       const std::shared_ptr<PlannerSemanticMapManager>&
                           planner_semantic_map_manager,
                       AccPlannerOutput acc_planner_output,
                       CruiseTaskOutput* result, PlannerState* planner_state,
                       ExternalCommandStatus* ext_cmd_status) {
  if (est_status.ok()) {
    AccTaskOutput acc_task_output;
    FillAccRelatedOutput(
        std::move(acc_planner_output), is_standwait,
        prev_collision_warning_request, plan_start_v, follow_time_headway,
        plan_time, ext_cmd_status->lcc_cruising_speed_limit, &acc_task_output);

    *planner_state->assist_plan_state.mutable_acc_task() =
        std::move(acc_task_output.acc_task_proto);
    if (acc_task_output.cruising_speed_limit.has_value()) {
      ext_cmd_status->output.cruising_speed_limit =
          acc_task_output.cruising_speed_limit;
      VLOG(2) << "Output speed limit "
              << acc_task_output.cruising_speed_limit.value();
    }
    ext_cmd_status->acc_state = ACC_OFF;
    ext_cmd_status->lcc_state = LCC_OFF;

    result->trajectory_info = std::move(acc_task_output.trajectory_info);
    result->debug_info = std::move(acc_task_output.debug_info);
    result->chart_data = std::move(acc_task_output.chart_data);
    result->hmi_content = std::move(acc_task_output.hmi_content);
  }
  PlannerStatus est_planner_status(
      PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
      "No valid multi-task est planner result has arrived yet.");
  est_planner_status.ToProto(result->debug_info.mutable_est_planner_status());
  est_status.ToProto(result->debug_info.mutable_planner_status());
  planner_state->prev_low_freq_psmm = planner_semantic_map_manager;
}

void FillPrevTargetLanePath(const CruiseTaskInput& input, const Vec2d& ego_pos,
                            PlannerState* planner_state) {
  if (planner_state->prev_target_lane_path.IsEmpty()) {
    auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        *input.planner_input->planner_semantic_map_manager,
        planner_state->prev_route_sections, ego_pos,
        input.route_output->route_navi_info);
    if (target_lane_path_or.ok()) {
      planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
    }
  }
}

std::shared_ptr<const ml::ContextFeature> ConstructPlannerMlContextFeature(
    const CruiseTaskInput& input) {
  if ((IsRunModeL4() && (FLAGS_planner_enable_captain_net_j5 ||
                         FLAGS_planner_enable_captain_net_onnx_trt)) ||
      FLAGS_planner_dumping_ml_data_in_simulation) {
    const ml::ContextFeatureExtractionInput context_feature_extraction_input{
        .av_context = input.planner_input->av_context.get(),
        .traffic_light_states = input.planner_input->traffic_light_states.get(),
        .object_manager = input.object_manager.get(),
        .real_objects = input.planner_input->real_objects,
        .virtual_objects = input.planner_input->virtual_objects,
        .psmm = input.planner_input->planner_semantic_map_manager.get(),
    };
    return std::make_shared<const ml::ContextFeature>(
        ml::ExtractContextFeature(context_feature_extraction_input));
  }
  return nullptr;
}

void FillPlannerDebugAndChart(
    const CruiseTaskInput& input, const PlannerStatus& on_road_plan_status,
    const PlannerStatus& est_status, const TurnSignalResult& turn_signal_result,
    const ExternalCommandStatus& ext_cmd_status,
    const PlannerSemanticMapManager& psmm, bool is_planner_async_mode,
    const TrafficLightInfoMap& tl_info_map,
    const mapping::LanePath& prev_target_lane_path,
    AsyncPlannerOutput* async_output, PlannerDebugProto* debug_proto,
    vis::vantage::ChartsDataProto* chart_data) {
  if (est_status.ok()) {
    debug_proto->set_turn_signal_reason_enum(turn_signal_result.reason);
  }

  if (IsNonEmptyPlannerResult(est_status)) {
    // Fill fallback debug and charts either when all est branches failed or
    // when fallback result is selected.
    if (async_output->retrived_low_freq_output != nullptr) {
      debug_proto->set_path_start_relative_index(
          async_output->retrived_low_freq_output->path_start_relative_index);
      *debug_proto->mutable_auto_tuning_data() =
          std::move(async_output->retrived_low_freq_output->auto_tuning_data);

      // Some fields will be moved.
      auto& multi_est_results = *async_output->retrived_low_freq_output;

      const bool fill_fallback =
          !est_status.ok() || multi_est_results.est_planner_output_list[0]
                                  .scheduler_output.is_fallback;
      const int n = multi_est_results.est_planner_debug_list.size();
      for (int plan_idx = 0; plan_idx < n; ++plan_idx) {
        if (!fill_fallback &&
            multi_est_results.est_planner_output_list[plan_idx]
                .scheduler_output.is_fallback) {
          continue;
        }
        // Fill planner debug even if the corres. planner branch did not
        // succeed.
        FillEstPlannerDebugProto(
            psmm, *input.planner_input->pose, input.route_output->signal,
            ext_cmd_status, multi_est_results.est_status_list[plan_idx],
            multi_est_results.est_planner_output_list[plan_idx]
                .scheduler_output,
            multi_est_results.est_planner_output_list[plan_idx],
            std::move(multi_est_results.est_planner_debug_list[plan_idx]),
            debug_proto->add_est_planner_debugs());

        *chart_data->add_est_chart_bundles() =
            std::move(multi_est_results.chart_data_list[plan_idx]);
      }
      *debug_proto->mutable_selector_debug() =
          std::move(multi_est_results.selector_debug);
    }
    *debug_proto->mutable_speed_considered_objects_prediction() =
        std::move(async_output->speed_considered_objects_prediction);

    if (is_planner_async_mode) {
      *chart_data->add_est_chart_bundles() =
          std::move(async_output->chart_data);
    }

    if (async_output->latest_low_freq_output != nullptr) {
      if (is_planner_async_mode) {
        // Fill async high_freq planner debug.
        FillHighFreqPlannerDebug(async_output,
                                 debug_proto->mutable_async_high_freq_debug());
      }
      for (const auto& path_point :
           async_output->latest_low_freq_output
               ->st_path_points_global_including_past) {
        *debug_proto->add_st_path_points_global_including_past() = path_point;
      }
    }
  }

  if (!tl_info_map.empty()) {
    // Report candidate traffic light info.
    ReportCandidateTrafficLightInfo(tl_info_map, debug_proto);
    // Report selected traffic light info.
    ReportSelectedTrafficLightInfo(
        tl_info_map,
        est_status.ok() ? prev_target_lane_path : mapping::LanePath(),
        debug_proto->mutable_selected_traffic_light_info());
  }
  on_road_plan_status.ToProto(debug_proto->mutable_planner_status());
  est_status.ToProto(debug_proto->mutable_est_planner_status());
}

}  // namespace

// NOLINTNEXTLINE
PlannerStatus RunCruiseTask(const CruiseTaskInput& input,
                            CruiseTaskOutput* result,
                            PlannerState* planner_state,
                            ExternalCommandStatus* ext_cmd_status,
                            ThreadPool* thread_pool) {
  SCOPED_QTRACE("OnRoadPlanningMainLoop");
  const auto& psmm = *input.planner_input->planner_semantic_map_manager;
  const auto& plan_start_point = input.plan_start_point_info->start_point;
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto& vehicle_params = input.planner_input->vehicle_params;
  const auto& autonomy_state = *input.planner_input->autonomy_state;

  // ----------------------------------------------------------
  // ----------------- Update Planner Params ------------------
  // ----------------------------------------------------------
  auto planner_params = RecreatePlannerParam(input, *ext_cmd_status);

  // ----------------------------------------------------------
  // ----------------------Route Result------------------------
  // ----------------------------------------------------------
  auto route_update_status = UpdateRouteResult(
      input.rerouted, ego_pos, psmm, input.route_output, planner_state);
  if (!route_update_status.ok()) {
    return route_update_status;
  }

  const absl::Cleanup fill_prev_target_lane_path = [&input, &ego_pos,
                                                    &planner_state]() {
    // Check and fill prev_target_lane_path before the end of each iteration.
    FillPrevTargetLanePath(input, ego_pos, planner_state);
  };

  // Clear state when not in auto mode.
  ClearStateInManualDrivingMode(*input.planner_input->autonomy_state,
                                planner_state);

  PlannerDebugProto debug_proto;
  vis::vantage::ChartsDataProto chart_data;
  // Fill reset info.
  debug_proto.set_reset(input.plan_start_point_info->reset);
  debug_proto.set_reset_reason(input.plan_start_point_info->reset_reason);
  // Clear PLC related states on reset.
  if (input.plan_start_point_info->reset &&
      ShouldResetAlcState(input.plan_start_point_info->reset_reason)) {
    planner_state->preferred_lane_path.Clear();
    ext_cmd_status->alc_state = ALC_STANDBY_ENABLE;
    ext_cmd_status->lane_change_command = DriverAction::LC_CMD_NONE;
  }

  // ----------------------------------------------------------
  // -------------------- Route Section -----------------------
  // ----------------------------------------------------------
  auto route_sections_proj_or = ProjectPointToRouteSections(
      psmm, planner_state->prev_route_sections, ego_pos,
      kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
      kDrivePassageKeepBehindLength);
  if (!route_sections_proj_or.ok()) {
    // Clear to force to switch route.
    planner_state->async_planner_state.wait_path_switch_route = false;
    planner_state->async_planner_state.wait_path_switch_route = false;
    planner_state->prev_route_sections.Clear();
    return PlannerStatus(
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED,
        route_sections_proj_or.status().message());
  }
  auto [route_sections_from_start, route_sections_with_behind, ego_pos_proj] =
      std::move(route_sections_proj_or).value();
  planner_state->prev_route_sections = std::move(route_sections_with_behind);
  debug_proto.set_planning_horizon(
      route_sections_from_start.planning_horizon(psmm));

  // ----------------------------------------------------------
  // -------------------- Smooth Reference Line ---------------
  // ----------------------------------------------------------
  const double half_av_width =
      vehicle_params.vehicle_geometry_params().width() * 0.5;
  auto smooth_result_map_or = BuildSmoothedResultMapFromRouteSections(
      psmm, route_sections_from_start, half_av_width,
      std::move(planner_state->smooth_result_map));
  if (smooth_result_map_or.ok()) {
    planner_state->smooth_result_map = std::move(smooth_result_map_or).value();
  }

  // ----------------------------------------------------------
  // ----------- Traffic light info collect  ------------------
  // ----------------------------------------------------------
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto tl_info_collector_output,
      CollectTrafficLightInfo(
          TrafficLightInfoCollectorInput{
              .psmm = input.planner_input->planner_semantic_map_manager.get(),
              .traffic_light_states =
                  input.planner_input->traffic_light_states.get(),
              .route_sections = &route_sections_from_start,
              .plan_time = input.plan_time},
          std::move(planner_state->yellow_light_observations)),
      PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED);
  planner_state->yellow_light_observations =
      std::move(tl_info_collector_output.yellow_light_observations);

  // ----------------------------------------------------------
  // -------------------Scene Reasoning -----------------------
  // ----------------------------------------------------------
  const auto scene_reasoning_output = RunSceneReasoningAndFillDebug(
      psmm, route_sections_from_start, input.route_output->route_navi_info,
      tl_info_collector_output.tl_info_map, *input.object_manager,
      *input.planner_input->prediction, plan_start_point,
      input.planner_input->sensor_fovs.get(), &debug_proto, thread_pool);

  absl::flat_hash_set<std::string> stalled_objects = FillStalledObjects(
      psmm, route_sections_from_start, *input.object_manager,
      debug_proto.stalled_objects(), ext_cmd_status->enable_lc_objects,
      FLAGS_planner_enable_obstacle_lane_change,
      FLAGS_planner_enable_bus_station_stalled_object_filter);

  // ------------------------------------------------------------------
  // ------------------ Previous trajectory planner -------------------
  // ------------------------------------------------------------------
  Future<absl::StatusOr<PreviousTrajectoryPlannerOutput>>
      future_prev_traj_result =
          ScheduleFuture(FLAGS_planner_allow_async_in_main_thread &&
                                 FLAGS_planner_run_prev_traj_async
                             ? thread_pool
                             : nullptr,
                         [&]() {
                           return RunPreviousTrajectoryPlanner(
                               psmm, vehicle_params.vehicle_geometry_params(),
                               vehicle_params.vehicle_drive_params(),
                               planner_params.motion_constraint_params(),
                               *input.time_aligned_prev_traj_points);
                         });

  const absl::Cleanup wait_prev_traj_result = [&future_prev_traj_result]() {
    future_prev_traj_result.Wait();
  };

  // ------------------------------------------------------------------
  // ------------- Collect and keep lc command from teleop ------------
  // ------------------------------------------------------------------
  const auto lc_cmd = ProcessLaneChangeCommands(*input.ext_cmd_queue);
  if (lc_cmd != DriverAction::LC_CMD_NONE) {
    QLOG(INFO) << "Received driver action "
               << DriverAction::LaneChangeCommand_Name(lc_cmd);
    planner_state->async_planner_state.pending_lane_change_command = lc_cmd;
  }

  const auto alc_confirmation = ext_cmd_status->alc_confirmation;
  if (alc_confirmation.has_value()) {
    QLOG(INFO) << "Received auto lane change user confirmation: "
               << *alc_confirmation;
    planner_state->async_planner_state.pending_alc_confirmation =
        alc_confirmation;
  }

  // ------------------------------------------------------------------
  // ---------------- Construct planner-ml context feature ------------
  // ------------------------------------------------------------------

  std::shared_ptr<const ml::ContextFeature> context_feature =
      ConstructPlannerMlContextFeature(input);

  // ----------------------------------------------------------
  // --------------------- Est planner ------------------------
  // ----------------------------------------------------------
  // Configure path look ahead duration for path plan start point.
  // TODO(huaiyuan): Move the calculation of plan_look_ahead_duration to
  // RunMultiTasksCruisePlanner.
  const auto path_look_ahead_duration = GetStPathPlanLookAheadTime(
      *input.plan_start_point_info, *input.planner_input->pose,
      FLAGS_planner_async_low_freq_cycle_iterations *
          absl::Seconds(FLAGS_planner_main_loop_interval),
      *input.previous_trajectory);

  const int max_cruise_iter =
      std::max(FLAGS_planner_max_cruise_async_iterations,
               FLAGS_planner_async_low_freq_cycle_iterations);
  const int max_alcc_iter =
      std::max(FLAGS_planner_max_alcc_async_iterations,
               FLAGS_planner_alcc_async_low_freq_cycle_iterations);

  UpdateAsyncCounter(
      planner_state->async_planner_state.task_transition ||
              planner_state->async_planner_state.secondary_counter.has_value()
          ? max_alcc_iter
          : max_cruise_iter,
      &planner_state->async_planner_state.counter);
  if (planner_state->async_planner_state.secondary_counter.has_value()) {
    UpdateAsyncCounter(
        max_cruise_iter,
        &(*planner_state->async_planner_state.secondary_counter));
  }

  const bool is_standwait =
      autonomy_state.has_assist_state() &&
              autonomy_state.assist_state().has_assist_noa_state() &&
              autonomy_state.assist_state().assist_noa_state().has_state()
          ? autonomy_state.assist_state().assist_noa_state().state() ==
                AssistNoaStateProto::NOA_STATE_STANDWAIT
          : false;

  std::unique_ptr<AsyncPlannerOutput> async_output =
      std::make_unique<AsyncPlannerOutput>();
  std::optional<AccPlannerOutput> acc_planner_output;
  PlannerStatus est_status;
  const MultiTasksCruisePlannerInput multi_task_cruise_planner_input{
      .coordinate_converter = input.coordinate_converter,
      .planner_semantic_map_manager =
          input.planner_input->planner_semantic_map_manager,
      .online_semantic_map = input.planner_input->online_semantic_map,
      .planner_params = &planner_params,
      .vehicle_params = &vehicle_params,
      .rm_output = input.route_output,
      .route_sections_from_start = &route_sections_from_start,
      .start_point_info = input.plan_start_point_info,
      .ego_nearest_lane_id = ego_pos_proj.lane_id,
      .min_path_look_ahead_duration = path_look_ahead_duration,
      .st_traj_mgr = input.st_traj_mgr,
      .object_manager = input.object_manager,
      .stalled_objects = &stalled_objects,
      .scene_reasoning = &scene_reasoning_output.scene_output_proto,
      .ext_cmd_status = ext_cmd_status,
      .time_aligned_prev_traj = input.time_aligned_prev_traj_points,
      .tl_info_map = &tl_info_collector_output.tl_info_map,
      .pose = input.planner_input->pose.get(),
      .chassis = input.planner_input->chassis.get(),
      .autonomy_state = autonomy_state.autonomy_state(),

      .previous_trajectory = input.previous_trajectory,
      .prev_target_lane_path = &planner_state->prev_target_lane_path,
      .prev_route_sections = &planner_state->prev_route_sections,
      .prev_length_along_route = planner_state->prev_length_along_route,
      .prev_max_reach_length = planner_state->prev_max_reach_length,
      .station_anchor = &planner_state->station_anchor,
      .lane_change_state = &planner_state->lane_change_state,
      .prev_lane_path_before_lc = &planner_state->prev_lane_path_before_lc,
      .preferred_lane_path = &planner_state->preferred_lane_path,
      .new_lc_command =
          planner_state->async_planner_state.pending_lane_change_command,
      .alc_confirmation =
          planner_state->async_planner_state.pending_alc_confirmation,
      .smooth_result_map = &planner_state->smooth_result_map,
      .prev_smooth_state = planner_state->prev_smooth_state,
      .parking_brake_release_time = planner_state->parking_brake_release_time,
      .decider_state = &planner_state->decider_state,
      .initializer_state = &planner_state->initializer_state,
      .selected_trajectory_optimizer_state_proto =
          planner_state->selected_trajectory_optimizer_state_proto.has_value()
              ? (&(*planner_state->selected_trajectory_optimizer_state_proto))
              : nullptr,
      .st_planner_object_trajectories =
          &planner_state->st_planner_object_trajectories,
      .selector_state = &planner_state->selector_state,
      .yellow_light_observations = &planner_state->yellow_light_observations,

      .traffic_light_states = input.planner_input->traffic_light_states.get(),
      .log_av_trajectory = input.planner_input->log_av_trajectory.get(),
      .prediction_debug = input.planner_input->prediction_debug.get(),
      .planner_model_pool = input.planner_input->planner_model_pool.get(),
      .real_objects = input.planner_input->real_objects,
      .virtual_objects = input.planner_input->virtual_objects,
      .planner_av_context = input.planner_input->av_context.get(),
      .context_feature = context_feature,
      .is_standwait = is_standwait,
      .prev_collision_warning_request =
          planner_state->prev_collision_warning_request};
  est_status = RunAsyncCruisePlanner(
      multi_task_cruise_planner_input,
      planner_state->assist_plan_state.acc_task(), &acc_planner_output,
      async_output.get(), &planner_state->async_planner_state, thread_pool);

  result->scheduled_async_low_freq = async_output->scheduled_async_low_freq;
  const bool is_planner_async_mode =
      IsPlannerAsync(FLAGS_planner_async_low_freq_cycle_iterations);
  // -----------------------------------------------------------------
  // ------------------- Fill output by acc result -------------------
  // -----------------------------------------------------------------
  if (acc_planner_output.has_value()) {
    UpdateResultByAcc(
        est_status, is_standwait, planner_state->prev_collision_warning_request,
        input.plan_start_point_info->start_point.v(),
        input.planner_params->speed_finder_params().follow_time_headway(),
        input.plan_time, input.planner_input->planner_semantic_map_manager,
        std::move(acc_planner_output.value()), result, planner_state,
        ext_cmd_status);
    return est_status;
  }

  // ------------------------------------------------------------
  // --------------------- Previous Planner----------------------
  // ------------------------------------------------------------
  PlannerStatus on_road_plan_status = OkPlannerStatus();
  std::vector<ApolloTrajectoryPointProto> planned_traj_points;

  // TODO(lidong): Find a method to wait all futures asynchronously.
  auto prev_traj_result = future_prev_traj_result.Get();

  // Clear some fields of planner state, should be filled by the active planner.
  ClearPrevEstPlannerState(planner_state);

  on_road_plan_status = UpdateAebStateToOutput(
      est_status, autonomy_state, is_planner_async_mode, prev_traj_result,
      input.aeb_output, planner_state, async_output.get(), &debug_proto,
      &planned_traj_points);
  if (!on_road_plan_status.ok()) {
    return on_road_plan_status;
  }

  // TODO(renjie): Delete the following kickout logic.
  // Kick out if planner falls back for too many consecutive frames.
  if (planner_state->previous_trajectory_plan_counter >
          FLAGS_prev_traj_planner_max_frame &&
      !OnTestBenchForRsim()) {
    const std::string reason =
        absl::StrCat("Previous-trajectory plan frames: ",
                     planner_state->previous_trajectory_plan_counter,
                     " Latest EstPlanner error: ", est_status.message());
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_PLANNER_TRAJECTORY_INVALID,
        "Use previous-trajectory plan for too many consecutive frames.",
        reason);
    on_road_plan_status = PlannerStatus(est_status.status_code(), reason);
  }

  // ------------------------------------------------------------
  // --------------------- Switch Route -------------------------
  // ------------------------------------------------------------
  UpdateAlternateRoute(est_status, async_output.get(), input.route_output,
                       ext_cmd_status);

  //-----------------------------------------------------------------
  //--------------------- Fill trajectory proto ---------------------
  //-----------------------------------------------------------------
  TurnSignalResult turn_signal_result;
  DrivingStateProto driving_state;
  // TODO(weijun): Delete them.
  mapping::LanePath target_lane_path_from_current;
  LaneChangeStateProto lane_change_state;
  TrajectoryValidationResultProto traj_validation_result;
  if (est_status.ok()) {
    const auto& scheduler_output =
        async_output->latest_low_freq_output->est_output.scheduler_output;
    turn_signal_result = DecideTurnSignal(
        psmm, input.route_output->signal,
        async_output->latest_low_freq_output->selector_state.pre_turn_signal,
        scheduler_output.drive_passage.lane_path(),
        async_output->latest_low_freq_output->est_output.redlight_lane_id,
        scheduler_output.lane_change_state, *ext_cmd_status,
        scheduler_output.drive_passage,
        scheduler_output.av_frenet_box_on_drive_passage,
        TurnSignalResult{.signal = scheduler_output.planner_turn_signal,
                         .reason = scheduler_output.turn_signal_reason},
        *input.planner_input->pose);

    driving_state =
        GetOnRoadDrivingState(vehicle_params.vehicle_geometry_params(),
                              input.plan_start_point_info->full_stop,
                              scheduler_output.drive_passage.lane_path());

    target_lane_path_from_current = scheduler_output.drive_passage.lane_path();

    lane_change_state = scheduler_output.lane_change_state;

    // TODO(weijun): double check
    traj_validation_result = is_planner_async_mode
                                 ? async_output->traj_validation_result
                                 : async_output->latest_low_freq_output
                                       ->est_debug.traj_validation_result;
  }

  //--------------------- Set door decision -------------------------
  const auto door_decision = ComputeDoorDecision(
      input.plan_time, ext_cmd_status->override_door_open,
      planner_state->last_door_override_time,
      driving_state.type() == DrivingStateProto::STOPPED_AT_END_OF_ROUTE,
      FLAGS_planner_open_door_at_route_end,
      FLAGS_planner_door_state_override_waiting_time);

  TrajectoryProto trajectory_info;
  const auto past_points = CreatePastPointsList(
      input.plan_time, *input.previous_trajectory,
      input.plan_start_point_info->reset, kMaxPastPointNum);

  FillTrajectoryProto(input.plan_time, planned_traj_points, past_points,
                      target_lane_path_from_current, lane_change_state,
                      turn_signal_result.signal, door_decision,
                      planner_state->previously_triggered_aeb, driving_state,
                      traj_validation_result, &trajectory_info);

  // -----------------------------------------------------------------
  // ------- Fill planner debug, planner state and charts ------------
  // -----------------------------------------------------------------

  FillPlannerDebugAndChart(input, on_road_plan_status, est_status,
                           turn_signal_result, *ext_cmd_status, psmm,
                           is_planner_async_mode,
                           tl_info_collector_output.tl_info_map,
                           planner_state->prev_target_lane_path,
                           async_output.get(), &debug_proto, &chart_data);

  // -----------------------------------------------------------------
  // --------------------- Update speed limit ------------------------
  // -----------------------------------------------------------------
  if (ext_cmd_status->noa_cruising_speed_limit.has_value()) {
    ext_cmd_status->output.cruising_speed_limit =
        *ext_cmd_status->noa_cruising_speed_limit;
  }

  // -----------------------------------------------------------------
  // ---------------------- Update PLC result ------------------------
  // -----------------------------------------------------------------
  const auto plc_extra_info =
      HandlePlcResult(est_status, input.plan_time, async_output.get(),
                      ext_cmd_status, planner_state, &debug_proto);

  // ------------------------------------------------------
  // ------------------ Report HMI content ----------------
  auto hmi_proto = FillHmiContentProto(
      psmm, target_lane_path_from_current, plc_extra_info.unsafe_object_ids,
      planned_traj_points, est_status, scene_reasoning_output,
      plc_extra_info.plc_target_lane_path_ptr, async_output.get(),
      plc_extra_info.left_solid_boundary, is_planner_async_mode, half_av_width);
  planner_state->prev_collision_warning_request =
      hmi_proto.collision_warning_request();
  // ------------------------------------------------------
  // ------------------ Report ODC content ----------------
  // ------------------------------------------------------
  UpdateRequestHelpLaneChangeByRoute(est_status, async_output.get(),
                                     ext_cmd_status);
  // ------------------------------------------------------------------
  // ------------ Dump planner-ml context feature if configured--------
  // ------------------------------------------------------------------
  if (FLAGS_planner_dumping_ml_data_in_simulation) {
    *debug_proto.mutable_dumped_data()->mutable_context_feature() =
        ContextFeatureToProto(*context_feature);
    const ml::ContextGroundTruthExtractionInput
        context_groundtruth_extraction_input{
            .context_feature = context_feature.get(),
            .log_av_trajectory = input.planner_input->log_av_trajectory.get(),
            .log_prediction = input.planner_input->log_prediction.get(),
            .veh_geom_params = &vehicle_params.vehicle_geometry_params(),
        };
    *debug_proto.mutable_dumped_data()->mutable_context_groundtruth() =
        ml::ExtractContextGroundTruth(context_groundtruth_extraction_input);
  }

  result->trajectory_info = std::move(trajectory_info);
  result->debug_info = std::move(debug_proto);
  result->chart_data = std::move(chart_data);
  result->hmi_content = std::move(hmi_proto);

  DestroyContainerAsyncMarkSource(std::move(async_output),
                                  (QCRAFT_LOC).ToString());

  // NOTE: This condition is added by onboard infra team(mengchunlei);
  if (OnTestBenchForRsim()) {
    // On rsim test, ignore this error
    return OkPlannerStatus();
  }
  return on_road_plan_status;
}  // NOLINT

}  // namespace qcraft::planner
