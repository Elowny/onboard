#include "onboard/planner/plan/alcc_task.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "common/proto/qacc.pb.h"
#include "common/proto/qalc.pb.h"
#include "common/proto/qlcc.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/car_common.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/alcc_turn_signal_decider.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/assist/proto/plc_result.pb.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/proto/captain_net_debug.pb.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/acc/acc_planner_output.h"
#include "onboard/planner/plan/acc/acc_task_internal.h"
#include "onboard/planner/plan/acc/acc_task_output.h"
#include "onboard/planner/plan/async_alcc_planner.h"
#include "onboard/planner/plan/async_planner_output.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/plan/multi_tasks_alcc_planner.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/plan/previous_trajectory_planner.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state_util.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/online_semantic_map_util.h"
#include "onboard/proto/assist_driving_system_state.pb.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

DEFINE_double(planner_alcc_add_on_look_ahead_time, 0.0,
              "extra look ahead time for alcc task.)");

namespace qcraft::planner {

namespace {

void UpdateLanePathStatesOnTransition(
    const std::shared_ptr<AsyncMultiTaskEstOutput>& latest_result,
    const Vec2d& ego_xy, AssistPlanStateProto* assist_plan_state) {
  if (latest_result == nullptr) return;

  const auto& latest_scheduler = latest_result->est_output.scheduler_output;

  const auto crossed_bound_or =
      CrossedBoundary(latest_scheduler.drive_passage, ego_xy);
  const bool crossed_bound = crossed_bound_or.ok() ? *crossed_bound_or : false;

  if (latest_scheduler.lane_change_state.stage() != LCS_NONE &&
      !latest_scheduler.lane_change_state.entered_target_lane() &&
      !latest_scheduler.lane_path_before_lc.IsEmpty() && !crossed_bound) {
    // Changing lane but not entering target lane path, meaning lc just started
    // and currently within lane path before lane change.
    latest_scheduler.lane_path_before_lc.ToProto(
        assist_plan_state->mutable_origin_lane_path());
    *assist_plan_state->mutable_target_lane_path() =
        assist_plan_state->origin_lane_path();
  }
}

void ParseEstPlannerDebugToProto(const PlannerStatus& planner_status,
                                 const EstPlannerOutput& est_output,
                                 EstPlannerDebug est_debug,
                                 EstPlannerDebugProto* est_planner_debug) {
  planner_status.ToProto(est_planner_debug->mutable_planner_status());

  *est_planner_debug->mutable_filtered() =
      std::move(est_debug.filtered_prediction_trajectories);

  *est_planner_debug->mutable_st_planner_object_trajectories() =
      std::move(est_debug.st_planner_object_trajectories);

  ToSchedulerOutputProto(est_output.scheduler_output,
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

void ParseEstPlannerOutputToPlannerState(
    const EstPlannerOutput& est_planner_output,
    const DeciderStateProto& high_freq_decider_state,
    PlannerState* planner_state) {
  const auto& scheduler_output = est_planner_output.scheduler_output;
  const auto& drive_passage = scheduler_output.drive_passage;

  planner_state->prev_target_lane_path =
      drive_passage.extend_lane_path()
          .BeforeLastOccurrenceOfLanePoint(drive_passage.lane_path().back())
          .AfterArclength(
              scheduler_output.av_frenet_box_on_drive_passage.s_min);
  planner_state->lane_change_state = scheduler_output.lane_change_state;
  // Note(jiayu): Consider async mode only.
  planner_state->decider_state = high_freq_decider_state;
  planner_state->initializer_state = est_planner_output.initializer_state;
  planner_state->st_planner_object_trajectories =
      est_planner_output.st_planner_object_trajectories;
  planner_state->prev_traj_end_info = est_planner_output.trajectory_end_info;
  planner_state->selected_trajectory_optimizer_state_proto =
      est_planner_output.trajectory_optimizer_state_proto;
}

void FillInLanePathInAssistState(const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePath& origin_lane_path,
                                 const mapping::LanePath& target_lane_path,
                                 AssistPlanStateProto* assist_plan_state) {
  const auto origin_lane_path_with_behind =
      origin_lane_path.IsEmpty()
          ? origin_lane_path
          : BackwardExtendLanePath(psmm, origin_lane_path,
                                   kDrivePassageKeepBehindLength);

  const auto target_lane_path_with_behind =
      target_lane_path.IsEmpty()
          ? target_lane_path
          : BackwardExtendLanePath(psmm, target_lane_path,
                                   kDrivePassageKeepBehindLength);

  origin_lane_path_with_behind.ToProto(
      assist_plan_state->mutable_origin_lane_path());
  target_lane_path_with_behind.ToProto(
      assist_plan_state->mutable_target_lane_path());
}

void ClearPrevEstPlannerState(PlannerState* planner_state) {
  ResetAlccAssistPlanState(&planner_state->assist_plan_state);

  planner_state->prev_target_lane_path.Clear();
  planner_state->lane_change_state.Clear();

  planner_state->decider_state.Clear();
  planner_state->initializer_state.Clear();

  planner_state->st_planner_object_trajectories.Clear();
  planner_state->prev_traj_end_info = std::nullopt;

  planner_state->prev_low_freq_psmm = nullptr;
  planner_state->prev_online_map_id = kInvalidOnlineMapId;
}

void UpdateOnlineMapDriftBuffer(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_semantic_map,
    const DrivePassage& drive_passage, const Vec2d& ego_xy,
    absl::Time plan_time,
    HistoryBufferAbslTime<PiecewiseLinearFunction<double>>*
        online_map_drift_buffer) {
  std::vector<Vec2d> target_lane_path_points;
  target_lane_path_points.reserve(
      drive_passage.last_real_station_index().value() + 1);
  for (auto i = StationIndex(0); i <= drive_passage.last_real_station_index();
       ++i) {
    const auto& station = drive_passage.station(i);
    if (station.accumulated_s() < 0.0) continue;
    target_lane_path_points.push_back(station.xy());
  }

  if (target_lane_path_points.empty()) return;

  ASSIGN_OR_VOID_RETURN(
      const auto frenet_frame,
      BuildKdTreeFrenetFrame(target_lane_path_points,
                             /*down_sample_raw_points=*/true));

  ASSIGN_OR_VOID_RETURN(
      const auto closest_lane_points,
      FindClosestLanePathPoints(psmm, online_semantic_map, frenet_frame, ego_xy,
                                /*valid_lane_length=*/30.0,
                                /*max_lat_offset_thres=*/0.3,
                                /*avg_lat_offset_thres=*/0.2));

  const int n = closest_lane_points.size();
  std::vector<double> s_vec, l_vec;
  s_vec.reserve(n);
  l_vec.reserve(n);
  for (const auto& pt : closest_lane_points) {
    const auto sl = frenet_frame.XYToSL(pt);
    if (sl.s < frenet_frame.start_s()) continue;
    if (sl.s > frenet_frame.end_s()) break;
    s_vec.push_back(sl.s);
    l_vec.push_back(sl.l);
  }
  if (s_vec.size() >= 2) {
    online_map_drift_buffer->push_back(
        plan_time,
        PiecewiseLinearFunction<double>(std::move(s_vec), std::move(l_vec)));
  }

  constexpr double kOnlineMapDriftStaleTime = 4.0;  // s.
  online_map_drift_buffer->ClearOlderThanRefTime(
      plan_time, absl::Seconds(kOnlineMapDriftStaleTime));
}

HmiContentProto FillHmiContentProto(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& target_lane_path,
    const std::vector<std::string>& unsafe_object_ids,
    const std::vector<ApolloTrajectoryPointProto>& planned_traj_points,
    const PlannerStatus& est_status,
    const mapping::LanePath* plc_target_lane_path_ptr,
    const AsyncPlannerOutput* async_output,
    std::optional<bool> left_solid_boundary, bool is_planner_async_mode) {
  HmiContentInput hmi_content_input{
      .psmm = &psmm,
      .lane_path = &target_lane_path,
      .unsafe_object_ids = &unsafe_object_ids,
      .plc_target_lane_path = plc_target_lane_path_ptr};

  if (!est_status.ok()) {
    return ReportHmiContent(hmi_content_input);
  }

  const auto& scheduler_output =
      async_output->latest_low_freq_output->est_output.scheduler_output;
  hmi_content_input.borrow_lane = scheduler_output.borrow_lane;
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
    hmi_content_input.collision_warning_request =
        est_planner_output.collision_warning_request;
  }

  if (async_output->latest_low_freq_output->nudge_object_info.has_value()) {
    hmi_content_input.nudge_object_info =
        &async_output->latest_low_freq_output->nudge_object_info.value();
  }

  auto hmi_proto = ReportHmiContent(hmi_content_input);

  HmiPathBoundaryProto::BoundaryRenderStyle left_style =
      HmiPathBoundaryProto::STYLE_NORMAL;
  HmiPathBoundaryProto::BoundaryRenderStyle right_style =
      HmiPathBoundaryProto::STYLE_NORMAL;
  if (left_solid_boundary.has_value()) {
    (*left_solid_boundary ? left_style : right_style) =
        HmiPathBoundaryProto::STYLE_WARN;
  }
  *hmi_proto.mutable_path_boundary() = ReportPathBoundaryToHmiContent(
      &scheduler_output.sl_boundary, left_style, right_style);

  hmi_proto.set_online_map_id(
      async_output->latest_low_freq_output->online_map_id);

  return hmi_proto;
}

void SetAlccCustomerRequirement(
    const AlccReqParamsProto& req_proto, double cur_v,
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

void FillOutputByAccResult(const AlccTaskInput& input,
                           const PlannerStatus& alcc_planner_status,
                           AccPlannerOutput acc_planner_output,
                           AlccTaskOutput* output, PlannerState* planner_state,
                           ExternalCommandStatus* ext_cmd_status) {
  if (alcc_planner_status.ok()) {
    const bool is_acc_standwait =
        input.autonomy_state == nullptr ||
                !input.autonomy_state->has_assist_state() ||
                !input.autonomy_state->assist_state().has_assist_acc_state() ||
                !input.autonomy_state->assist_state()
                     .assist_acc_state()
                     .has_state()
            ? false
            : input.autonomy_state->assist_state().assist_acc_state().state() ==
                  AssistAccStateProto::ACC_STATE_STANDWAIT;
    AccTaskOutput acc_task_output;
    FillAccRelatedOutput(
        std::move(acc_planner_output), is_acc_standwait,
        planner_state->prev_collision_warning_request,
        input.plan_start_point_info->start_point.v(),
        input.alcc_params->speed_finder_params().follow_time_headway(),
        input.plan_time, ext_cmd_status->lcc_cruising_speed_limit,
        &acc_task_output);

    *planner_state->assist_plan_state.mutable_acc_task() =
        std::move(acc_task_output.acc_task_proto);
    if (acc_task_output.cruising_speed_limit.has_value()) {
      ext_cmd_status->output.cruising_speed_limit =
          acc_task_output.cruising_speed_limit;
      VLOG(2) << "Output speed limit "
              << acc_task_output.cruising_speed_limit.value();
    }
    ext_cmd_status->acc_state = ACC_OFF;
    ext_cmd_status->lcc_state = LCC_ENABLE;

    output->trajectory_info = std::move(acc_task_output.trajectory_info);
    output->debug_info = std::move(acc_task_output.debug_info);
    output->chart_data = std::move(acc_task_output.chart_data);
    output->hmi_content = std::move(acc_task_output.hmi_content);
  }
  PlannerStatus est_planner_status(
      PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
      "No valid ALCC multi-task est planner result has arrived yet.");
  est_planner_status.ToProto(output->debug_info.mutable_est_planner_status());
  alcc_planner_status.ToProto(output->debug_info.mutable_planner_status());
  planner_state->prev_low_freq_psmm = input.planner_semantic_map_manager;
}

void UpdatePlannerState(
    const AlccTaskInput& input, const PlannerStatus& alcc_planner_status,
    absl::StatusOr<PreviousTrajectoryPlannerOutput> prev_traj_result,
    PlannerState* planner_state, AsyncPlannerOutput* async_output,
    ExternalCommandStatus* ext_cmd_status,
    AlccTurnSignalResult* turn_signal_result,
    mapping::LanePath* target_lane_path_from_current,
    std::vector<ApolloTrajectoryPointProto>* planned_traj_points,
    PlannerDebugProto* debug_proto, PlannerStatus* alcc_task_status

) {
  // Clear some fields of planner state, should be updated if planner succeeds.
  ClearPrevEstPlannerState(planner_state);

  if (alcc_planner_status.ok()) {
    const auto lc_direction =
        async_output->latest_low_freq_output->lc_direction;
    const auto& est_output = async_output->latest_low_freq_output->est_output;
    const auto& scheduler_output = est_output.scheduler_output;
    const auto& drive_passage = scheduler_output.drive_passage;

    const auto& plan_start_point = input.plan_start_point_info->start_point;
    auto ego_xy = Vec2dFromApolloTrajectoryPointProto(plan_start_point);

    // Update online map drift buffer when low freq result retrived only.
    if (FLAGS_planner_enable_online_map_auto_correction &&
        async_output->retrived_low_freq_output != nullptr &&
        input.use_online_semantic_map) {
      UpdateOnlineMapDriftBuffer(*input.planner_semantic_map_manager,
                                 *input.online_semantic_map, drive_passage,
                                 ego_xy, input.plan_time,
                                 &planner_state->online_map_drift_buffer);
    }

    const auto& plc_result = async_output->latest_low_freq_output->plc_result;
    if (plc_result.has_value()) {
      ext_cmd_status->lane_change_command = plc_result->lane_change_command;
    }

    const auto alc_state = async_output->alc_state;
    if (alc_state == QALCState::ALC_STANDBY_ENABLE ||
        alc_state == QALCState::ALC_COMPLETED) {
      ext_cmd_status->lane_change_command = DriverAction::LC_CMD_NONE;
    }
    ext_cmd_status->lcc_state = LCC_ENABLE;
    ext_cmd_status->alc_state = alc_state;
    planner_state->assist_plan_state.set_alc_state(alc_state);
    planner_state->assist_plan_state.set_lc_direction(lc_direction);
    if (async_output->lane_change_target_point.has_value()) {
      *planner_state->assist_plan_state.mutable_lane_change_target_point() =
          std::move(async_output->lane_change_target_point).value();
    } else {
      planner_state->assist_plan_state.clear_lane_change_target_point();
    }

    const auto& high_freq_decider_state = async_output->decider_state;
    ParseEstPlannerOutputToPlannerState(est_output, high_freq_decider_state,
                                        planner_state);

    planner_state->prev_low_freq_psmm =
        async_output->latest_low_freq_output->low_freq_psmm;
    planner_state->prev_online_map_id =
        async_output->latest_low_freq_output->online_map_id;

    FillInLanePathInAssistState(
        async_output->latest_low_freq_output->low_freq_psmm == nullptr
            ? *input.planner_semantic_map_manager
            : *async_output->latest_low_freq_output->low_freq_psmm,
        async_output->origin_lane_path, async_output->target_lane_path,
        &planner_state->assist_plan_state);

    *turn_signal_result = RunAlccTurnSignalDecider(*ext_cmd_status, alc_state);

    *planned_traj_points = std::move(async_output->traj_points);

    *target_lane_path_from_current = drive_passage.lane_path();

    planner_state->previous_trajectory_plan_counter = 0;
    debug_proto->set_active_planner(EST_PLANNER);
  } else if (prev_traj_result.ok()) {
    QEVENT_EVERY_N_SECONDS("zixuan", "alcc_use_prev_traj_planner",
                           /*seconds=*/3.0, [&](QEvent* /*qevent*/) {});
    debug_proto->set_active_planner(PREV_TRAJ_PLANNER);
    *planned_traj_points = std::move(prev_traj_result->trajectory_points);
    planner_state->previous_trajectory_plan_counter++;
  } else {
    planner_state->previous_trajectory_plan_counter = 0;
    const std::string reason = absl::StrCat(
        "No valid plan for ALCC.\n  Est planner error: ",
        alcc_planner_status.message(),
        "  Previous-trajectory error: ", prev_traj_result.status().ToString());
    *alcc_task_status =
        PlannerStatus(alcc_planner_status.status_code(), reason);
  }

  const int max_prev_planner_frame =
      CeilToInt(FLAGS_planner_alcc_prev_traj_planner_max_time /
                FLAGS_planner_main_loop_interval);
  if (planner_state->previous_trajectory_plan_counter >
      max_prev_planner_frame) {
    const std::string reason = absl::StrCat(
        "ALCC uses previous-trajectory plan for too many consecutive frames. "
        "Previous-trajectory plan frames: ",
        planner_state->previous_trajectory_plan_counter,
        " Latest EstPlanner error: ", alcc_planner_status.message());
    *alcc_task_status =
        PlannerStatus(alcc_planner_status.status_code(), reason);
  }
}

void ReportPlcStatus(PlcInternalStatus plc_status,
                     DriverAction::LaneChangeCommand lc_cmd,
                     const PlannerStatus& alcc_planner_status) {
  if (plc_status == PlcInternalStatus::BRANCH_NOT_FOUND) {
    if (lc_cmd == DriverAction::LC_CMD_CANCEL) {
      QLOG(ERROR) << "ALCC lane change return failed: origin branch not found.";
      QRUNEVENT_WITH_ENUM_NOTICE(
          QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
          QRunEvent::PLC_REJECT_RETURN_ORIGIN_UNAVAILABLE);
    } else {
      QLOG(ERROR) << "ALCC lane change failed: target branch not found.";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_REJECT_TARGET_NOT_FOUND);
    }
  } else if (!alcc_planner_status.ok()) {
    QLOG(ERROR) << "ALCC lane change failed: all branches failed.";
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_FAIL_ALL_BRANCHES);
  } else {
    if (lc_cmd == DriverAction::LC_CMD_CANCEL) {
      QLOG(ERROR)
          << "ALCC lane change return failed: planner internal failure.";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_FAIL_ALL_BRANCHES);
    } else {
      QLOG(ERROR) << "ALCC lane change failed: planner internal failure.";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_FAIL_INTERNAL);
    }
  }
}

struct FillDebugOutput {
  vis::vantage::ChartsDataProto chart_data;
  std::vector<std::string> unsafe_object_ids;
  mapping::LanePath* plc_target_lane_path_ptr = nullptr;
  std::optional<bool> left_solid_boundary = std::nullopt;
};
FillDebugOutput FillDebugAndCharts(
    const AlccTaskInput& input, const PlannerStatus& alcc_planner_status,
    QALCState prev_alc_state, const AlccTurnSignalResult& turn_signal_result,
    AsyncPlannerOutput* async_output, ExternalCommandStatus* ext_cmd_status,
    PlannerDebugProto* debug_proto) {
  FillDebugOutput output;

  // Fill low freq planner debug.
  if (async_output->retrived_low_freq_output != nullptr) {
    auto& multi_est_results = *async_output->retrived_low_freq_output;
    debug_proto->set_path_start_relative_index(
        multi_est_results.path_start_relative_index);

    for (int plan_idx = 0;
         plan_idx < multi_est_results.est_planner_debug_list.size();
         ++plan_idx) {
      // Fill planner debug even if the corres. planner branch did not
      // succeed.
      ParseEstPlannerDebugToProto(
          multi_est_results.est_status_list[plan_idx],
          multi_est_results.est_planner_output_list[plan_idx],
          std::move(multi_est_results.est_planner_debug_list[plan_idx]),
          debug_proto->add_est_planner_debugs());

      *output.chart_data.add_est_chart_bundles() =
          std::move(multi_est_results.chart_data_list[plan_idx]);
    }
  }

  const bool is_planner_async_mode =
      IsPlannerAsync(FLAGS_planner_alcc_async_low_freq_cycle_iterations);
  if (IsNonEmptyPlannerResult(alcc_planner_status)) {
    if (is_planner_async_mode) {
      *output.chart_data.add_est_chart_bundles() =
          std::move(async_output->chart_data);
    }
    *debug_proto->mutable_speed_considered_objects_prediction() =
        std::move(async_output->speed_considered_objects_prediction);

    // Fill async high_freq planner debug.
    if (async_output->latest_low_freq_output != nullptr) {
      if (is_planner_async_mode) {
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

  // Fill reset info.
  debug_proto->set_reset(input.plan_start_point_info->reset);
  debug_proto->set_reset_reason(input.plan_start_point_info->reset_reason);
  debug_proto->set_turn_signal_reason_enum(turn_signal_result.reason);

  // -----------------------------------------------------------------
  // ---------------------- Update PLC result ------------------------
  // -----------------------------------------------------------------

  if (async_output->latest_low_freq_output != nullptr &&
      async_output->latest_low_freq_output->plc_result.has_value()) {
    const auto& plc_result = async_output->latest_low_freq_output->plc_result;

    const auto plc_status = plc_result->status;
    const auto lc_cmd = plc_result->lane_change_command;
    const bool branch_invalid =
        plc_status == PlcInternalStatus::BRANCH_NOT_FOUND ||
        plc_status == PlcInternalStatus::BRANCH_FAILED_INTERNAL;

    // Report plc signals.
    if (!alcc_planner_status.ok() || branch_invalid) {
      ReportPlcStatus(plc_status, lc_cmd, alcc_planner_status);
    } else {
      ReportPlcEventSignal(prev_alc_state, ext_cmd_status->alc_state, lc_cmd,
                           plc_status);
      if (plc_status == PlcInternalStatus::SOLID_BOUNDARY ||
          plc_status == PlcInternalStatus::UNSAFE_OBJECT) {
        output.unsafe_object_ids = plc_result->unsafe_object_ids;
        output.left_solid_boundary = plc_result->left_solid_boundary;
        if (plc_status == PlcInternalStatus::UNSAFE_OBJECT) {
          output.plc_target_lane_path_ptr = &async_output->target_lane_path;
        }
      }
    }

    // Update plc states.
    const bool lc_prepare = ext_cmd_status->alc_state == QALCState::ALC_PREPARE;
    if (!alcc_planner_status.ok() ||
        (branch_invalid && lc_prepare &&
         !FLAGS_planner_lc_prepare_when_branch_invalid)) {
      ext_cmd_status->lane_change_command = DriverAction::LC_CMD_NONE;
      ext_cmd_status->plc_prepare_start_time = std::nullopt;
      ext_cmd_status->alc_state = QALCState::ALC_STANDBY_ENABLE;
    } else {
      if (lc_prepare) {
        if (!ext_cmd_status->plc_prepare_start_time.has_value()) {
          ext_cmd_status->plc_prepare_start_time = input.plan_time;
        }
      } else if (ext_cmd_status->plc_prepare_start_time.has_value()) {
        ext_cmd_status->plc_prepare_start_time = std::nullopt;
      }
    }

    debug_proto->set_plc_status(plc_status);
  }

  return output;
}

void UpdateAsyncState(const ApolloTrajectoryPointProto& start_point,
                      PlannerState* planner_state) {
  const int max_cruise_iter =
      std::max(FLAGS_planner_max_cruise_async_iterations,
               FLAGS_planner_async_low_freq_cycle_iterations);
  const int max_alcc_iter =
      std::max(FLAGS_planner_max_alcc_async_iterations,
               FLAGS_planner_alcc_async_low_freq_cycle_iterations);

  UpdateAsyncCounter(
      planner_state->async_planner_state.task_transition ||
              planner_state->async_planner_state.secondary_counter.has_value()
          ? max_cruise_iter
          : max_alcc_iter,
      &planner_state->async_planner_state.counter);
  if (planner_state->async_planner_state.secondary_counter.has_value()) {
    UpdateAsyncCounter(
        max_alcc_iter,
        &(*planner_state->async_planner_state.secondary_counter));

    // Update lane path states before scheduling an alcc low-freq module during
    // task transition, since the former ones came from noa result and might
    // correspond to a lane changing scheduler.
    if (ShouldRunLowFreqModule(
            planner_state->async_planner_state.secondary_counter.value())) {
      UpdateLanePathStatesOnTransition(
          planner_state->async_planner_state.latest_multi_task_est_result,
          Vec2dFromApolloTrajectoryPointProto(start_point),
          &planner_state->assist_plan_state);
    }
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-function-size,readability-function-cognitive-complexity)
PlannerStatus RunAlccTask(const AlccTaskInput& input, AlccTaskOutput* output,
                          PlannerState* planner_state,
                          ExternalCommandStatus* ext_cmd_status,
                          ThreadPool* thread_pool) {
  SCOPED_QTRACE("Async ALCC Task");

  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& vehicle_params = *input.vehicle_params;

  ext_cmd_status->lcc_state = QLCCState::LCC_DISABLE;
  // ----------------------------------------------------------
  // ----------------- Update Planner Params ------------------
  // ----------------------------------------------------------
  // Update planner params in alcc task.
  // TODO(jiayu): Replace planner params with alcc params later.
  auto alcc_params = *QCHECK_NOTNULL(input.alcc_params);
  UpdateFollowHeadwayTimeAccordHmi(alcc_params.mutable_speed_finder_params(),
                                   ext_cmd_status->following_distance_level);
  SetAlccCustomerRequirement(alcc_params.alcc_req_params(),
                             input.plan_start_point_info->start_point.v(),
                             alcc_params.mutable_speed_finder_params(),
                             alcc_params.mutable_motion_constraint_params());

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
                               alcc_params.motion_constraint_params(),
                               *input.time_aligned_prev_traj_points);
                         });

  const absl::Cleanup wait_prev_traj_result = [&future_prev_traj_result]() {
    future_prev_traj_result.Wait();
  };

  // ----------------------------------------------------------
  // --------------------- Est planner ------------------------
  // ----------------------------------------------------------
  const auto path_look_ahead_time = GetStPathPlanLookAheadTime(
      *input.plan_start_point_info, *input.pose,
      FLAGS_planner_alcc_async_low_freq_cycle_iterations *
              absl::Seconds(FLAGS_planner_main_loop_interval) +
          absl::Seconds(FLAGS_planner_alcc_add_on_look_ahead_time),
      planner_state->previous_trajectory);

  const auto prev_alc_state = planner_state->assist_plan_state.alc_state();
  const auto new_lc_cmd = ProcessLaneChangeCommands(*input.ext_cmd_queue);
  if (new_lc_cmd != DriverAction::LC_CMD_NONE) {
    QLOG(INFO) << "Received driver action "
               << DriverAction::LaneChangeCommand_Name(new_lc_cmd);
    planner_state->async_planner_state.pending_lane_change_command = new_lc_cmd;
  }
  UpdateAsyncState(input.plan_start_point_info->start_point, planner_state);

  const auto& autonomy_state = *input.autonomy_state;
  const bool is_standwait =
      autonomy_state.has_assist_state() &&
              autonomy_state.assist_state().has_assist_lcc_state() &&
              autonomy_state.assist_state().assist_lcc_state().has_state()
          ? autonomy_state.assist_state().assist_lcc_state().state() ==
                AssistLccStateProto::LCC_STATE_STANDWAIT
          : false;

  std::unique_ptr<AsyncPlannerOutput> async_output =
      std::make_unique<AsyncPlannerOutput>();
  std::optional<AccPlannerOutput> acc_planner_output;
  auto alcc_planner_status = RunAsyncAlccPlanner(
      MultiTasksAlccPlannerInput{
          .planner_semantic_map_manager = input.planner_semantic_map_manager,
          .pose = input.pose,
          .chassis = input.chassis,
          .autonomy_state = input.autonomy_state,
          .alcc_params = &alcc_params,
          .acc_params = input.acc_params,
          .vehicle_params = input.vehicle_params,
          .plan_start_point_info = input.plan_start_point_info,
          .plan_time = input.plan_time,
          .st_traj_mgr = input.st_traj_mgr,
          .object_manager = input.object_manager,
          .time_aligned_prev_traj_points = input.time_aligned_prev_traj_points,
          .log_av_trajectory = input.log_av_trajectory,
          .online_semantic_map = input.online_semantic_map,
          .path_look_ahead_time = path_look_ahead_time,
          .assist_plan_state = &planner_state->assist_plan_state,
          .parking_brake_release_time =
              planner_state->parking_brake_release_time,
          .decider_state = &planner_state->decider_state,
          .initializer_state = &planner_state->initializer_state,
          .selected_trajectory_optimizer_state_proto =
              (planner_state->selected_trajectory_optimizer_state_proto
                   .has_value())
                  ? (&(*planner_state
                            ->selected_trajectory_optimizer_state_proto))
                  : nullptr,
          .st_planner_object_trajectories =
              &planner_state->st_planner_object_trajectories,
          .previous_trajectory = &planner_state->previous_trajectory,
          .ext_cmd_status = ext_cmd_status,
          .prev_low_freq_psmm = planner_state->prev_low_freq_psmm,
          .use_online_semantic_map = input.use_online_semantic_map,
          .is_engage_steer_only = IsAutoDriveToAutoSteerOnly(
              planner_state->previous_autonomy_state.autonomy_state(),
              input.autonomy_state->autonomy_state()),
          .new_lc_command =
              planner_state->async_planner_state.pending_lane_change_command,
          .av_context = input.av_context,
          .online_map_drift_buffer = &planner_state->online_map_drift_buffer,
          .is_standwait = is_standwait,
          .prev_collision_warning_request =
              planner_state->prev_collision_warning_request},
      planner_state->assist_plan_state.acc_task(), &acc_planner_output,
      async_output.get(), &planner_state->async_planner_state, thread_pool);
  output->scheduled_async_low_freq = async_output->scheduled_async_low_freq;

  // -----------------------------------------------------------------
  // ------------------- Fill output by acc result -------------------
  // -----------------------------------------------------------------
  if (acc_planner_output.has_value()) {
    FillOutputByAccResult(input, alcc_planner_status,
                          std::move(acc_planner_output.value()), output,
                          planner_state, ext_cmd_status);

    return alcc_planner_status;
  }

  auto prev_traj_result = future_prev_traj_result.Get();

  // -----------------------------------------------------------------
  // --------------------- Update planner state ------------------------
  // -----------------------------------------------------------------
  AlccTurnSignalResult turn_signal_result;
  mapping::LanePath target_lane_path_from_current;
  std::vector<ApolloTrajectoryPointProto> planned_traj_points;
  PlannerDebugProto debug_proto;
  PlannerStatus alcc_task_status = OkPlannerStatus();
  UpdatePlannerState(input, alcc_planner_status, std::move(prev_traj_result),
                     planner_state, async_output.get(), ext_cmd_status,
                     &turn_signal_result, &target_lane_path_from_current,
                     &planned_traj_points, &debug_proto, &alcc_task_status);

  //-----------------------------------------------------------------
  //--------------------- Fill trajectory proto ---------------------
  //-----------------------------------------------------------------
  TrajectoryValidationResultProto traj_validation_result;
  if (alcc_planner_status.ok()) {
    traj_validation_result = std::move(async_output->traj_validation_result);
  }

  TrajectoryProto trajectory_info;
  const auto past_points = CreatePastPointsList(
      input.plan_time, planner_state->previous_trajectory,
      input.plan_start_point_info->reset, kMaxPastPointNum);
  FillTrajectoryProto(input.plan_time, planned_traj_points, past_points,
                      target_lane_path_from_current,
                      planner_state->lane_change_state,
                      turn_signal_result.signal, DoorDecision(),
                      /*is_aeb_triggered=*/false, DrivingStateProto(),
                      traj_validation_result, &trajectory_info);

  // -----------------------------------------------------------------
  // --------------------- Fill planner debug  and charts  -----------
  // -----------------------------------------------------------------
  FillDebugOutput fill_debug_output = FillDebugAndCharts(
      input, alcc_planner_status, prev_alc_state, turn_signal_result,
      async_output.get(), ext_cmd_status, &debug_proto);

  // -----------------------------------------------------------------
  // ----------------------- Report HMI content ----------------------
  // -----------------------------------------------------------------
  auto hmi_content = FillHmiContentProto(
      psmm, target_lane_path_from_current, fill_debug_output.unsafe_object_ids,
      planned_traj_points, alcc_planner_status,
      fill_debug_output.plc_target_lane_path_ptr, async_output.get(),
      fill_debug_output.left_solid_boundary,
      IsPlannerAsync(FLAGS_planner_alcc_async_low_freq_cycle_iterations));
  planner_state->prev_collision_warning_request =
      hmi_content.collision_warning_request();

  // -----------------------------------------------------------------
  // ------------------ Update external command info -----------------
  // -----------------------------------------------------------------
  if (ext_cmd_status->lcc_cruising_speed_limit.has_value()) {
    ext_cmd_status->output.cruising_speed_limit =
        *ext_cmd_status->lcc_cruising_speed_limit;
  }

  alcc_task_status.ToProto(debug_proto.mutable_planner_status());
  alcc_planner_status.ToProto(debug_proto.mutable_est_planner_status());

  output->trajectory_info = std::move(trajectory_info);
  output->debug_info = std::move(debug_proto);
  output->chart_data = std::move(fill_debug_output.chart_data);
  output->hmi_content = std::move(hmi_content);

  DestroyContainerAsyncMarkSource(std::move(async_output),
                                  (QCRAFT_LOC).ToString());

  // NOTE: This condition is added by onboard infra team(mengchunlei);
  if (OnTestBenchForRsim()) {
    // On rsim test, ignore this error
    return OkPlannerStatus();
  }
  return alcc_task_status;
}

}  // namespace qcraft::planner
