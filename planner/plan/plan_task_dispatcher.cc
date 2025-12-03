#include "onboard/planner/plan/plan_task_dispatcher.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <optional>
#include <ostream>
#include <queue>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/qacc.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/assist_util.h"
#include "onboard/planner/assist/planner_odc_generator.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/assist/tja_internal.h"
#include "onboard/planner/assist/tja_state.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/emergency_stop.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/mapless/mapless_task.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/motion_state_filter.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/predicted_motion_filter.h"
#include "onboard/planner/object/reflected_object_in_proximity_filter.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/planner/object/unknown_roadway_position_object_filter.h"
#include "onboard/planner/plan/acc/acc_task.h"
#include "onboard/planner/plan/acc/acc_task_input.h"
#include "onboard/planner/plan/acc/acc_task_output.h"
#include "onboard/planner/plan/aeb_planner.h"
#include "onboard/planner/plan/alcc_task.h"
#include "onboard/planner/plan/apa_parking_task.h"
#include "onboard/planner/plan/cruise_task.h"
#include "onboard/planner/plan/free_drive_task.h"
#include "onboard/planner/plan/parking_task.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/plan_task_helper.h"
#include "onboard/planner/plan/plan_task_switcher.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"
#include "onboard/planner/plan/uturn_task.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/proto/route_external_command.pb.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/proto/scheduler.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/assist_driving_system_state.pb.h"
#include "onboard/proto/assist_state.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/parking_spot_finder.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/perception/parking/parking_freespace.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/remote_assist_common.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

namespace {
struct OnlineMapInfo {
  std::shared_ptr<PlannerSemanticMapManager> psmm = nullptr;
  std::shared_ptr<const mapping::OnlineSemanticMapProto> online_semantic_map =
      nullptr;
};

absl::StatusOr<mapping::LanePath> AlignLanePathToPose(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const PoseProto& ego_pose) {
  ASSIGN_OR_RETURN(const auto checked_lane_path,
                   TrimTrailingNotFoundLanes(psmm, lane_path));
  const auto points = SampleLanePathPoints(psmm, checked_lane_path);
  ASSIGN_OR_RETURN(const auto ff,
                   BuildBruteForceFrenetFrame(points,
                                              /*down_sample_raw_points=*/true));

  const FrenetCoordinate sl =
      ff.XYToSL(Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));

  return lane_path.AfterArclength(sl.s);
}

void SetFreespaceResetReason(const PlanStartPointInfo& plan_start_point_info,
                             bool freespace_reset,
                             ResetReasonProto::Reason freespace_reset_reason,
                             PlannerDebugProto* output) {
  const bool reset = plan_start_point_info.reset;
  output->set_reset(reset || freespace_reset);
  if (reset) {
    output->set_reset_reason(plan_start_point_info.reset_reason);
  }
  if (freespace_reset) {
    output->set_reset_reason(freespace_reset_reason);
  }
}

bool UpdatePlanTasksQueueWhenUturnAhead(
    bool planner_enable_runtime_uturn_task,
    const PlannerSemanticMapManager& psmm, const PoseProto& ego_pose,
    const PlannerStateProto& prev_planner_state,
    const RouteManagerOutput& route_output,
    const mapping::LanePath& prev_target_lane_path,
    std::deque<PlanTask>* plan_task_queue) {
  // Create new tasks at run time: uturn.
  if (planner_enable_runtime_uturn_task &&
      plan_task_queue->front().type() == ON_ROAD_CRUISE_PLAN) {
    const auto aligned_lane_path_or =
        AlignLanePathToPose(psmm, prev_target_lane_path, ego_pose);

    std::optional<TrajectoryEndInfoProto> prev_traj_end_info = std::nullopt;
    if (prev_planner_state.has_previous_trajectory_end_info()) {
      prev_traj_end_info = prev_planner_state.previous_trajectory_end_info();
    }
    if (aligned_lane_path_or.ok()) {
      auto new_tasks_or = CreateUturnTask(
          psmm, ego_pose, *aligned_lane_path_or,
          route_output.route_sections_from_current.destination(),
          prev_traj_end_info, prev_planner_state.previous_trajectory());

      if (new_tasks_or.ok()) {
        plan_task_queue->pop_front();
        auto new_tasks = std::move(new_tasks_or).value();
        for (auto it = new_tasks.rbegin(); it != new_tasks.rend(); it++) {
          plan_task_queue->emplace_front(std::move(*it));
        }
        // QEvent for uturn.
        QEVENT("fengzhuang", "uturn_task_created", [&](QEvent*) {});
        return true;
      }
    }
  }
  return false;
}

bool UpdatePlanTasksQueueByOutOfBlockedRequest(
    const PlannerSemanticMapManager& psmm, const PoseProto& ego_pose,
    const std::queue<OutOfBlockedRoadRequestProto>&
        pending_out_of_blocked_road_requests,
    const VehicleGeometryParamsProto& veh_geo_params,
    std::deque<PlanTask>* plan_task_queue) {
  if (plan_task_queue->front().type() == ON_ROAD_CRUISE_PLAN &&
      !pending_out_of_blocked_road_requests.empty()) {
    const auto& blocked_road_request =
        pending_out_of_blocked_road_requests.back();
    if (blocked_road_request.has_enable() && blocked_road_request.enable()) {
      auto blocked_task_or =
          CreateBlockedRoadTask(psmm, ego_pose, veh_geo_params);

      if (blocked_task_or.ok()) {
        plan_task_queue->emplace_front(std::move(blocked_task_or).value());
        return true;
      }
    }
  }
  return false;
}

PlannerDebugProto SimplifyPlannerDebugProto(PlannerDebugProto&& proto) {
  PlannerDebugProto new_proto;

  if (proto.has_speed_considered_objects_prediction()) {
    new_proto.mutable_speed_considered_objects_prediction()->Swap(
        proto.mutable_speed_considered_objects_prediction());
  }

  if (proto.has_turn_signal_reason_enum()) {
    new_proto.set_turn_signal_reason_enum(proto.turn_signal_reason_enum());
  }

  if (proto.has_selected_traffic_light_info()) {
    new_proto.mutable_selected_traffic_light_info()->Swap(
        proto.mutable_selected_traffic_light_info());
  }

  if (proto.has_active_planner()) {
    new_proto.set_active_planner(proto.active_planner());
  }

  if (proto.has_reset()) {
    new_proto.set_reset(proto.reset());
  }

  if (proto.has_reset_reason()) {
    new_proto.set_reset_reason(proto.reset_reason());
  }

  if (proto.has_est_planner_status()) {
    new_proto.mutable_est_planner_status()->Swap(
        proto.mutable_est_planner_status());
  }

  if (!proto.est_planner_debugs().empty()) {
    new_proto.mutable_est_planner_debugs()->Reserve(
        proto.est_planner_debugs_size());
    for (auto& est_debug : *proto.mutable_est_planner_debugs()) {
      auto* new_est_debug = new_proto.add_est_planner_debugs();
      new_est_debug->mutable_planner_status()->Swap(
          est_debug.mutable_planner_status());
    }
  }

  if (proto.async_high_freq_debug().has_planner_status()) {
    new_proto.mutable_async_high_freq_debug()->mutable_planner_status()->Swap(
        proto.mutable_async_high_freq_debug()->mutable_planner_status());
  }

  if (!proto.stalled_objects().empty()) {
    new_proto.mutable_stalled_objects()->Reserve(proto.stalled_objects_size());
    for (auto& stalled_object : *proto.mutable_stalled_objects()) {
      *new_proto.add_stalled_objects() = std::move(stalled_object);
    }
  }

  return new_proto;
}

void FillPlannerDebugProto(PlannerDebugProto planner_debug, bool is_simplify,
                           PlannerDebugProto* proto) {
  if (is_simplify) {
    *proto = SimplifyPlannerDebugProto(std::move(planner_debug));
  } else {
    *proto = std::move(planner_debug);
  }
}

void ParseRouteRelatedOdcToExternalCommandStatus(
    const RouteRelatedOdc& route_related_odc,
    ExternalCommandOutput* ext_cmd_output) {
  ext_cmd_output->distance_to_toll = route_related_odc.distance_to_toll;
  ext_cmd_output->distance_to_traffic_light =
      route_related_odc.distance_to_traffic_light;
}

void ParseOdcToExternalCommandStatus(const OdcGeneratorOutput& odc,
                                     ExternalCommandOutput* ext_cmd_output) {
  ext_cmd_output->current_lane_average_curvature_radius =
      odc.current_lane_average_curvature_radius;
  ext_cmd_output->is_av_overlap_boundary = odc.is_av_overlap_boundary;
  ext_cmd_output->is_valid_both_side_boundary = odc.is_valid_both_side_boundary;
  ext_cmd_output->is_av_in_emergency_lane = odc.is_av_in_emergency_lane;
  ext_cmd_output->current_lane_width = odc.current_lane_width;
  ext_cmd_output->current_lane_length = odc.current_lane_length;
  ext_cmd_output->lane_path_lost = odc.lane_path_lost;
  ext_cmd_output->is_acc_engage_only = odc.is_acc_engage_only;
}

absl::StatusOr<OdcGeneratorOutput> GeneratePlannerOdcOnRoad(
    const PlannerSemanticMapManager& psmm, const PoseProto& pose,
    const VehicleGeometryParamsProto& vehicle_geom,
    const PlannerStatus& planner_status,
    const mapping::LanePathProto& lane_path_proto, bool is_lane_changing) {
  const auto target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), lane_path_proto);

  const OdcGeneratorInput odc_input{.psmm = &psmm,
                                    .pose = &pose,
                                    .vehicle_geom = &vehicle_geom,
                                    .target_lane_path = &target_lane_path,
                                    .is_lane_changing = is_lane_changing};

  return planner_status.ok() && !target_lane_path.IsEmpty()
             ? GeneratePlannerOdcByLanePath(odc_input)
             : GeneratePlannerOdcByPose(odc_input);
}

inline bool IsLaneChangeStage(LaneChangeStage stage) {
  switch (stage) {
    case LCS_NONE:
    case LCS_WAITING:
      return false;
    case LCS_EXECUTING:
    case LCS_PAUSE:
    case LCS_RETURN:
      return true;
  }
}

bool ShouldRunAebPlanner(PlanTaskType type) {
  // TODO(hang, bo): change this when accomplish new aeb planner.
  switch (type) {
    case ON_ROAD_CRUISE_PLAN:
      return true;
    case OFF_ROAD_PLAN:
    case UTURN_PLAN:
    case BLOCKED_PLAN:
    case ALCC_PLAN:
    case ACC_PLAN:
    case MAPLESS_NOA:
    case APA_PLAN:
      return false;
  }
}

void PreprocessOnlineMapUsingPrevLanePath(
    const std::vector<Vec2d>& prev_lane_path_points, const PoseProto& pose,
    OnlineMapInfo* online_map_info) {
  if (prev_lane_path_points.empty()) return;

  auto new_online_map_or = BuildOnlineMapFromPrevLanePath(
      *online_map_info->psmm, *online_map_info->online_semantic_map,
      prev_lane_path_points, Vec2dFromPoseProto(pose));
  if (!new_online_map_or.ok()) {
    QLOG(INFO) << "PreprocessOnlineMapUsingPrevLanePath: "
               << new_online_map_or.status();
    return;
  }

  ASSIGN_OR_VOID_RETURN(online_map_info->psmm,
                        BuildOnlineMapPsmm(*new_online_map_or));
  online_map_info->online_semantic_map =
      std::make_shared<const mapping::OnlineSemanticMapProto>(
          std::move(new_online_map_or).value());
}

absl::StatusOr<OnlineMapInfo> PreprocessAlccOnlineMap(
    const AutonomyStateProto& autonomy_state, const PoseProto& pose,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::shared_ptr<PlannerSemanticMapManager>& psmm,
    const std::shared_ptr<const mapping::OnlineSemanticMapProto>& origin_map,
    const std::vector<Vec2d>& prev_lane_path_points, TjaState* tja_state) {
  OnlineMapInfo online_map_info;
  online_map_info.psmm = psmm;
  online_map_info.online_semantic_map = origin_map;

  if (origin_map == nullptr) {
    return online_map_info;
  }

  if (!FLAGS_planner_enable_tja_in_alcc_task ||
      (IsOnboardMode() &&
       !IsLateralAutonomousDrivingMode(autonomy_state.autonomy_state())) ||
      !ShouldUseTjaOnlineSemanticMap(pose, *psmm, tja_state)) {
    tja_state->Reset();

    // TODO(zixuan): Add precondition.
    PreprocessOnlineMapUsingPrevLanePath(prev_lane_path_points, pose,
                                         &online_map_info);

    return online_map_info;
  }

  auto online_semantic_map_or = ActivateOnlineSemanticMap(
      pose, st_traj_mgr, vehicle_geom_params, *psmm, *origin_map, tja_state);
  if (online_semantic_map_or.ok()) {
    if (!tja_state->planner_use_tja_map) {
      QEVENT_EVERY_N_SECONDS("chengyang", "enter_tja",
                             /*seconds=*/5.0, [&](QEvent*) {});
    }
    tja_state->planner_use_tja_map = true;
    QLOG(INFO) << "Using Tja online semantic map in alcc task.";
    online_map_info.online_semantic_map =
        std::move(online_semantic_map_or).value();
    ASSIGN_OR_RETURN(online_map_info.psmm,
                     BuildOnlineMapPsmm(*online_map_info.online_semantic_map));
  } else {
    tja_state->Reset();
    QLOG(WARNING) << "When activate tja map, "
                  << online_semantic_map_or.status().message();

    PreprocessOnlineMapUsingPrevLanePath(prev_lane_path_points, pose,
                                         &online_map_info);
  }

  return online_map_info;
}

PlannerStatus UpdatePlannerSemanticMapManagerByPlanTask(
    const std::shared_ptr<const mapping::OnlineSemanticMapProto>&
        online_semantic_map,
    const std::shared_ptr<const LocalizationTransformProto>&
        localization_transform,
    PlanTaskType task_type, std::shared_ptr<PlannerSemanticMapManager>* psmm,
    bool is_l4_mode, ThreadPool* thread_pool) {
  if (!IsHdMapBasedTask(task_type)) {
    if (online_semantic_map != nullptr) {
      auto online_psmm_or =
          BuildOnlineMapPsmm(*online_semantic_map, thread_pool);
      if (!online_psmm_or.ok()) {
        return ClassifyTaskErrorToPlannerStatus(
            task_type,
            absl::StrFormat("Build vision map failed for %s.",
                            PlanTaskType_Name(task_type)),
            is_l4_mode);
      }
      *psmm = std::move(online_psmm_or).value();
    }
  } else {
    if (localization_transform == nullptr) {
      return ClassifyTaskErrorToPlannerStatus(
          task_type,
          absl::StrCat(
              "localization transform is not available. Refuse to plan in ",
              PlanTaskType_Name(task_type)),
          is_l4_mode);
    }
    const double max_lt_delay = FLAGS_planner_max_localization_transform_delay;
    const double now = ToUnixDoubleSeconds(Clock::Now());
    if (localization_transform->header().timestamp() <
        (now - max_lt_delay) * 1e6) {
      const std::string error_msg = absl::StrFormat(
          "localization transform is stale for %.3fs. The max staleness "
          "defined by flag planner_max_localization_transform_delay is %.3f. "
          "You may increase the flag to tolerate this problem.",
          now - localization_transform->header().timestamp() * 1e-6,
          max_lt_delay);
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_LOCALIZATION_TRANSFORM_STALE,
                        "Check Localization transform state: Stale", error_msg);
      return ClassifyTaskErrorToPlannerStatus(task_type, error_msg, is_l4_mode);
    }
  }

  if (*psmm == nullptr) {
    return ClassifyTaskErrorToPlannerStatus(
        task_type,
        absl::StrFormat("No valid map available for %s.",
                        PlanTaskType_Name(task_type)),
        is_l4_mode);
  }
  return OkPlannerStatus();
}

void UpdatePlannerStateInfoByCurrentTask(
    const PlannerSemanticMapManager& psmm, const CoordinateConverter& cc,
    const PoseProto& pose, const PlanTask& current_task, int task_queue_size,
    bool new_task, bool rerouted, bool has_stopped_at_route_end,
    std::vector<PathPoint>* previous_st_path_global_including_past,
    bool* stopped_at_route_end) {
  if (new_task && current_task.type() == ON_ROAD_CRUISE_PLAN) {
    previous_st_path_global_including_past->clear();
  }
  // Reset stopped at route end when reroute.
  if (rerouted) {
    *stopped_at_route_end = false;
  }

  if (!has_stopped_at_route_end && task_queue_size == 1 &&
      current_task.has_destination_info()) {
    *stopped_at_route_end =
        ReachedRouteEnd(Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
                        pose.vel_body().x(), current_task, cc, psmm);
  }
}

bool UpdatePlanTaskQueue(
    const PlannerSemanticMapManager& psmm, const CoordinateConverter& cc,
    const PoseProto& pose, const AutonomyStateProto& autonomy_state,
    const RouteManagerOutput& route_output,
    const PlannerStateProto& planner_state_proto,
    const mapping::LanePath& prev_target_lane_path,
    const PlannerFunctionsParamsProto& planner_functions_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const std::queue<OutOfBlockedRoadRequestProto>&
        pending_out_of_blocked_road_requests,
    int planner_runtime_uturn_level,
    int planner_async_low_freq_cycle_iterations,
    std::deque<PlanTask>* plan_task_queue) {
  bool new_task = false;
  // Update task queue when av need to U-turn, only in auto drive mode.
  const bool enable_three_point_turn =
      (planner_runtime_uturn_level == 2 ||
       (planner_runtime_uturn_level == 1 &&
        planner_functions_params.enable_three_point_turn()));
  if (planner_async_low_freq_cycle_iterations == 0 &&
      IS_AUTO_DRIVE(autonomy_state.autonomy_state()) &&
      UpdatePlanTasksQueueWhenUturnAhead(
          enable_three_point_turn, psmm, pose, planner_state_proto,
          route_output, prev_target_lane_path, plan_task_queue)) {
    new_task = true;
  }

  // Update task queue when av need to out of blocked.
  if (UpdatePlanTasksQueueByOutOfBlockedRequest(
          psmm, pose, pending_out_of_blocked_road_requests, veh_geo_params,
          plan_task_queue)) {
    new_task = true;
  }

  // Update task queue when current task completed.
  if (plan_task_queue->size() > 1 &&
      PlanTaskCompeleted(plan_task_queue->front(), autonomy_state, cc,
                         Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()) +
                             veh_geo_params.front_edge_to_center() *
                                 Vec2d::FastUnitFromAngle(pose.yaw()),
                         pose.yaw(), pose.vel_body().x(), psmm)) {
    // Iterate to next task.
    plan_task_queue->pop_front();
    new_task = true;
  }

  // Finish current off road plan task when takeover.
  if (plan_task_queue->size() > 0 &&
      plan_task_queue->front().type() == OFF_ROAD_PLAN &&
      autonomy_state.autonomy_state() ==
          AutonomyStateProto::EMERGENCY_TO_TAKEOVER) {
    plan_task_queue->pop_front();
    new_task = true;
  }

  return new_task;
}

PlanStartPointInfo PreprocessPlanStartPoint(
    const TrajectoryProto& previous_trajectory, const PoseProto& pose,
    const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state, const Chassis& chassis,
    const PlannerParamsProto& planner_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, PlanTaskType task_type,
    absl::Time predicted_plan_time, bool previously_triggered_aeb) {
  switch (task_type) {
    case ON_ROAD_CRUISE_PLAN:
    case ALCC_PLAN:
    case MAPLESS_NOA:
      return ComputeEstPlanStartPoint(
          predicted_plan_time, previous_trajectory, pose, now_autonomy_state,
          prev_autonomy_state, previously_triggered_aeb, chassis,
          planner_params.motion_constraint_params(), vehicle_geom_params,
          vehicle_drive_params);
    case ACC_PLAN:
      return ComputeACCPlanStartPoint(
          predicted_plan_time, previous_trajectory, pose, now_autonomy_state,
          prev_autonomy_state, previously_triggered_aeb, chassis,
          planner_params.acc_params().motion_constraint_params(),
          vehicle_geom_params, vehicle_drive_params);
    case OFF_ROAD_PLAN:
    case UTURN_PLAN:
    case BLOCKED_PLAN:
    case APA_PLAN:
      return ComputeFreespacePlanStartPoint(
          predicted_plan_time, previous_trajectory, pose, now_autonomy_state,
          prev_autonomy_state, chassis,
          planner_params.motion_constraint_params(), vehicle_geom_params,
          vehicle_drive_params);
      break;
  }
}

std::vector<ApolloTrajectoryPointProto> PreprocessPrevTrajectoryPoints(
    const PlanStartPointInfo& plan_start_point_info,
    const PlannerParamsProto& planner_params,
    const TrajectoryProto& previous_trajectory, PlanTaskType task_type) {
  const auto& motion_constraint_params =
      task_type == ACC_PLAN
          ? planner_params.acc_params().motion_constraint_params()
          : planner_params.motion_constraint_params();

  const bool reset = task_type == ACC_PLAN
                         ? (plan_start_point_info.reset &&
                            (plan_start_point_info.reset_reason !=
                             ResetReasonProto::SPEED_ONLY))
                         : plan_start_point_info.reset;
  return CreatePreviousTrajectory(plan_start_point_info.plan_time,
                                  previous_trajectory, motion_constraint_params,
                                  reset);
}

absl::StatusOr<PlannerObjectManager> PreprocessPlannerObjectManager(
    const PoseProto& pose, const VehicleGeometryParamsProto& veh_geo_params,
    ObjectVector<PlannerObject> planner_objects, PlanTaskType task_type,
    double planner_prediction_probability_threshold,
    double planner_filter_reflected_object_distance, ThreadPool* thread_pool) {
  // Enabled low likelihood object filter.
  const LowLikelihoodFilter low_likelihood_filter(
      planner_prediction_probability_threshold,
      /*only_use_most_likely_traj=*/false);
  const MotionStateFilter motion_state_filter(pose, veh_geo_params);
  const PredictedMotionFilter predicted_motion_filter(planner_objects);
  std::vector<const TrajectoryFilter*> filters = {&low_likelihood_filter,
                                                  &predicted_motion_filter};
  // Enabled motion state filter on ON_ROAD_CRUISE_PLAN.
  if (task_type == ON_ROAD_CRUISE_PLAN) {
    filters.push_back(&motion_state_filter);
  }
  // TODO(lidong): Delete the code after March 01, 2022
  const ReflectedObjectInProximityFilter object_in_proximity_filter(
      pose, veh_geo_params, planner_filter_reflected_object_distance);
  if (planner_filter_reflected_object_distance > 0.0) {
    filters.push_back(&object_in_proximity_filter);
  }
  const UnknownRoadwayPositionObjectFilter
      unknown_roadway_position_object_filter;
  if (FLAGS_planner_filter_unknown_roadway_position_object) {
    filters.push_back(&unknown_roadway_position_object_filter);
  }
  PlannerObjectManagerBuilder obj_mgr_builder;
  obj_mgr_builder.set_planner_objects(std::move(planner_objects))
      .set_filters(filters);
  return obj_mgr_builder.Build(/*filtered_trajs=*/nullptr, thread_pool);
}

absl::StatusOr<ParkingSpotFinderInfo> GetParkingSpotFinderSpotInfo(
    const PlannerInput& input, ExternalCommandInfo* ext_cmd_info) {
  const auto parking_spot_id_opt = ext_cmd_info->status.apa_parking_spot_id;
  if (!parking_spot_id_opt.has_value()) {
    return absl::UnavailableError("No spot ID recieved.");
  }

  if (input.parking_spot_finder == nullptr) {
    return absl::UnavailableError("Not parking spot finder input.");
  }

  std::optional<ParkingSpotFinderInfo> parking_spot_info_opt = std::nullopt;
  for (const auto& spot : input.parking_spot_finder->spots()) {
    if (spot.id() == *parking_spot_id_opt) {
      parking_spot_info_opt = spot;
      break;
    }
  }
  if (!parking_spot_info_opt.has_value()) {
    return absl::UnavailableError(
        absl::StrFormat("Can't find ID %s from parking spot finder input.",
                        *parking_spot_id_opt));
  }
  return *parking_spot_info_opt;
}

void MaybeClearParkingSpotId(
    PathManagerStateProto::DriveState path_manager_drive_state,
    ExternalCommandInfo* ext_cmd_info) {
  if (path_manager_drive_state == PathManagerStateProto::REACH_FINAL_GOAL) {
    ext_cmd_info->status.apa_parking_spot_id = std::nullopt;
  }
}

PlannerStatus ExecuteAccPlan(
    const PlannerInput& input,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const PlanStartPointInfo& plan_start_point_info,
    const PlannerSemanticMapManager* psmm,
    const SpacetimeTrajectoryManager* st_traj_mgr, bool is_acc_engage_only,
    ExternalCommandInfo* ext_cmd_info, PlannerState* planner_state,
    PlannerOutput* output) {
  VLOG(2) << "Execute Acc Plan, is acc engage only? " << is_acc_engage_only;
  //   ---------------------- ODC -----------------------
  const auto odc_output = GeneratePlannerOdcByPose((OdcGeneratorInput{
      .psmm = psmm,
      .pose = input.pose.get(),
      .vehicle_geom = &input.vehicle_params.vehicle_geometry_params(),
      .target_lane_path = nullptr,
      .is_lane_changing = false,
      .is_acc_engage_only = is_acc_engage_only}));
  if (odc_output.ok()) {
    ParseOdcToExternalCommandStatus(*odc_output, &ext_cmd_info->status.output);
  }

  AccTaskOutput acc_output;
  auto status = RunAccTask(
      AccTaskInput{
          .planner_semantic_map_manager = psmm,
          .pose = input.pose.get(),
          .steering_percentage =
              input.chassis == nullptr
                  ? std::nullopt
                  : std::make_optional(input.chassis->steering_percentage()),
          .acc_params = &input.planner_params.acc_params(),
          .vehicle_geometry_params =
              &(input.vehicle_params.vehicle_geometry_params()),
          .vehicle_drive_params =
              &(input.vehicle_params.vehicle_drive_params()),
          .plan_start_point_info = &plan_start_point_info,
          .plan_time = plan_start_point_info.plan_time,
          .st_traj_mgr = st_traj_mgr,
          .prev_trajectory = &planner_state->previous_trajectory,
          .time_aligned_prev_traj = &time_aligned_prev_traj,
          .is_acc_standwait =
              input.autonomy_state == nullptr ||
                      !input.autonomy_state->has_assist_state() ||
                      !input.autonomy_state->assist_state()
                           .has_assist_acc_state() ||
                      !input.autonomy_state->assist_state()
                           .assist_acc_state()
                           .has_state()
                  ? false
                  : input.autonomy_state->assist_state()
                            .assist_acc_state()
                            .state() ==
                        AssistAccStateProto::ACC_STATE_STANDWAIT,
          .prev_collision_warning_request =
              planner_state->prev_collision_warning_request,
          .average_kappa = input.av_context->GetAvKappaCacheAverage(),
          .acc_task_proto = &planner_state->assist_plan_state.acc_task(),
          .lcc_cruising_speed_limit =
              ext_cmd_info->status.lcc_cruising_speed_limit,
          .following_distance_level =
              ext_cmd_info->status.following_distance_level},
      &acc_output);

  *output->mutable_trajectory() = std::move(acc_output.trajectory_info);
  FillPlannerDebugProto(std::move(acc_output.debug_info),
                        FLAGS_planner_simplify_debug_proto,
                        output->mutable_planner_debug());
  *output->mutable_charts_data() = std::move(acc_output.chart_data);
  *output->mutable_hmi_content() = std::move(acc_output.hmi_content);
  *planner_state->assist_plan_state.mutable_acc_task() =
      std::move(acc_output.acc_task_proto);
  ext_cmd_info->status.acc_state = acc_output.acc_state;
  if (acc_output.cruising_speed_limit.has_value()) {
    ext_cmd_info->status.output.cruising_speed_limit =
        acc_output.cruising_speed_limit;
    VLOG(2) << "Output speed limit " << acc_output.cruising_speed_limit.value();
  }
  return status;
}

TrajectoryProto::CollisionRisk GetCollisionRiskType(
    const PoseProto& av_pose, const ObjectsProto& objects,
    const VehicleGeometryParamsProto& veh_geom_param) {
  constexpr double kAvSpeedThres = 0.2;  // m/s.
  if (av_pose.vel_smooth().x() <= kAvSpeedThres) {
    return TrajectoryProto::NO_COLLISION_RISK;
  }
  const double half_length = 0.5 * (veh_geom_param.front_edge_to_center() -
                                    veh_geom_param.wheel_base());
  const double half_width = veh_geom_param.width() * 0.5;
  const double rac_to_center = half_length + veh_geom_param.wheel_base();
  const Vec2d rac(av_pose.pos_smooth().x(), av_pose.pos_smooth().y());
  const Vec2d tangent = Vec2d::FastUnitFromAngle(av_pose.yaw());
  const Vec2d center = rac + tangent * rac_to_center;
  const Box2d av_box(half_length, half_width, center, av_pose.yaw(), tangent);

  const auto is_considered_type = [](const auto type) {
    return type == OT_VEHICLE || type == OT_MOTORCYCLIST ||
           type == OT_PEDESTRIAN || type == OT_CYCLIST ||
           type == OT_LARGE_VEHICLE || type == OT_TRICYCLIST;
  };

  for (const ObjectProto& object : objects.objects()) {
    if (!is_considered_type(object.type())) {
      continue;
    }
    const auto contour = ComputeObjectContour(object);
    if (contour.HasOverlap(av_box)) {
      return TrajectoryProto::COLLISION_OCCURRED;
    }
  }
  return TrajectoryProto::NO_COLLISION_RISK;
}

}  // namespace

// NOLINTNEXTLINE
PlannerStatus RunPlanTaskDispatcher(
    const CoordinateConverter& coordinate_converter, const PlannerInput& input,
    const RouteManagerOutput& route_output, const ObjectsProto* objects_proto,
    absl::Time current_time, double time_interval,
    ExternalCommandInfo* ext_cmd_info, PlannerState* planner_state,
    PlannerOutput* output, ThreadPool* thread_pool) {
  SCOPED_QTRACE("RunPlanTaskDispatcher");

  const auto& veh_geo_params = input.vehicle_params.vehicle_geometry_params();
  const auto& veh_drive_params = input.vehicle_params.vehicle_drive_params();

  // -----------------------------------------------------
  // ------------ Update plan task queue -----------------
  // -----------------------------------------------------
  const bool rerouted =
      route_output.update_id != planner_state->route_update_id;
  planner_state->route_update_id = route_output.update_id;

  auto& plan_task_queue = planner_state->plan_task_queue;

  if (IsRunModeL4() && input.planner_semantic_map_manager == nullptr) {
    return ClassifyTaskErrorToPlannerStatus(ON_ROAD_CRUISE_PLAN,
                                            "Empty HD map for L4 mode.",
                                            /*is_driverless_mode=*/true);
  }
  // Psmm is only used for L4 mode to judge plan task.
  auto switch_task_result = SwitchPlanTask(
      GetRunMode(), FLAGS_planner_task_init_type, plan_task_queue,
      input.planner_semantic_map_manager,
      input.autonomy_state->assist_state().assist_drive_system_state(),
      route_output, rerouted);

  if (switch_task_result.switched) {
    ResetAssistPlanStateByTaskType(
        switch_task_result.new_task_queue.front().type(),
        &planner_state->assist_plan_state, &ext_cmd_info->status);
    UpdatePlannerStateOnTaskSwitch(
        plan_task_queue.front().type(),
        switch_task_result.new_task_queue.front().type(),
        FLAGS_planner_async_low_freq_cycle_iterations,
        FLAGS_planner_alcc_async_low_freq_cycle_iterations, planner_state);
  }
  plan_task_queue = std::move(switch_task_result.new_task_queue);

  if (plan_task_queue.empty()) {
    return PlannerStatus(PlannerStatusProto::PLANNER_INTERNAL_FAILED,
                         "Empty task queue.");
  }

  // Construct online psmm for alcc task.
  auto psmm = input.planner_semantic_map_manager;
  const auto& front_plan_task_type = plan_task_queue.front().type();

  auto update_psmm_status = UpdatePlannerSemanticMapManagerByPlanTask(
      input.online_semantic_map, input.localization_transform,
      front_plan_task_type, &psmm, IsRunModeL4(), thread_pool);
  if (!update_psmm_status.ok()) {
    return update_psmm_status;
  }

  const auto& planner_params = input.planner_params;
  bool new_task = UpdatePlanTaskQueue(
      *QCHECK_NOTNULL(psmm), coordinate_converter, *QCHECK_NOTNULL(input.pose),
      *QCHECK_NOTNULL(input.autonomy_state), route_output,
      *QCHECK_NOTNULL(input.planner_state_proto),
      planner_state->prev_target_lane_path,
      planner_params.planner_functions_params(), veh_geo_params,
      ext_cmd_info->queue.pending_out_of_blocked_road_requests,
      FLAGS_planner_runtime_uturn_level,
      FLAGS_planner_async_low_freq_cycle_iterations, &plan_task_queue);
  new_task = new_task || rerouted;
  const auto& current_task = plan_task_queue.front();

  UpdatePlannerStateInfoByCurrentTask(
      *psmm, coordinate_converter, *input.pose, current_task,
      plan_task_queue.size(), new_task, rerouted,
      planner_state->stopped_at_route_end,
      &planner_state->previous_st_path_global_including_past,
      &planner_state->stopped_at_route_end);

  // -----------------------------------------------------
  // ------------ Prepare data ---------------------------
  // -----------------------------------------------------
  // Validate cruising speed limit only in NOA/ALCC/ACC task.
  if (IsRunModeL4()) {
    ext_cmd_info->status.lcc_cruising_speed_limit.reset();
    ext_cmd_info->status.noa_cruising_speed_limit.reset();
  }

  TrajectoryProto previous_trajectory = planner_state->previous_trajectory;

  const absl::Cleanup clean_prev_traj_async = [&previous_trajectory] {
    DestroyContainerAsyncMarkSource(std::move(previous_trajectory),
                                    (QCRAFT_LOC).ToString());
  };

  const auto predicted_plan_time =
      current_time + absl::Milliseconds(FLAGS_planner_lookforward_time_ms);
  if (FLAGS_planner_enable_cross_iteration_tf &&
      current_task.type() == ON_ROAD_CRUISE_PLAN) {
    std::vector<PathPoint> previous_st_path;
    ConvertPreviousPathToCurrentSmooth(
        coordinate_converter,
        planner_state->previous_st_path_global_including_past,
        &previous_st_path);
    ConvertPreviousTrajectoryToCurrentSmoothLateral(
        predicted_plan_time, std::move(previous_st_path), &previous_trajectory);
  }

  // Find plan start point.
  // NOTE: For freespace tasks, plan start point v is not clamped by zero here
  // when reset (because driving direction is currently unknown) but left to
  // freespace planner. And also, plan start point may be reset in freespace
  // planner. Use the result with caution.
  const auto plan_start_point_info = PreprocessPlanStartPoint(
      previous_trajectory, *input.pose, *input.autonomy_state,
      planner_state->previous_autonomy_state, *input.chassis, planner_params,
      veh_geo_params, veh_drive_params, current_task.type(),
      predicted_plan_time, planner_state->previously_triggered_aeb);
  const auto& plan_time = plan_start_point_info.plan_time;

  const auto time_aligned_prev_traj_points =
      PreprocessPrevTrajectoryPoints(plan_start_point_info, planner_params,
                                     previous_trajectory, current_task.type());

  const bool should_run_aeb_planner = ShouldRunAebPlanner(current_task.type());
  AebPlannerOutput aeb_output;
  if (should_run_aeb_planner) {
    aeb_output.emergency_stop_info =
        aeb::GenerateEmergencyStopInfo(input, time_aligned_prev_traj_points);
    aeb_output.trajectory_points = aeb::PlanEmergencyStopTrajectory(
        plan_start_point_info.start_point,
        plan_start_point_info.path_s_increment_from_previous_frame,
        plan_start_point_info.reset, time_aligned_prev_traj_points,
        planner_params);
  }

  // Preprocess planner objects.
  auto preprocess_planner_objects_output = PreprocessPlannerObjects(
      *psmm, *QCHECK_NOTNULL(input.traffic_light_states),
      input.prediction_conflict_resolver_params, input.prediction.get(),
      objects_proto, plan_time, FLAGS_planner_consider_objects, thread_pool);

  auto object_manager_or = PreprocessPlannerObjectManager(
      *input.pose, veh_geo_params,
      std::move(preprocess_planner_objects_output.planner_objects),
      current_task.type(), FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_filter_reflected_object_distance, thread_pool);
  if (!object_manager_or.ok()) {
    auto* planner_status =
        output->mutable_planner_debug()->mutable_planner_status();
    planner_status->set_status(PlannerStatusProto::OBJECT_MANAGER_FAILED);
    planner_status->set_message(
        std::string(object_manager_or.status().message()));
    return ClassifyTaskErrorToPlannerStatus(
        plan_task_queue.front().type(), object_manager_or.status().message(),
        IsRunModeL4());
  }
  auto object_manager = std::make_shared<const PlannerObjectManager>(
      std::move(*object_manager_or));
  auto st_traj_mgr = std::make_shared<SpacetimeTrajectoryManager>(
      absl::Span<const TrajectoryFilter*>{}, object_manager->planner_objects(),
      thread_pool);

  std::vector<BoundaryClusterProto> cluster_objects;
  if (input.fusion_parking_freespace != nullptr) {
    cluster_objects = {
        input.fusion_parking_freespace->boundary_cluster().begin(),
        input.fusion_parking_freespace->boundary_cluster().end()};
    // TODO(renjie): Clear stationary objects from object_manager and
    // st_traj_mgr.
  }
  auto cluster_obj_mgr =
      std::make_shared<PlannerClusterObjectManager>(std::move(cluster_objects));

  bool should_destroy_async = true;
  const absl::Cleanup clean_object_container_async =
      [&object_manager, &st_traj_mgr, &cluster_obj_mgr, &should_destroy_async] {
        if (should_destroy_async) {
          DestroyContainerAsyncMarkSource(std::move(object_manager),
                                          "task_dispatcher:object_manager");
          DestroyContainerAsyncMarkSource(std::move(st_traj_mgr),
                                          "task_dispatcher:st_traj_mgr");
          DestroyContainerAsyncMarkSource(std::move(cluster_obj_mgr),
                                          "task_dispatcher:cluster_obj_mgr");
        }
      };

  if (FLAGS_planner_enable_collision_risk) {
    const auto collosion_risk_type =
        GetCollisionRiskType(*input.pose, *input.real_objects, veh_geo_params);
    output->mutable_trajectory()->set_collision_risk(collosion_risk_type);
  }

  QCHECK_NOTNULL(psmm->semantic_map_manager());

  // Calculate route related odc.
  {
    const auto route_related_odc_or = GeneratePlannerOdcByRoute(
        input.planner_semantic_map_manager.get(), &route_output);
    if (route_related_odc_or.ok()) {
      ParseRouteRelatedOdcToExternalCommandStatus(*route_related_odc_or,
                                                  &ext_cmd_info->status.output);
    }
  }

  // ---------- Executing current task --------------------
  switch (current_task.type()) {
    case ON_ROAD_CRUISE_PLAN: {
      //   ----------------- Cruise Task -------------------
      std::unique_ptr<CruiseTaskOutput> cruise_result =
          std::make_unique<CruiseTaskOutput>();
      const auto on_road_plan_status = RunCruiseTask(
          CruiseTaskInput{
              .rerouted = rerouted,
              .coordinate_converter = &coordinate_converter,
              .planner_input = &input,
              .planner_params = &planner_params,
              .route_output = &route_output,
              .plan_start_point_info = &plan_start_point_info,
              .ext_cmd_queue = &ext_cmd_info->queue,
              .plan_time = plan_time,
              .st_traj_mgr = st_traj_mgr,
              .object_manager = object_manager,
              .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
              .previous_trajectory = &previous_trajectory,
              .aeb_output = &aeb_output},
          cruise_result.get(), planner_state, &ext_cmd_info->status,
          thread_pool);
      should_destroy_async = !cruise_result->scheduled_async_low_freq;

      *output->mutable_trajectory() = std::move(cruise_result->trajectory_info);
      FillPlannerDebugProto(std::move(cruise_result->debug_info),
                            FLAGS_planner_simplify_debug_proto,
                            output->mutable_planner_debug());

      *output->mutable_charts_data() = std::move(cruise_result->chart_data);
      *output->mutable_hmi_content() = std::move(cruise_result->hmi_content);

      DestroyContainerAsyncMarkSource(std::move(cruise_result),
                                      (QCRAFT_LOC).ToString());

      if (!IsRunModeL4()) {
        //   ---------------------- ODC -----------------------
        auto odc_output = GeneratePlannerOdcOnRoad(
            planner_state->prev_low_freq_psmm == nullptr
                ? *psmm
                : *planner_state->prev_low_freq_psmm,
            *input.pose, input.vehicle_params.vehicle_geometry_params(),
            on_road_plan_status,
            output->trajectory().target_lane_path_from_current(),
            IsLaneChangeStage(output->trajectory().lane_change_stage()));
        if (odc_output.ok()) {
          ParseOdcToExternalCommandStatus(*odc_output,
                                          &ext_cmd_info->status.output);
        }
      }

      if (!on_road_plan_status.ok()) {
        on_road_plan_status.ToProto(
            output->mutable_planner_debug()->mutable_planner_status());
        if (on_road_plan_status.status_code() ==
            PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED) {
          ext_cmd_info->status.output.planner_to_router_command =
              ExternalRouterCommand::INPLACE_REROUTE;
        }

        return ClassifyTaskErrorToPlannerStatus(
            ON_ROAD_CRUISE_PLAN, on_road_plan_status.message(), IsRunModeL4());
      }

      break;
    }
    case UTURN_PLAN: {
      //   ----------------- Uturn Task -------------------
      ASSIGN_OR_RETURN(
          const auto goal,
          CalculateGoalPoseByDestinationInfo(*psmm, coordinate_converter,
                                             current_task.destination_info()),
          ClassifyTaskErrorToPlannerStatus(
              UTURN_PLAN, "Task destination must be specified.",
              IsRunModeL4()));

      mapping::LanePath uturn_ref_lane_path;
      if (current_task.destination_info().uturn_ref_lane_path.has_value()) {
        uturn_ref_lane_path.FromProto(
            psmm->semantic_map_manager(),
            current_task.destination_info().uturn_ref_lane_path.value());
      }

      const bool replan = !IsAutoDrive(input.autonomy_state->autonomy_state());
      UTurnTaskOutput result;
      const auto status = RunUTurnTask(
          UTurnTaskInput{
              .reset = new_task || replan,
              .autonomy_state = input.autonomy_state.get(),
              .psmm = psmm.get(),
              .coordinate_converter = &coordinate_converter,
              .goal = &goal,
              .lane_path = uturn_ref_lane_path.IsEmpty() ? nullptr
                                                         : &uturn_ref_lane_path,
              .pose = input.pose.get(),
              .chassis = input.chassis.get(),
              .plan_start_point_info = &plan_start_point_info,
              .plan_time = plan_time,
              .freespace_params =
                  &planner_params.freespace_params_for_driving(),
              .vehicle_models_params = &planner_params.vehicle_models_params(),
              .veh_geo_params = &veh_geo_params,
              .veh_drive_params = &veh_drive_params,
              .prev_trajectory_proto = &previous_trajectory,
              .object_manager = object_manager.get(),
              .cluster_object_manager = cluster_obj_mgr.get(),
              .time_interval = time_interval},
          &planner_state->freespace_planner_state, &result, thread_pool);

      *output->mutable_trajectory() = std::move(result.trajectory_info);
      *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
          std::move(result.debug_proto);
      *output->mutable_charts_data() = std::move(result.chart_data);
      output->mutable_planner_debug()->set_active_planner(result.planner_type);
      SetFreespaceResetReason(plan_start_point_info, result.reset,
                              result.reset_reason,
                              output->mutable_planner_debug());

      if (!status.ok()) {
        return ClassifyTaskErrorToPlannerStatus(UTURN_PLAN, status.message(),
                                                IsRunModeL4());
      }

      break;
    }
    case OFF_ROAD_PLAN: {
      if (current_task.destination_info().dest.parking_spots.has_value()) {
        //   ----------------- Parking Task -------------------
        const auto& parking_spots =
            current_task.destination_info().dest.parking_spots;
        QCHECK(!parking_spots->empty());
        const bool replan =
            !IsAutoDrive(input.autonomy_state->autonomy_state());
        // Parking spots are given.
        ParkingTaskOutput result;
        const auto status = RunParkingTask(
            ParkingTaskInput{.reset = new_task || replan,
                             .autonomy_state = input.autonomy_state.get(),
                             .psmm = psmm.get(),
                             .coordinate_converter = &coordinate_converter,
                             .parking_spot_id = parking_spots->front(),
                             .pose = input.pose.get(),
                             .chassis = input.chassis.get(),
                             .plan_start_point_info = &plan_start_point_info,
                             .plan_time = plan_time,
                             .freespace_params =
                                 &planner_params.freespace_params_for_parking(),
                             .vehicle_models_params =
                                 &planner_params.vehicle_models_params(),
                             .veh_geo_params = &veh_geo_params,
                             .veh_drive_params = &veh_drive_params,
                             .prev_trajectory_proto = &previous_trajectory,
                             .object_manager = object_manager.get(),
                             .cluster_object_manager = cluster_obj_mgr.get(),
                             .time_interval = time_interval},
            &planner_state->freespace_planner_state, &result, thread_pool);

        *output->mutable_trajectory() = std::move(result.trajectory_info);
        *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
            std::move(result.debug_proto);
        *output->mutable_charts_data() = std::move(result.chart_data);
        output->mutable_planner_debug()->set_active_planner(
            result.planner_type);
        SetFreespaceResetReason(plan_start_point_info, result.reset,
                                result.reset_reason,
                                output->mutable_planner_debug());

        if (!status.ok()) {
          return ClassifyTaskErrorToPlannerStatus(
              OFF_ROAD_PLAN, status.message(), IsRunModeL4());
        }
      } else {
        //   ----------------- Free Drive Task -------------------
        ASSIGN_OR_RETURN(
            const auto goal,
            CalculateGoalPoseByDestinationInfo(*psmm, coordinate_converter,
                                               current_task.destination_info()),
            ClassifyTaskErrorToPlannerStatus(
                OFF_ROAD_PLAN, "Task destination must be specified.",
                IsRunModeL4()));
        const bool replan =
            !IsAutoDrive(input.autonomy_state->autonomy_state());
        FreeDriveTaskOutput result;
        const auto status = RunFreeDriveTask(
            FreeDriveTaskInput{
                .reset = new_task || replan,
                .autonomy_state = input.autonomy_state.get(),
                .psmm = psmm.get(),
                .coordinate_converter = &coordinate_converter,
                .goal = &goal,
                .pose = input.pose.get(),
                .chassis = input.chassis.get(),
                .plan_start_point_info = &plan_start_point_info,
                .plan_time = plan_time,
                .freespace_params =
                    &planner_params.freespace_params_for_driving(),
                .vehicle_models_params =
                    &planner_params.vehicle_models_params(),
                .veh_geo_params = &veh_geo_params,
                .veh_drive_params = &veh_drive_params,
                .prev_trajectory_proto = &previous_trajectory,
                .object_manager = object_manager.get(),
                .cluster_object_manager = cluster_obj_mgr.get(),
                .time_interval = time_interval},
            &planner_state->freespace_planner_state, &result, thread_pool);

        *output->mutable_trajectory() = std::move(result.trajectory_info);
        *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
            std::move(result.debug_proto);
        *output->mutable_charts_data() = std::move(result.chart_data);
        output->mutable_planner_debug()->set_active_planner(
            result.planner_type);
        SetFreespaceResetReason(plan_start_point_info, result.reset,
                                result.reset_reason,
                                output->mutable_planner_debug());

        if (!status.ok()) {
          return ClassifyTaskErrorToPlannerStatus(
              OFF_ROAD_PLAN, status.message(), IsRunModeL4());
        }
      }
      break;
    }
    case ACC_PLAN: {
      //   ----------------- L2: ACC Task -------------------
      const auto status = ExecuteAccPlan(
          input, time_aligned_prev_traj_points, plan_start_point_info,
          psmm.get(), st_traj_mgr.get(),
          /*is_acc_engage_only=*/false, ext_cmd_info, planner_state, output);
      if (!status.ok()) {
        // ACC plan failed, request manual override.
        return ClassifyTaskErrorToPlannerStatus(ACC_PLAN, status.message(),
                                                IsRunModeL4());
      }
      break;
    }
    case ALCC_PLAN: {
      //   ----------------- L2: ALCC Task -------------------
      std::vector<Vec2d> prev_lane_path_points;
      if (planner_state->prev_low_freq_psmm != nullptr) {
        auto prev_lane_path_points_or =
            SampleLanePathByStep(*(planner_state->prev_low_freq_psmm),
                                 planner_state->prev_target_lane_path,
                                 /*step=*/1.0);
        if (prev_lane_path_points_or.ok()) {
          prev_lane_path_points = std::move(prev_lane_path_points_or).value();
        }
      }

      // Process online map.
      const auto online_map_info_or = PreprocessAlccOnlineMap(
          *input.autonomy_state, *input.pose, veh_geo_params, *st_traj_mgr,
          psmm, input.online_semantic_map, prev_lane_path_points,
          &planner_state->tja_state);
      AlccTaskOutput alcc_output;
      PlannerStatus status;
      if (online_map_info_or.ok()) {
        // Only run Async LCC task when we successfully generate ALCC online
        // map.
        const auto& online_map_info = online_map_info_or.value();
        status = RunAlccTask(
            AlccTaskInput{
                .planner_semantic_map_manager = online_map_info.psmm,
                .pose = input.pose.get(),
                .chassis = input.chassis.get(),
                .autonomy_state = input.autonomy_state.get(),
                .alcc_params = &planner_params.alcc_params(),
                .acc_params = &planner_params.acc_params(),
                .vehicle_params = &input.vehicle_params,
                .plan_start_point_info = &plan_start_point_info,
                .ext_cmd_queue = &ext_cmd_info->queue,
                .plan_time = plan_time,
                .st_traj_mgr = st_traj_mgr,
                .object_manager = object_manager,
                .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
                .log_av_trajectory = input.log_av_trajectory.get(),
                .online_semantic_map = online_map_info.online_semantic_map,
                // TODO(zixuan): Delete use online map option.
                .use_online_semantic_map =
                    online_map_info.psmm->IsOnVisionMap(),
                .av_context = input.av_context.get()},
            &alcc_output, planner_state, &ext_cmd_info->status, thread_pool);
        should_destroy_async = !alcc_output.scheduled_async_low_freq;

      } else {
        // Fail to generate ALCC online map, set a failure status code.
        status = PlannerStatus(PlannerStatusProto::ALCC_MAIN_LOOP_FAILED,
                               online_map_info_or.status().message());
      }

      // Execute an ACC plan if we fail to execute ALCC plan and current assit
      // state is OFF/NOT_READY
      const auto& assit_driving_state =
          input.autonomy_state->assist_state().assist_drive_system_state();
      const bool need_to_execute_acc_plan =
          FLAGS_planner_debug_force_acc_in_lcc ||
          ((!status.ok()) &&
           ((assit_driving_state == AssistStateProto::ASSIST_OFF) ||
            (assit_driving_state == AssistStateProto::ASSIST_NOT_READY)));
      if (need_to_execute_acc_plan) {
        //   ----------------- L2: ACC Task if ALCC failed -------------------
        const auto acc_status = ExecuteAccPlan(
            input, time_aligned_prev_traj_points, plan_start_point_info,
            psmm.get(), st_traj_mgr.get(),
            /*is_acc_engage_only=*/true, ext_cmd_info, planner_state, output);
        if (!acc_status.ok()) {
          // ACC plan failed, request manual override.
          return ClassifyTaskErrorToPlannerStatus(ACC_PLAN, status.message(),
                                                  IsRunModeL4());
        }
        break;
      }

      *output->mutable_trajectory() = std::move(alcc_output.trajectory_info);
      *output->mutable_charts_data() = std::move(alcc_output.chart_data);
      *output->mutable_hmi_content() = std::move(alcc_output.hmi_content);
      FillPlannerDebugProto(std::move(alcc_output.debug_info),
                            FLAGS_planner_simplify_debug_proto,
                            output->mutable_planner_debug());

      if (!planner_state->tja_state.planner_use_tja_map) {
        // Keep the reference center line before entering TJA.
        FillPlannerCenterLine(output->planner_debug()
                                  .async_high_freq_debug()
                                  .scheduler()
                                  .path_boundary()
                                  .reference_center(),
                              &planner_state->tja_state.planner_center_line);
      }

      //   ---------------------- ODC -----------------------
      const auto& cur_psmm =
          online_map_info_or.ok() ? *(online_map_info_or->psmm) : *psmm;
      auto odc_output = GeneratePlannerOdcOnRoad(
          planner_state->prev_low_freq_psmm == nullptr
              ? cur_psmm
              : *planner_state->prev_low_freq_psmm,
          *input.pose, input.vehicle_params.vehicle_geometry_params(), status,
          output->trajectory().target_lane_path_from_current(),
          IsLaneChangeStage(output->trajectory().lane_change_stage()));
      if (odc_output.ok()) {
        ParseOdcToExternalCommandStatus(*odc_output,
                                        &ext_cmd_info->status.output);
      }

      if (!status.ok()) {
        return ClassifyTaskErrorToPlannerStatus(ALCC_PLAN, status.message(),
                                                IsRunModeL4());
      }

      break;
    }
    case APA_PLAN: {
      //   ----------------- L2: APA Task -------------------
      // Clear states in some autonomy states.
      const auto autonomy_apa_state =
          input.autonomy_state->assist_state().assist_apa_state().state();
      const auto previous_autonomy_apa_state =
          planner_state->previous_autonomy_state.assist_state()
              .assist_apa_state()
              .state();
      if (autonomy_apa_state == AssistApaStateProto::APA_STATE_OFF ||
          autonomy_apa_state == AssistApaStateProto::APA_STATE_PASSIVE ||
          autonomy_apa_state == AssistApaStateProto::APA_STATE_PARKING_FINISH ||
          autonomy_apa_state == AssistApaStateProto::APA_STATE_FAULT ||
          (autonomy_apa_state !=
               AssistApaStateProto::APA_STATE_PARKING_SAFE_STOP &&
           previous_autonomy_apa_state ==
               AssistApaStateProto::APA_STATE_PARKING_SAFE_STOP)) {
        planner_state->freespace_planner_state.Clear();
        ext_cmd_info->status.apa_parking_spot_id = std::nullopt;
      }

      const auto parking_spot_info_or =
          GetParkingSpotFinderSpotInfo(input, ext_cmd_info);
      if (!parking_spot_info_or.ok()) {
        output->mutable_planner_debug()->set_reset(plan_start_point_info.reset);
        output->mutable_planner_debug()->set_reset_reason(
            plan_start_point_info.reset_reason);
        auto planner_status = ClassifyTaskErrorToPlannerStatus(
            APA_PLAN, parking_spot_info_or.status().message(), IsRunModeL4());
        planner_status.ToProto(
            output->mutable_planner_debug()->mutable_planner_status());
        return planner_status;
      }
      const bool replan =
          (autonomy_apa_state !=
               AssistApaStateProto::APA_STATE_PARKING_ACTIVE_ON &&
           autonomy_apa_state !=
               AssistApaStateProto::APA_STATE_PARKING_ACTIVE_STANDBY &&
           autonomy_apa_state !=
               AssistApaStateProto::APA_STATE_PARKING_ACTIVE_PAUSE &&
           autonomy_apa_state !=
               AssistApaStateProto::APA_STATE_PARKING_SAFE_STOP);
      ApaParkingTaskOutput result;
      const auto status = RunApaParkingTask(
          ApaParkingTaskInput{
              .reset = new_task || replan,
              .autonomy_state = input.autonomy_state.get(),
              .parking_spot_info =
                  mapping::ParkingSpotInfo(*parking_spot_info_or),
              .pose = input.pose.get(),
              .chassis = input.chassis.get(),
              .plan_start_point_info = &plan_start_point_info,
              .plan_time = plan_time,
              .freespace_params =
                  &planner_params.freespace_params_for_parking(),
              .vehicle_models_params = &planner_params.vehicle_models_params(),
              .veh_geo_params = &veh_geo_params,
              .veh_drive_params = &veh_drive_params,
              .prev_trajectory_proto = &previous_trajectory,
              .object_manager = object_manager.get(),
              .cluster_object_manager = cluster_obj_mgr.get(),
              .time_interval = time_interval},
          &planner_state->freespace_planner_state, &result, thread_pool);
      *output->mutable_trajectory() = std::move(result.trajectory_info);
      *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
          std::move(result.debug_proto);
      *output->mutable_charts_data() = std::move(result.chart_data);
      output->mutable_planner_debug()->set_active_planner(result.planner_type);
      SetFreespaceResetReason(plan_start_point_info, result.reset,
                              result.reset_reason,
                              output->mutable_planner_debug());

      // Clear parking spot ID if parking finished.
      MaybeClearParkingSpotId(
          planner_state->freespace_planner_state.path_manager_state()
              .drive_state(),
          ext_cmd_info);

      if (!status.ok()) {
        auto planner_status = ClassifyTaskErrorToPlannerStatus(
            APA_PLAN, status.message(), IsRunModeL4());
        planner_status.ToProto(
            output->mutable_planner_debug()->mutable_planner_status());
        return planner_status;
      }

      break;
    }
    case BLOCKED_PLAN: {
      //   ----------------- Blocked Task -------------------
      ASSIGN_OR_RETURN(
          const auto goal,
          CalculateGoalPoseByDestinationInfo(*psmm, coordinate_converter,
                                             current_task.destination_info()),
          ClassifyTaskErrorToPlannerStatus(
              BLOCKED_PLAN, "Task destination must be specified.",
              IsRunModeL4()));
      const bool replan = !IsAutoDrive(input.autonomy_state->autonomy_state());
      FreeDriveTaskOutput result;
      const auto status = RunFreeDriveTask(
          FreeDriveTaskInput{
              .reset = new_task || replan,
              .psmm = psmm.get(),
              .coordinate_converter = &coordinate_converter,
              .goal = &goal,
              .pose = input.pose.get(),
              .chassis = input.chassis.get(),
              .plan_start_point_info = &plan_start_point_info,
              .plan_time = plan_time,
              .freespace_params =
                  &planner_params.freespace_params_for_driving(),
              .vehicle_models_params = &planner_params.vehicle_models_params(),
              .veh_geo_params = &veh_geo_params,
              .veh_drive_params = &veh_drive_params,
              .prev_trajectory_proto = &previous_trajectory,
              .object_manager = object_manager.get(),
              .cluster_object_manager = cluster_obj_mgr.get(),
              .time_interval = time_interval},
          &planner_state->freespace_planner_state, &result, thread_pool);

      *output->mutable_trajectory() = std::move(result.trajectory_info);
      *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
          std::move(result.debug_proto);
      *output->mutable_charts_data() = std::move(result.chart_data);
      output->mutable_planner_debug()->set_active_planner(result.planner_type);
      SetFreespaceResetReason(plan_start_point_info, result.reset,
                              result.reset_reason,
                              output->mutable_planner_debug());

      if (!status.ok()) {
        return ClassifyTaskErrorToPlannerStatus(BLOCKED_PLAN, status.message(),
                                                IsRunModeL4());
      }

      break;
    }
    case MAPLESS_NOA: {
      const auto status =
          RunMaplessTask(planner_state, &ext_cmd_info->status, thread_pool);

      if (!status.ok()) {
        return ClassifyTaskErrorToPlannerStatus(MAPLESS_NOA, status.message(),
                                                IsRunModeL4());
      }

      break;
      // TODO(weijun): collect results.
    }
  }

  planner_state->previous_trajectory = output->trajectory();
  if (input.chassis->parking_brake()) {
    planner_state->parking_brake_release_time = plan_time;
  }

  // Fill prediction post process result to PlannerDebugProto.
  *output->mutable_planner_debug()->mutable_prediction_post_process() =
      std::move(preprocess_planner_objects_output.prediction_post_process);
  SCOPED_QTRACE("RunPlanTaskDispatcher_end");

  return OkPlannerStatus();
}  // NOLINT

}  // namespace qcraft::planner
