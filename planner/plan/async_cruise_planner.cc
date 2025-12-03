#include "onboard/planner/plan/async_cruise_planner.h"

#include <float.h>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <cxxabi.h>  // for __forced_unwind
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/car_common.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/external_command_info.h"
#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_gap_finder.h"
#include "onboard/planner/decision/traffic_gap_result.h"
#include "onboard/planner/decision/traffic_gap_v2/proto/traffic_gap.pb.h"
#include "onboard/planner/decision/traffic_gap_v2/traffic_gap_finder.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/planner_object_util.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/acc/acc_task_input.h"
#include "onboard/planner/plan/acc/acc_task_internal.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/plan/multi_tasks_cruise_planner.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/selector/selector_output.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

double FindLanePathStartOffsetFromSections(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const mapping::LanePath& lane_path) {
  const double front_sec_frac = route_sections_from_start.start_fraction();
  const auto front_section_id = route_sections_from_start.section_id(0);
  for (const auto& lane_seg : lane_path) {
    SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, lane_seg.lane_id, 0.0);
    if (lane_info.section_id == front_section_id) {
      return lane_path.FirstOccurrenceOfLanePointToArclength(mapping::LanePoint(
          lane_seg.lane_id,
          std::max(lane_seg.start_fraction,
                   std::min(front_sec_frac, lane_seg.end_fraction))));
    }
  }
  return 0.0;
}

absl::Status UpdateLowFreqEstResult(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const PlannerObjectManager& object_manager,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const mapping::LanePath& prev_target_lane_path,
    const mapping::LanePath& prev_lane_path_before_lc,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const TrafficLightInfoMap& tl_info_map, bool prev_smooth_state,
    AutonomyStateProto::State autonomy_state,
    AsyncMultiTaskEstOutput* mutable_est_result, ThreadPool* thread_pool) {
  FUNC_QTRACE();
  auto& scheduler_output = mutable_est_result->est_output.scheduler_output;
  const auto& drive_passage = scheduler_output.drive_passage;
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  scheduler_output.target_offset_from_start =
      FindLanePathStartOffsetFromSections(psmm, route_sections_from_start,
                                          drive_passage.lane_path());
  const Box2d ego_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_RETURN(
      scheduler_output.av_frenet_box_on_drive_passage,
      drive_passage.QueryFrenetBoxAt(ego_box),
      absl::OutOfRangeError(absl::StrCat("Ego box ", ego_box.DebugString(),
                                         " is out of drive passage!")));

  const bool is_lc_pause =
      scheduler_output.lane_change_state.stage() == LaneChangeStage::LCS_PAUSE;
  const auto& ego_frenet_box = scheduler_output.av_frenet_box_on_drive_passage;
  scheduler_output.should_smooth =
      ShouldSmoothRefLane(tl_info_map, drive_passage, prev_smooth_state);
  const double ref_center_l =
      CalcAvhRefCenterL(psmm, drive_passage, ego_frenet_box, smooth_result_map,
                        scheduler_output.should_smooth);
  ASSIGN_OR_RETURN(
      scheduler_output.lane_change_state,
      MakeLaneChangeState(drive_passage, ego_pos, ego_frenet_box,
                          prev_target_lane_path, prev_lane_path_before_lc,
                          scheduler_output.lane_change_state, ref_center_l,
                          autonomy_state));
  if (is_lc_pause) {
    scheduler_output.lane_change_state.set_stage(LaneChangeStage::LCS_PAUSE);
  }

  auto& route_target_info = mutable_est_result->route_target_info;
  if (route_target_info.has_value()) {
    const auto ego_frenet_box_on_target_or =
        route_target_info->frenet_frame.QueryFrenetBoxAt(ego_box);
    if (ego_frenet_box_on_target_or.ok()) {
      route_target_info->ego_frenet_box = *ego_frenet_box_on_target_or;

      const SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
          .passage = &route_target_info->drive_passage,
          .sl_boundary = &route_target_info->sl_boundary,
          .obj_mgr = &object_manager,
          .on_vision_map = psmm.IsOnVisionMap()};
      route_target_info->st_traj_mgr =
          BuildSpacetimeTrajectoryManager(st_mgr_builder_input, thread_pool);
    } else {
      route_target_info = std::nullopt;
    }
  }

  return absl::OkStatus();
}

Future<PlannerStatus> ScheduleFutureMultiTaskEst(
    const MultiTasksCruisePlannerInput& input,
    const std::shared_ptr<PathBoundedEstPlannerOutput>&
        future_multi_task_est_result,
    ThreadPool* thread_pool, bool update_path_sections) {
  SCOPED_QTRACE("ScheduleFutureMultiTaskEst");

  if (!IsPlannerAsync(FLAGS_planner_async_low_freq_cycle_iterations)) {
    // Not in async mode, no need to make copies.
    return ScheduleFuture(
        thread_pool, [thread_pool, future_multi_task_est_result, &input]() {
          return RunMultiTasksCruisePlanner(
              input, future_multi_task_est_result.get(),
              FLAGS_planner_multi_est_in_parallel ? thread_pool : nullptr);
        });
  }

  const auto& psmm = input.planner_semantic_map_manager;
  const auto& vehicle_params = *input.vehicle_params;

  auto route_sections_from_start = *input.route_sections_from_start;
  auto ego_nearest_lane_id = input.ego_nearest_lane_id;
  auto prev_route_sections = *input.prev_route_sections;
  auto prev_target_lane_path = *input.prev_target_lane_path;
  auto smooth_result_map = *input.smooth_result_map;
  auto tl_info_map = *input.tl_info_map;
  // Update path route sections related message if necessary.
  if (update_path_sections) {
    QLOG(INFO) << "Try to update route for low freq module.";
    // Route sections
    const Vec2d pos = Vec2dFromApolloTrajectoryPointProto(
        input.start_point_info->start_point);
    ASSIGN_OR_RETURN(auto path_prev_route_sections,
                     BackwardExtendRouteSectionsFromPos(
                         *psmm, input.rm_output->route_sections_from_current,
                         pos, kDrivePassageKeepBehindLength),
                     Future<PlannerStatus>());

    auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        *psmm, path_prev_route_sections, pos, input.rm_output->route_navi_info);
    if (target_lane_path_or.ok()) {
      prev_target_lane_path = std::move(target_lane_path_or).value();
    }

    ASSIGN_OR_RETURN(
        auto route_sections_proj,
        ProjectPointToRouteSections(
            *psmm, path_prev_route_sections, pos,
            kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
            kDrivePassageKeepBehindLength),
        Future<PlannerStatus>());

    PointOnRouteSections ego_proj;
    std::tie(route_sections_from_start, prev_route_sections, ego_proj) =
        std::move(route_sections_proj);
    ego_nearest_lane_id = ego_proj.lane_id;

    // Smooth reference line.
    auto smooth_result_map_or = BuildSmoothedResultMapFromRouteSections(
        *psmm, route_sections_from_start,
        vehicle_params.vehicle_geometry_params().width() * 0.5,
        *input.smooth_result_map);
    if (smooth_result_map_or.ok()) {
      smooth_result_map = std::move(smooth_result_map_or).value();
    }

    // Traffic light info
    ASSIGN_OR_RETURN(auto tl_info_collector_output,
                     CollectTrafficLightInfo(
                         TrafficLightInfoCollectorInput{
                             .psmm = psmm.get(),
                             .traffic_light_states = input.traffic_light_states,
                             .route_sections = &route_sections_from_start,
                             .plan_time = input.start_point_info->plan_time},
                         *input.yellow_light_observations),
                     Future<PlannerStatus>());
    tl_info_map = std::move(tl_info_collector_output.tl_info_map);
  }

  return ScheduleFuture(
      thread_pool,
      [thread_pool, future_multi_task_est_result,
       psmm = input.planner_semantic_map_manager,
       online_semantic_map = input.online_semantic_map, &vehicle_params,
       coordinate_converter = *input.coordinate_converter,
       planner_params = *input.planner_params, rm_output = *input.rm_output,
       route_sections_from_start = std::move(route_sections_from_start),
       start_point_info = *input.start_point_info, ego_nearest_lane_id,
       min_path_look_ahead_duration = input.min_path_look_ahead_duration,
       st_traj_mgr = input.st_traj_mgr, object_manager = input.object_manager,
       stalled_objects = *input.stalled_objects,
       scene_reasoning = *input.scene_reasoning,
       ext_cmd_status = *input.ext_cmd_status,
       time_aligned_prev_traj = *input.time_aligned_prev_traj,
       tl_info_map = std::move(tl_info_map),
       previous_trajectory = *input.previous_trajectory,
       prev_target_lane_path = std::move(prev_target_lane_path),
       prev_route_sections = std::move(prev_route_sections),
       prev_length_along_route = input.prev_length_along_route,
       prev_max_reach_length = input.prev_max_reach_length,
       prev_station_anchor = *input.station_anchor,
       prev_lane_change_state = *input.lane_change_state,
       prev_lane_path_before_lc = *input.prev_lane_path_before_lc,
       preferred_lane_path = *input.preferred_lane_path,
       new_lc_command = input.new_lc_command,
       alc_confirmation = input.alc_confirmation,
       smooth_result_map = std::move(smooth_result_map),
       prev_smooth_state = input.prev_smooth_state,
       parking_brake_release_time = input.parking_brake_release_time,
       prev_decider_state = *input.decider_state,
       prev_initializer_state = *input.initializer_state,
       selected_trajectory_optimizer_state_proto =
           (input.selected_trajectory_optimizer_state_proto != nullptr)
               ? *input.selected_trajectory_optimizer_state_proto
               : TrajectoryOptimizerStateProto(),
       has_selected_trajectory_optimizer_state_proto =
           (input.selected_trajectory_optimizer_state_proto != nullptr),
       prev_st_planner_trajectories = *input.st_planner_object_trajectories,
       prev_selector_state = *input.selector_state,
       traffic_light_states = *input.traffic_light_states,
       log_av_trajectory = input.log_av_trajectory != nullptr
                               ? *input.log_av_trajectory
                               : TrajectoryProto(),
       prediction_debug = input.prediction_debug == nullptr
                              ? PredictionDebugProto()
                              : *input.prediction_debug,
       planner_model_pool = input.planner_model_pool,
       real_objects = input.real_objects,
       virtual_objects = input.virtual_objects,
       planner_av_context = input.planner_av_context,
       context_feature = input.context_feature,
       is_standwait = input.is_standwait, chassis = input.chassis,
       autonomy_state = input.autonomy_state]() {
        const MultiTasksCruisePlannerInput multi_task_cruise_planner_input{
            .coordinate_converter = &coordinate_converter,
            .planner_semantic_map_manager = psmm,
            .online_semantic_map = online_semantic_map,
            .planner_params = &planner_params,
            .vehicle_params = &vehicle_params,
            .rm_output = &rm_output,
            .route_sections_from_start = &route_sections_from_start,
            .start_point_info = &start_point_info,
            .ego_nearest_lane_id = ego_nearest_lane_id,
            .min_path_look_ahead_duration = min_path_look_ahead_duration,
            .st_traj_mgr = st_traj_mgr,
            .object_manager = object_manager,
            .stalled_objects = &stalled_objects,
            .scene_reasoning = &scene_reasoning,
            .ext_cmd_status = &ext_cmd_status,
            .time_aligned_prev_traj = &time_aligned_prev_traj,
            .tl_info_map = &tl_info_map,
            .chassis = chassis,
            .autonomy_state = autonomy_state,
            .previous_trajectory = &previous_trajectory,
            .prev_target_lane_path = &prev_target_lane_path,
            .prev_route_sections = &prev_route_sections,
            .prev_length_along_route = prev_length_along_route,
            .prev_max_reach_length = prev_max_reach_length,
            .station_anchor = &prev_station_anchor,
            .lane_change_state = &prev_lane_change_state,
            .prev_lane_path_before_lc = &prev_lane_path_before_lc,
            .preferred_lane_path = &preferred_lane_path,
            .new_lc_command = new_lc_command,
            .alc_confirmation = alc_confirmation,
            .smooth_result_map = &smooth_result_map,
            .prev_smooth_state = prev_smooth_state,
            .parking_brake_release_time = parking_brake_release_time,
            .decider_state = &prev_decider_state,
            .initializer_state = &prev_initializer_state,
            .selected_trajectory_optimizer_state_proto =
                has_selected_trajectory_optimizer_state_proto
                    ? (&selected_trajectory_optimizer_state_proto)
                    : nullptr,
            .st_planner_object_trajectories = &prev_st_planner_trajectories,
            .selector_state = &prev_selector_state,
            .traffic_light_states = &traffic_light_states,
            .log_av_trajectory = &log_av_trajectory,
            .prediction_debug = &prediction_debug,
            .planner_model_pool = planner_model_pool,
            .real_objects = real_objects,
            .virtual_objects = virtual_objects,
            .planner_av_context = planner_av_context,
            .context_feature = context_feature,
            .is_standwait = is_standwait};
        return RunMultiTasksCruisePlanner(
            multi_task_cruise_planner_input, future_multi_task_est_result.get(),
            FLAGS_planner_multi_est_in_parallel ? thread_pool : nullptr);
      });
}

void ModifySpeedFinderParamsStyle(LaneChangeStage lc_stage,
                                  LaneChangeStyle lc_style,
                                  const PlannerParamsProto& planner_params,
                                  SpeedFinderParamsProto* speed_finder_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (lc_style) {
      case LC_STYLE_NORMAL:
        break;
      case LC_STYLE_RADICAL:
        *speed_finder_params = planner_params.speed_finder_lc_radical_params();
        break;
      case LC_STYLE_CONSERVATIVE:
        *speed_finder_params =
            planner_params.speed_finder_lc_conservative_params();
        break;
    }
  }
}

PlannerStatus RunDeciderAndSpeedPlanner(
    const MultiTasksCruisePlannerInput& input,
    const PlannerSemanticMapManager& low_freq_psmm,
    const DrivingMapTopo& driving_map_topo,
    const std::vector<PathPoint>& prev_st_path_points_global,
    const std::vector<PathPoint>& prev_st_path_points,
    const std::optional<RouteTargetInfo>& route_target_info,
    const EstPlannerOutput& est_output, AsyncPlannerOutput* output,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("RunDeciderAndSpeedPlanner");

  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const auto& vehicle_model = vehicle_params.vehicle_params().model();

  const auto& scheduler_output = est_output.scheduler_output;
  const auto& drive_passage = scheduler_output.drive_passage;
  const auto& sl_boundary = scheduler_output.sl_boundary;
  const auto& ego_frenet_box = scheduler_output.av_frenet_box_on_drive_passage;

  const auto& planner_params = *input.planner_params;
  const auto& decision_config = planner_params.decision_constraint_config();
  const auto& motion_constraint_params =
      planner_params.motion_constraint_params();
  const auto& plan_start_point = input.start_point_info->start_point;
  const auto& pre_decider_state = *input.decider_state;

  // Build spacetime trajectory manager (with filters).
  const auto st_traj_mgr = BuildSpacetimeTrajectoryManager(
      SpacetimeTrajectoryManagerBuilderInput{
          .passage = &drive_passage,
          .sl_boundary = &sl_boundary,
          .obj_mgr = input.object_manager.get(),
          .on_vision_map = low_freq_psmm.IsOnVisionMap()},
      thread_pool);

  // Export filtered object to debug message.
  output->filtered_prediction_trajectories.Clear();
  output->filtered_prediction_trajectories.mutable_filtered()->Reserve(
      st_traj_mgr.ignored_trajectories().size());
  for (const auto& ignored : st_traj_mgr.ignored_trajectories()) {
    auto* filtered = output->filtered_prediction_trajectories.add_filtered();
    filtered->set_reason(ignored.reason);
    filtered->set_id(ignored.object_id);
    filtered->set_index(ignored.traj->index());
  }

  // Run decider
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geom_params,
      .motion_constraint_params = &motion_constraint_params,
      .config = &decision_config,
      .planner_semantic_map_manager = &low_freq_psmm,
      .lc_state = &scheduler_output.lane_change_state,
      .plan_start_point = &plan_start_point,
      .target_offset_from_start = scheduler_output.target_offset_from_start,
      .lane_path_before_lc = &scheduler_output.lane_path_before_lc,
      .passage = &drive_passage,
      .sl_boundary = &sl_boundary,
      .ego_frenet_box = &ego_frenet_box,
      .borrow_lane_boundary = scheduler_output.borrow_lane,
      .obj_mgr = input.object_manager.get(),
      .st_traj_mgr = &st_traj_mgr,
      .tl_info_map = input.tl_info_map,
      .pre_decider_state = input.decider_state,
      .parking_brake_release_time = input.parking_brake_release_time,
      .teleop_enable_traffic_light_stop =
          input.ext_cmd_status->enable_traffic_light_stopping,
      .enable_pull_over = input.ext_cmd_status->enable_pull_over,
      .brake_to_stop = input.ext_cmd_status->brake_to_stop,
      .max_reach_length = scheduler_output.max_reach_length,
      .vehicle_model = vehicle_model,
      .plan_time = input.start_point_info->plan_time,
      .scene_reasoning = input.scene_reasoning,
      .enable_stop_polyline_stopping = false,
      .is_engage_steer_only = false,
      .enable_force_stop = input.is_standwait};
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);
  auto& mutable_decider_state = decider_output.decider_state;
  auto& mutable_constraint_manager = decider_output.constraint_manager;
  output->distance_to_traffic_light_stop_line =
      decider_output.distance_to_traffic_light_stop_line;

  // Run traffic gap finder only in high freq.
  if (decision_config.enable_prepare_lc() && route_target_info.has_value()) {
    if (FLAGS_planner_use_traffic_gap_finder_v2) {
      constexpr double kLaneSpeedLimitPreviewTime = 4.0;  // s.
      const double speed_limit =
          drive_passage
              .QuerySpeedLimitAtS(ego_frenet_box.center_s() +
                                  plan_start_point.v() *
                                      kLaneSpeedLimitPreviewTime)
              .value_or(DBL_MAX);

      double max_reach_length = scheduler_output.max_reach_length;
      if (route_target_info->merge_point.has_value()) {
        const auto merge_point_sl_or = drive_passage.QueryFrenetCoordinateAt(
            *route_target_info->merge_point);
        if (merge_point_sl_or.ok()) {
          const double length_to_merge_point =
              merge_point_sl_or->s - ego_frenet_box.center_s();
          max_reach_length =
              std::max(0.0, std::min(max_reach_length, length_to_merge_point));
        }
      }
      TrafficGapDebugProto debug_info;
      auto traffic_gap = FindBestTrafficGapOnRouteTarget(
          *drive_passage.frenet_frame(), ego_frenet_box, st_traj_mgr,
          route_target_info->frenet_frame, route_target_info->ego_frenet_box,
          route_target_info->st_traj_mgr, plan_start_point.v(),
          plan_start_point.a(), max_reach_length, speed_limit,
          pre_decider_state.traffic_gap_state().leader_id(),
          pre_decider_state.traffic_gap_state().follower_id(), &debug_info);
      traffic_gap.ToProto(mutable_decider_state.mutable_traffic_gap_state());
      mutable_constraint_manager.SetTrafficGap(std::move(traffic_gap));
      mutable_constraint_manager.SetTrafficGapDebug(std::move(debug_info));
    } else {
      const auto traffic_gaps = FindCandidateTrafficGapsOnLanePath(
          route_target_info->frenet_frame, route_target_info->ego_frenet_box,
          route_target_info->st_traj_mgr);
      auto gap_or = EvaluateAndTakeBestTrafficGap(
          traffic_gaps, route_target_info->ego_frenet_box,
          plan_start_point.v());
      if (gap_or.ok()) {
        mutable_constraint_manager.SetTrafficGap(std::move(gap_or).value());
      }
    }
  }
  output->decider_state = std::move(mutable_decider_state);

  // Calculate aligned path.
  std::vector<PathPoint> aligned_st_path_points;
  if (FLAGS_planner_enable_cross_iteration_tf &&
      !prev_st_path_points_global.empty()) {
    std::vector<PathPoint> temp_prev_st_path_points;
    ConvertPreviousPathToCurrentSmooth(*input.coordinate_converter,
                                       prev_st_path_points_global,
                                       &temp_prev_st_path_points);
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        aligned_st_path_points,
        AlignPathWithPlanStartPoint(temp_prev_st_path_points,
                                    input.start_point_info->start_point),
        PlannerStatusProto::PATH_EXTENSION_FAILED);
  } else {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        aligned_st_path_points,
        AlignPathWithPlanStartPoint(prev_st_path_points,
                                    input.start_point_info->start_point),
        PlannerStatusProto::PATH_EXTENSION_FAILED);
  }
  const auto aligned_path = DiscretizedPath::CreateResampledPath(
      aligned_st_path_points, kPathSampleInterval);

  // Run speed.
  const SpeedFinderInput speed_input{
      .base_name = "async_high_freq",
      .driving_map_topo = &driving_map_topo,
      .psmm = &low_freq_psmm,
      .traj_mgr = &st_traj_mgr,
      .constraint_mgr = &mutable_constraint_manager,
      .leading_trajs = &est_output.leading_trajs,
      .follower_set = &est_output.follower_set,
      .drive_passage = &drive_passage,
      .path_sl_boundary = &sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &aligned_path,
      .st_path_points = &aligned_st_path_points,
      .time_aligned_prev_traj = input.time_aligned_prev_traj,
      // speed planning always starts at plan start point
      .plan_start_v = input.start_point_info->start_point.v(),
      .plan_start_a = input.start_point_info->start_point.a(),
      .plan_start_j = input.start_point_info->start_point.j(),
      .plan_time = input.start_point_info->plan_time,
      .planner_av_context = input.planner_av_context,
      .real_objects = input.real_objects.get(),
      .virtual_objects = input.virtual_objects.get(),
      .planner_model_pool = input.planner_model_pool,
      .run_act_net_speed_decision =
          IsRunModeL4() && FLAGS_planner_enable_act_net_speed,
  };

  // Modify style settings for StPathPlanner.
  auto speed_finder_params = planner_params.speed_finder_params();
  if (FLAGS_planner_enable_lc_style_params) {
    ModifySpeedFinderParamsStyle(scheduler_output.lane_change_state.stage(),
                                 input.ext_cmd_status->lane_change_style,
                                 *input.planner_params, &speed_finder_params);
  }

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geom_params, vehicle_drive_params,
                motion_constraint_params, speed_finder_params, thread_pool),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  // Fill speed out to est output.
  vis::vantage::ChartDataBundleProto* chart_data = &output->chart_data;
  chart_data->add_charts()->Swap(&speed_output.st_graph_chart);
  chart_data->add_charts()->Swap(&speed_output.vt_graph_chart);
  chart_data->add_charts()->Swap(&speed_output.prediction_vt_chart);
  chart_data->add_charts()->Swap(&speed_output.traj_chart);
  if (FLAGS_planner_send_speed_path_chart_data) {
    chart_data->add_charts()->Swap(&speed_output.path_chart);
  }
  if (FLAGS_planner_send_interactive_speed_to_chart) {
    chart_data->add_charts()->Swap(&speed_output.sampling_dp_chart);
    chart_data->add_charts()->Swap(&speed_output.interactive_speed_chart);
  }
  output->speed_finder_debug.CopyFrom(speed_output.speed_finder_proto);
  FillDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                  &output->decision_constraints);
  // Move final trajectory to output.
  output->traj_points = std::move(speed_output.trajectory_points);
  output->considered_st_objects = std::move(speed_output.considered_st_objects);
  output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  // For hmi display.
  output->alerted_front_vehicle = est_output.alerted_front_vehicle;
  output->collision_warning_request = est_output.collision_warning_request;

  output->traj_validation_result.Clear();
  const bool valid = ValidateEstTrajectory(
      low_freq_psmm, output->considered_st_objects,
      input.start_point_info->full_stop, scheduler_output, vehicle_geom_params,
      vehicle_drive_params, motion_constraint_params, output->traj_points,
      &output->traj_validation_result, thread_pool);
  if (!valid) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat("Validation failed: ",
                     output->traj_validation_result.DebugString()));
  }

  return OkPlannerStatus();
}

PlannerStatus TryRetrieveLowFreqResult(bool is_cruise_planner_async,
                                       bool must_retrieve_result, int counter,
                                       AsyncPlannerOutput* output,
                                       AsyncPlannerState* async_planner_state) {
  SCOPED_QTRACE("RetrieveLowFreqResult");

  // --------- Retrieve the latest path ---------
  if (!async_planner_state->future_multi_task_est_status.IsValid() &&
      async_planner_state->latest_multi_task_est_result == nullptr) {
    return PlannerStatus(
        PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
        "The latest multi-task est planner is not available yet.");
  }

  const bool is_low_freq_ready =
      async_planner_state->future_multi_task_est_status.IsReady();
  if (!IsDSimMode() && !is_low_freq_ready && must_retrieve_result &&
      is_cruise_planner_async) {
    QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
            QIssueSubType::QIST_PLANNER_PROCESS_TIMEOUT,
            absl::StrCat("Async cruise low-freq module did not finish in ",
                         FLAGS_planner_max_cruise_async_iterations,
                         " iterations."));
  }

  if (!is_low_freq_ready && is_cruise_planner_async) {
    QLOG(WARNING) << absl::StrCat(
        "Async cruise low-freq module should be ready but is still running "
        "in ",
        counter, " iterations.");
    QEVENT_EVERY_N_SECONDS("zixuan", "cruise_low_freq_timeout",
                           /*seconds=*/1.0, [](QEvent* qevent) {
                             qevent->AddField(
                                 "expected_iter",
                                 FLAGS_planner_async_low_freq_cycle_iterations);
                           });
  }

  if (must_retrieve_result || !is_cruise_planner_async || is_low_freq_ready) {
    auto est_status = async_planner_state->future_multi_task_est_status.Get();

    output->retrived_low_freq_output =
        std::move(async_planner_state->future_multi_task_est_result);

    async_planner_state->future_multi_task_est_result = nullptr;
    // Since result has been moved, status should be reset correspondingly.

    DestroyContainerAsyncMarkSource(
        std::move(async_planner_state->future_multi_task_est_status),
        "future_multi_task_est_planner");
    async_planner_state->future_multi_task_est_status = Future<PlannerStatus>();
    output->low_freq_result_retrived = true;

    if (!est_status.ok()) {
      UpdateAsyncCounterAfterLowFreqRetrieved(async_planner_state);
      async_planner_state->latest_multi_task_est_result = nullptr;
      return PlannerStatus(
          est_status.status_code(),
          absl::StrCat("The latest multi-task est planner failed: ",
                       est_status.message()));
    }

    auto& mutable_multi_task_est_result = *output->retrived_low_freq_output;
    {
      // TODO(weijun): no copy.
      SCOPED_QTRACE("CreateLatestMultiTaskEstResult");
      async_planner_state->latest_multi_task_est_result =
          std::make_shared<AsyncMultiTaskEstOutput>(AsyncMultiTaskEstOutput{
              .est_status = est_status,
              .est_output =
                  mutable_multi_task_est_result.est_planner_output_list[0],
              .est_debug =
                  mutable_multi_task_est_result.est_planner_debug_list[0],
              .st_path_points_global_including_past =
                  mutable_multi_task_est_result
                      .st_path_points_global_including_past,
              .route_target_info =
                  std::move(mutable_multi_task_est_result.route_target_info),
              .plc_result = std::move(mutable_multi_task_est_result.plc_result),
              .nudge_object_info =
                  std::move(mutable_multi_task_est_result.nudge_object_info),
              .low_freq_psmm = mutable_multi_task_est_result.low_freq_psmm,
              .driving_map_topo =
                  mutable_multi_task_est_result.driving_map_topo,
              .selector_state = mutable_multi_task_est_result.selector_state,
              .selector_output = mutable_multi_task_est_result.selector_output,
              .alc_state = mutable_multi_task_est_result.alc_state,
              .st_path_points_including_past =
                  mutable_multi_task_est_result.st_path_points_including_past});
    }

    UpdateAsyncCounterAfterLowFreqRetrieved(async_planner_state);
  }
  return OkPlannerStatus();
}

}  // namespace

PlannerStatus RunAsyncCruisePlanner(
    const MultiTasksCruisePlannerInput& input,
    const QACCTaskProto& acc_task_proto,
    std::optional<AccPlannerOutput>* acc_output_ptr, AsyncPlannerOutput* output,
    AsyncPlannerState* async_planner_state, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  const int counter = async_planner_state->secondary_counter.has_value()
                          ? *async_planner_state->secondary_counter
                          : async_planner_state->counter;

  if (ShouldRunLowFreqModule(counter)) {
    SCOPED_QTRACE("ScheduleToRunLowFreqModule");
    if (IsPlannerAsync(FLAGS_planner_async_low_freq_cycle_iterations)) {
      output->scheduled_async_low_freq = true;
    }
    // --------- Run low-freq module ---------
    const bool update_path_route_sections =
        async_planner_state->wait_path_switch_route ||
        async_planner_state->task_transition ||
        async_planner_state->secondary_counter.has_value();
    async_planner_state->wait_path_switch_route = false;
    async_planner_state->low_freq_smooth_result_map = *input.smooth_result_map;
    async_planner_state->low_freq_tl_info_map = *input.tl_info_map;
    async_planner_state->future_multi_task_est_result =
        std::make_shared<PathBoundedEstPlannerOutput>();
    async_planner_state->future_multi_task_est_status =
        ScheduleFutureMultiTaskEst(
            input, async_planner_state->future_multi_task_est_result,
            thread_pool, update_path_route_sections);
    if (!async_planner_state->future_multi_task_est_status.IsValid()) {
      async_planner_state->latest_multi_task_est_result = nullptr;
      return PlannerStatus(
          PlannerStatusProto::LOW_FREQ_SCHEDULE_FUTURE_FAILED,
          "Failed to schedule future for async cruise planner.");
    }
    async_planner_state->pending_lane_change_command =
        DriverAction::LC_CMD_NONE;
    async_planner_state->pending_alc_confirmation = std::nullopt;
  }

  const bool should_retrieve_result =
      (!async_planner_state->task_transition &&
       ShouldRetrieveLowFreqResult(
           counter, FLAGS_planner_async_low_freq_cycle_iterations)) ||
      (async_planner_state->task_transition &&
       ShouldRetrieveLowFreqResult(
           async_planner_state->counter,
           FLAGS_planner_alcc_async_low_freq_cycle_iterations));

  const int max_cruise_iter =
      std::max(FLAGS_planner_max_cruise_async_iterations,
               FLAGS_planner_async_low_freq_cycle_iterations);
  const int max_alcc_iter =
      std::max(FLAGS_planner_max_alcc_async_iterations,
               FLAGS_planner_alcc_async_low_freq_cycle_iterations);

  const bool must_retrieve_result =
      (!async_planner_state->task_transition &&
       MustRetrieveLowFreqResult(counter, max_cruise_iter)) ||
      (async_planner_state->task_transition &&
       MustRetrieveLowFreqResult(async_planner_state->counter, max_alcc_iter));
  const bool is_cruise_planner_async =
      IsPlannerAsync(FLAGS_planner_async_low_freq_cycle_iterations);
  output->low_freq_result_retrived = false;

  if (should_retrieve_result) {
    auto status =
        TryRetrieveLowFreqResult(is_cruise_planner_async, must_retrieve_result,
                                 counter, output, async_planner_state);

    if (!status.ok()) {
      return status;
    }
  }

  if (ShouldRunAccPlannerInAsyncPlanner(*async_planner_state)) {
    AccPlannerOutput acc_output;
    const auto& ext_cmd_status = *input.ext_cmd_status;
    auto acc_planner_status = RunAccPlanner(
        AccTaskInput{
            .planner_semantic_map_manager =
                input.planner_semantic_map_manager.get(),
            .pose = input.pose,
            .steering_percentage =
                input.chassis == nullptr
                    ? std::nullopt
                    : std::make_optional(input.chassis->steering_percentage()),
            .acc_params = &input.planner_params->acc_params(),
            .vehicle_geometry_params =
                &(input.vehicle_params->vehicle_geometry_params()),
            .vehicle_drive_params =
                &(input.vehicle_params->vehicle_drive_params()),
            .plan_start_point_info = input.start_point_info,
            .plan_time = input.start_point_info->plan_time,
            .st_traj_mgr = input.st_traj_mgr.get(),
            .prev_trajectory = input.previous_trajectory,
            .time_aligned_prev_traj = input.time_aligned_prev_traj,
            .is_acc_standwait = input.is_standwait,
            .average_kappa = input.planner_av_context->GetAvKappaCacheAverage(),
            .acc_task_proto = &acc_task_proto,
            .lcc_cruising_speed_limit = ext_cmd_status.noa_cruising_speed_limit,
            .following_distance_level = ext_cmd_status.following_distance_level,
        },
        &acc_output);
    *acc_output_ptr = std::move(acc_output);
    return acc_planner_status;
  }

  if (async_planner_state->latest_multi_task_est_result == nullptr) {
    return PlannerStatus(
        PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
        "No valid multi-task est planner result has arrived yet.");
  }

  output->latest_low_freq_output =
      async_planner_state->latest_multi_task_est_result;

  if (!is_cruise_planner_async) {
    output->speed_considered_objects_prediction =
        CollectSpeedConsideredObjectsPrediction(
            *input.object_manager,
            output->latest_low_freq_output->est_output.considered_st_objects,
            FLAGS_planner_export_all_prediction_to_speed_considered);

    return output->latest_low_freq_output->est_status;
  }

  AsyncMultiTaskEstOutput& latest_est_result =
      *async_planner_state->latest_multi_task_est_result;
  const auto low_freq_psmm = latest_est_result.low_freq_psmm != nullptr
                                 ? latest_est_result.low_freq_psmm
                                 : input.planner_semantic_map_manager;

  // Refresh members related to ego pose and others.
  if (const auto update_status = UpdateLowFreqEstResult(
          *low_freq_psmm, *input.route_sections_from_start,
          *input.object_manager, input.start_point_info->start_point,
          input.vehicle_params->vehicle_geometry_params(),
          *input.prev_target_lane_path, *input.prev_lane_path_before_lc,
          async_planner_state->low_freq_smooth_result_map,
          async_planner_state->low_freq_tl_info_map, input.prev_smooth_state,
          input.autonomy_state, &latest_est_result, thread_pool);
      !update_status.ok()) {
    return PlannerStatus(PlannerStatusProto::SCHEDULER_UNAVAILABLE,
                         update_status.message());
  }

  // -------------------------------------
  // --------- Run Speed Planner ---------
  // -------------------------------------
  output->est_status = RunDeciderAndSpeedPlanner(
      input, *low_freq_psmm, *latest_est_result.driving_map_topo,
      latest_est_result.st_path_points_global_including_past,
      latest_est_result.st_path_points_including_past,
      latest_est_result.route_target_info, latest_est_result.est_output, output,
      thread_pool);

  output->speed_considered_objects_prediction =
      CollectSpeedConsideredObjectsPrediction(
          *input.object_manager, output->considered_st_objects,
          FLAGS_planner_export_all_prediction_to_speed_considered);

  return output->est_status;
}

}  // namespace planner
}  // namespace qcraft
