#include "onboard/planner/mapless/async_mapless_planner.h"

#include <float.h>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
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
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/util.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/external_command_info.h"
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
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/mapless/multi_tasks_mapless_planner.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/planner_object_util.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/async_planner_util.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

absl::Status UpdateLowFreqEstResult(
    const PlannerSemanticMapManager& psmm,
    const PlannerObjectManager& object_manager,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    AsyncMultiTaskEstOutput* mutable_est_result, ThreadPool* thread_pool) {
  FUNC_QTRACE();
  auto& scheduler_output = mutable_est_result->est_output.scheduler_output;
  const auto& drive_passage = scheduler_output.drive_passage;

  const auto ego_xy = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  ASSIGN_OR_RETURN(
      const auto ego_sl, drive_passage.QueryFrenetCoordinateAt(ego_xy),
      absl::OutOfRangeError(absl::StrCat("Ego pos ", ego_xy.DebugString(),
                                         " is out of drive passage!")));
  scheduler_output.target_offset_from_start =
      ego_sl.s - drive_passage.lane_path_start_s();

  const Box2d ego_box = ComputeAvBox(
      ego_xy, plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_RETURN(
      scheduler_output.av_frenet_box_on_drive_passage,
      drive_passage.QueryFrenetBoxAt(ego_box),
      absl::OutOfRangeError(absl::StrCat("Ego box ", ego_box.DebugString(),
                                         " is out of drive passage!")));

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
    const MultiTasksMaplessPlannerInput& input,
    const std::shared_ptr<PathBoundedEstPlannerOutput>&
        future_multi_task_est_result,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

  if (!IsPlannerAsync(FLAGS_planner_mapless_async_low_freq_cycle_iterations)) {
    // Not in async mode, no need to make copies.
    return ScheduleFuture(
        thread_pool, [thread_pool, future_multi_task_est_result, &input]() {
          return RunMultiTasksMaplessPlanner(
              input, future_multi_task_est_result.get(),
              FLAGS_planner_multi_est_in_parallel ? thread_pool : nullptr);
        });
  }

  const auto& vehicle_params = *input.vehicle_params;

  return ScheduleFuture(
      thread_pool,
      [thread_pool, future_multi_task_est_result,
       psmm = input.planner_semantic_map_manager, &vehicle_params,
       pose = *input.pose, autonomy_state = *input.autonomy_state,
       planner_params = *input.planner_params,
       plan_start_point_info = *input.plan_start_point_info,
       st_traj_mgr = input.st_traj_mgr, object_manager = input.object_manager,
       online_semantic_map = input.online_semantic_map,
       path_look_ahead_time = input.path_look_ahead_time,
       parking_brake_release_time = input.parking_brake_release_time,
       st_planner_object_trajectories = *input.st_planner_object_trajectories,
       time_aligned_prev_traj_points = *input.time_aligned_prev_traj_points,
       log_av_trajectory = input.log_av_trajectory != nullptr
                               ? *input.log_av_trajectory
                               : TrajectoryProto(),
       decider_state = *input.decider_state,
       initializer_state = *input.initializer_state,
       selected_trajectory_optimizer_state_proto =
           (input.selected_trajectory_optimizer_state_proto != nullptr)
               ? *input.selected_trajectory_optimizer_state_proto
               : TrajectoryOptimizerStateProto(),
       has_selected_trajectory_optimizer_state_proto =
           (input.selected_trajectory_optimizer_state_proto != nullptr),
       previous_trajectory = *input.previous_trajectory,
       ext_cmd_status = *input.ext_cmd_status,
       prev_low_freq_psmm = input.prev_low_freq_psmm,
       prev_target_lane_path = *input.prev_target_lane_path,
       prev_lane_change_state = *input.lane_change_state]() {
        const MultiTasksMaplessPlannerInput multi_task_mapless_planner_input{
            .planner_semantic_map_manager = psmm,
            .pose = &pose,
            .autonomy_state = &autonomy_state,
            .planner_params = &planner_params,
            .vehicle_params = &vehicle_params,
            .plan_start_point_info = &plan_start_point_info,
            .st_traj_mgr = st_traj_mgr,
            .object_manager = object_manager,
            .online_semantic_map = online_semantic_map,
            .path_look_ahead_time = path_look_ahead_time,
            .parking_brake_release_time = parking_brake_release_time,
            .st_planner_object_trajectories = &st_planner_object_trajectories,
            .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
            .log_av_trajectory = &log_av_trajectory,
            .decider_state = &decider_state,
            .initializer_state = &initializer_state,
            .selected_trajectory_optimizer_state_proto =
                has_selected_trajectory_optimizer_state_proto
                    ? (&selected_trajectory_optimizer_state_proto)
                    : nullptr,
            .previous_trajectory = &previous_trajectory,
            .ext_cmd_status = &ext_cmd_status,
            .prev_low_freq_psmm = prev_low_freq_psmm,
            .prev_target_lane_path = &prev_target_lane_path,
            .lane_change_state = &prev_lane_change_state};
        return RunMultiTasksMaplessPlanner(
            multi_task_mapless_planner_input,
            future_multi_task_est_result.get(),
            FLAGS_planner_multi_est_in_parallel ? thread_pool : nullptr);
      });
}

PlannerStatus RunDeciderAndSpeedPlanner(
    const MultiTasksMaplessPlannerInput& input,
    const PlannerSemanticMapManager& low_freq_psmm,
    const DrivingMapTopo& driving_map_topo,
    const std::vector<PathPoint>& prev_st_path_points_global,
    const std::optional<RouteTargetInfo>& route_target_info,
    const EstPlannerOutput& est_output, AsyncPlannerOutput* output,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

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
  const auto& plan_start_point_info = *input.plan_start_point_info;
  const auto& plan_start_point = plan_start_point_info.start_point;
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

  const TrafficLightInfoMap empty_tl_map;
  const absl::flat_hash_set<std::string> empty_stalled_objects;
  const SceneOutputProto empty_scene_reasoning;

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
      .passage = &scheduler_output.drive_passage,
      .sl_boundary = &scheduler_output.sl_boundary,
      .ego_frenet_box = &ego_frenet_box,
      .borrow_lane_boundary = scheduler_output.borrow_lane,
      .obj_mgr = input.object_manager.get(),
      .st_traj_mgr = &st_traj_mgr,
      .tl_info_map = &empty_tl_map,
      .pre_decider_state = &pre_decider_state,
      .parking_brake_release_time = input.parking_brake_release_time,
      .teleop_enable_traffic_light_stop =
          input.ext_cmd_status->enable_traffic_light_stopping,
      .enable_pull_over = input.ext_cmd_status->enable_pull_over,
      .brake_to_stop = input.ext_cmd_status->brake_to_stop,
      .max_reach_length = scheduler_output.max_reach_length,
      .vehicle_model = vehicle_model,
      .plan_time = plan_start_point_info.plan_time,
      .scene_reasoning = &empty_scene_reasoning,
      .enable_stop_polyline_stopping = false,
      .is_engage_steer_only = false,
      .enable_force_stop = false};
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

      TrafficGapDebugProto debug_info;
      auto traffic_gap = FindBestTrafficGapOnRouteTarget(
          *drive_passage.frenet_frame(), ego_frenet_box, st_traj_mgr,
          route_target_info->frenet_frame, route_target_info->ego_frenet_box,
          route_target_info->st_traj_mgr, plan_start_point.v(),
          plan_start_point.a(), scheduler_output.max_reach_length, speed_limit,
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
    std::vector<PathPoint> prev_st_path_points;
    ConvertPreviousPathToCurrentSmooth(low_freq_psmm.coordinate_converter(),
                                       prev_st_path_points_global,
                                       &prev_st_path_points);
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        aligned_st_path_points,
        AlignPathWithPlanStartPoint(prev_st_path_points, plan_start_point),
        PlannerStatusProto::PATH_EXTENSION_FAILED);
  } else {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        aligned_st_path_points,
        AlignPathWithPlanStartPoint(est_output.st_path_points,
                                    plan_start_point),
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
      .stalled_objects = &empty_stalled_objects,
      .path = &aligned_path,
      .st_path_points = &aligned_st_path_points,
      .time_aligned_prev_traj = input.time_aligned_prev_traj_points,
      // speed planning always starts at plan start point
      .plan_start_v = plan_start_point.v(),
      .plan_start_a = plan_start_point.a(),
      .plan_start_j = plan_start_point.j(),
      .plan_time = plan_start_point_info.plan_time};

  const auto& speed_finder_params = planner_params.speed_finder_params();

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
      plan_start_point_info.full_stop, scheduler_output, vehicle_geom_params,
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

}  // namespace

PlannerStatus RunAsyncMaplessPlanner(const MultiTasksMaplessPlannerInput& input,
                                     AsyncPlannerOutput* output,
                                     AsyncPlannerState* async_planner_state,
                                     ThreadPool* thread_pool) {
  FUNC_QTRACE();

  const int counter = async_planner_state->secondary_counter.has_value()
                          ? *async_planner_state->secondary_counter
                          : async_planner_state->counter;

  if (ShouldRunLowFreqModule(counter)) {
    SCOPED_QTRACE("ScheduleToRunLowFreqModule");
    if (IsPlannerAsync(FLAGS_planner_mapless_async_low_freq_cycle_iterations)) {
      output->scheduled_async_low_freq = true;
    }
    // --------- Run low-freq module ---------
    async_planner_state->future_multi_task_est_result =
        std::make_shared<PathBoundedEstPlannerOutput>();
    async_planner_state->future_multi_task_est_status =
        ScheduleFutureMultiTaskEst(
            input, async_planner_state->future_multi_task_est_result,
            thread_pool);
    if (!async_planner_state->future_multi_task_est_status.IsValid()) {
      async_planner_state->latest_multi_task_est_result = nullptr;
      return PlannerStatus(
          PlannerStatusProto::LOW_FREQ_SCHEDULE_FUTURE_FAILED,
          "Failed to schedule future for async mapless planner.");
    }
  }

  const bool should_retrieve_result =
      (!async_planner_state->task_transition &&
       ShouldRetrieveLowFreqResult(
           counter, FLAGS_planner_mapless_async_low_freq_cycle_iterations)) ||
      (async_planner_state->task_transition &&
       ShouldRetrieveLowFreqResult(
           async_planner_state->counter,
           FLAGS_planner_alcc_async_low_freq_cycle_iterations));

  const int max_mapless_iter =
      std::max(FLAGS_planner_max_mapless_async_iterations,
               FLAGS_planner_mapless_async_low_freq_cycle_iterations);
  const int max_alcc_iter =
      std::max(FLAGS_planner_max_alcc_async_iterations,
               FLAGS_planner_alcc_async_low_freq_cycle_iterations);

  const bool must_retrieve_result =
      (!async_planner_state->task_transition &&
       MustRetrieveLowFreqResult(counter, max_mapless_iter)) ||
      (async_planner_state->task_transition &&
       MustRetrieveLowFreqResult(async_planner_state->counter, max_alcc_iter));

  output->low_freq_result_retrived = false;
  if (should_retrieve_result) {
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
    const bool is_mapless_planner_async =
        IsPlannerAsync(FLAGS_planner_mapless_async_low_freq_cycle_iterations);
    if (!IsDSimMode() && !is_low_freq_ready && must_retrieve_result &&
        is_mapless_planner_async) {
      QISSUEX(QIssueSeverity::QIS_ERROR, QIssueType::QIT_PERFORMANCE,
              QIssueSubType::QIST_PLANNER_PROCESS_TIMEOUT,
              absl::StrCat("Async mapless low-freq module did not finish in ",
                           FLAGS_planner_max_mapless_async_iterations,
                           " iterations."));
    }

    if (!is_low_freq_ready && is_mapless_planner_async) {
      QLOG(WARNING) << absl::StrCat(
          "Async mapless low-freq module should be ready but is still running "
          "in ",
          counter, " iterations.");
      QEVENT_EVERY_N_SECONDS(
          "zixuan", "mapless_low_freq_timeout",
          /*seconds=*/1.0, [](QEvent* qevent) {
            qevent->AddField(
                "expected_iter",
                FLAGS_planner_mapless_async_low_freq_cycle_iterations);
          });
    }

    if (must_retrieve_result || !is_mapless_planner_async ||
        is_low_freq_ready) {
      auto est_status = async_planner_state->future_multi_task_est_status.Get();

      output->retrived_low_freq_output =
          std::move(async_planner_state->future_multi_task_est_result);

      async_planner_state->future_multi_task_est_result = nullptr;
      // Since result has been moved, status should be reset correspondingly.

      DestroyContainerAsyncMarkSource(
          std::move(async_planner_state->future_multi_task_est_status),
          "future_multi_task_est_planner");
      async_planner_state->future_multi_task_est_status =
          Future<PlannerStatus>();
      output->low_freq_result_retrived = true;

      if (!est_status.ok()) {
        async_planner_state->latest_multi_task_est_result = nullptr;
        return PlannerStatus(
            est_status.status_code(),
            absl::StrCat("The latest multi-task est planner failed: ",
                         est_status.message()));
      }

      auto& mutable_multi_task_est_result = *output->retrived_low_freq_output;
      if (!IsRunModeL4() || is_mapless_planner_async) {
        // TODO(weijun): no copy.
        SCOPED_QTRACE("CreateLatestMultiTaskEstResult");
        async_planner_state->latest_multi_task_est_result =
            std::make_shared<AsyncMultiTaskEstOutput>(AsyncMultiTaskEstOutput{
                .est_status = est_status,
                .est_output =
                    mutable_multi_task_est_result.est_planner_output_list[0],
                .est_debug =
                    mutable_multi_task_est_result.est_planner_debug_list[0],
                .route_target_info =
                    std::move(mutable_multi_task_est_result.route_target_info),
                .low_freq_psmm = mutable_multi_task_est_result.low_freq_psmm,
                .driving_map_topo =
                    mutable_multi_task_est_result.driving_map_topo,
                .st_path_points_including_past =
                    mutable_multi_task_est_result
                        .st_path_points_including_past});
      }

      if (async_planner_state->secondary_counter.has_value()) {
        async_planner_state->counter = kAsyncCounterInitVal;
        async_planner_state->secondary_counter = std::nullopt;
      } else if (async_planner_state->task_transition) {
        async_planner_state->task_transition = false;
        async_planner_state->secondary_counter = kAsyncCounterInitVal;
      } else {
        async_planner_state->counter = kAsyncCounterInitVal;
      }

      if (!is_mapless_planner_async) {
        // Not in async mode, return directly.
        return est_status;
      }
    }
  }

  if (async_planner_state->latest_multi_task_est_result == nullptr) {
    return PlannerStatus(
        PlannerStatusProto::LOW_FREQ_RESULT_NOT_YET_AVAILABLE,
        "No valid multi-task est planner result has arrived yet.");
  }

  output->latest_low_freq_output =
      async_planner_state->latest_multi_task_est_result;

  AsyncMultiTaskEstOutput& latest_est_result =
      *async_planner_state->latest_multi_task_est_result;
  const auto low_freq_psmm = latest_est_result.low_freq_psmm != nullptr
                                 ? latest_est_result.low_freq_psmm
                                 : input.planner_semantic_map_manager;

  // TODO(weijun): do not use mutable latest est result.
  // Refresh members related to ego pose and others.
  if (const auto update_status = UpdateLowFreqEstResult(
          *low_freq_psmm, *input.object_manager,
          input.plan_start_point_info->start_point,
          input.vehicle_params->vehicle_geometry_params(), &latest_est_result,
          thread_pool);
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
