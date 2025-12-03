#include "onboard/planner/est_planner.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
#include <optional>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/async/async_util.h"
#include "onboard/async/thread_pool.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/hmi_util.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories_builder.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/st_path_planner.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/charts.pb.h"

namespace qcraft {
namespace planner {

namespace {
void ModifySpeedFinderParamsStyle(
    const SpeedFinderParamsProto& speed_finder_lc_radical_params,
    const SpeedFinderParamsProto& speed_finder_lc_conservative_params,
    LaneChangeStage lc_stage, LaneChangeStyle lc_style,
    SpeedFinderParamsProto* speed_finder_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (lc_style) {
      case LC_STYLE_NORMAL:
        break;
      case LC_STYLE_RADICAL:
        *speed_finder_params = speed_finder_lc_radical_params;
        break;
      case LC_STYLE_CONSERVATIVE:
        *speed_finder_params = speed_finder_lc_conservative_params;
        break;
    }
  }
}

}  // namespace

PlannerStatus RunEstPlanner(const EstPlannerInput& input,
                            SchedulerOutput scheduler_output,
                            EstPlannerOutput* est_output,
                            EstPlannerDebug* debug_info,
                            vis::vantage::ChartDataBundleProto* chart_data,
                            ThreadPool* thread_pool) {
  SCOPED_QTRACE("EstPlanner");
  QCHECK_NOTNULL(est_output);
  QCHECK_NOTNULL(debug_info);
  QCHECK_NOTNULL(chart_data);

  absl::Cleanup fill_scheduler_output = [&est_output, &scheduler_output] {
    est_output->scheduler_output = std::move(scheduler_output);
  };

  const auto& motion_constraint_params =
      *QCHECK_NOTNULL(input.motion_constraint_params);
  const auto& decision_constraint_config =
      *QCHECK_NOTNULL(input.decision_constraint_config);
  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const auto& vehicle_model = vehicle_params.vehicle_params().model();

  // Build constraint manager.
  // TODO(yumeng): Serialize this decision context if it need to be persist
  // across iterations.
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geom_params,
      .motion_constraint_params = &motion_constraint_params,
      .config = &decision_constraint_config,
      .planner_semantic_map_manager = input.planner_semantic_map_manager,
      .lc_state = &scheduler_output.lane_change_state,
      .plan_start_point = &input.start_point_info->start_point,
      .lane_path_before_lc = &scheduler_output.lane_path_before_lc,
      .passage = &scheduler_output.drive_passage,
      .sl_boundary = &scheduler_output.sl_boundary,
      .ego_frenet_box = &scheduler_output.av_frenet_box_on_drive_passage,
      .borrow_lane_boundary = scheduler_output.borrow_lane,
      .obj_mgr = input.obj_mgr,
      .st_traj_mgr = input.st_traj_mgr,
      .tl_info_map = input.tl_info_map,
      .pre_decider_state = input.decider_state,
      .parking_brake_release_time = input.parking_brake_release_time,
      .teleop_enable_traffic_light_stop = input.enable_traffic_light_stopping,
      .enable_pull_over = input.enable_pull_over,
      .brake_to_stop = input.brake_to_stop,
      .max_reach_length = scheduler_output.max_reach_length,
      .vehicle_model = vehicle_model,
      .plan_time = input.start_point_info->plan_time,
      .scene_reasoning = input.scene_reasoning,
      .enable_stop_polyline_stopping = false,
      .is_engage_steer_only = false,
      .enable_force_stop = input.enable_force_stop};
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);

  est_output->distance_to_traffic_light_stop_line =
      decider_output.distance_to_traffic_light_stop_line;

  SpacetimePlannerObjectTrajectoriesBuilderInput
      st_planner_object_traj_builder_input{
          .psmm = input.planner_semantic_map_manager,
          .passage = &scheduler_output.drive_passage,
          .sl_boundary = &scheduler_output.sl_boundary,
          .lane_change_state = &scheduler_output.lane_change_state,
          .veh_geom = &vehicle_geom_params,
          .plan_start_point = &input.start_point_info->start_point,
          .st_planner_start_offset =
              input.st_path_start_point_info
                  ->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          .prev_st_trajs = input.st_planner_object_trajectories,
          .time_aligned_prev_traj = input.time_aligned_prev_traj,
          .stop_lines = decider_output.constraint_manager.StopLine(),
          .spacetime_planner_object_trajectories_params =
              input.spacetime_planner_object_trajectories_params,
          .planner_objects = input.obj_mgr->planner_objects()};
  auto init_st_planner_object_traj = BuildSpacetimePlannerObjectTrajectories(
      st_planner_object_traj_builder_input, input.st_traj_mgr->trajectories());

  StPathPlannerInput st_path_input{
      .plan_id = input.plan_id,
      .is_run_model_l4 = input.is_run_model_l4,
      .st_path_start_point_info = input.st_path_start_point_info,
      .path_look_ahead_duration = input.st_path_start_point_info->plan_time -
                                  input.start_point_info->plan_time,
      .vehicle_params = input.vehicle_params,
      .planner_semantic_map_manager = input.planner_semantic_map_manager,
      .smooth_result_map = input.smooth_result_map,
      .scheduler_output = std::move(scheduler_output),
      .traj_mgr = input.st_traj_mgr,
      .lane_change_style = input.lane_change_style,

      // For rebuilding constraint manager on lc pause.
      .start_point_info = input.start_point_info,
      .obj_mgr = input.obj_mgr,
      .tl_info_map = input.tl_info_map,
      .prev_decider_state = input.decider_state,
      .parking_brake_release_time = input.parking_brake_release_time,
      .enable_pull_over = input.enable_pull_over,
      .enable_traffic_light_stopping = input.enable_traffic_light_stopping,
      .brake_to_stop = input.brake_to_stop,
      .enable_force_stop = input.enable_force_stop,

      .init_st_planner_object_traj = std::move(init_st_planner_object_traj),
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .decider_output = std::move(decider_output),
      .prev_target_lane_path_from_start =
          input.prev_target_lane_path_from_start,
      .time_aligned_prev_traj = input.time_aligned_prev_traj,
      .prev_initializer_state = input.initializer_state,
      .trajectory_optimizer_state_proto =
          input.trajectory_optimizer_state_proto,
      .log_av_trajectory = input.log_av_trajectory,
      .captain_net_output = input.captain_net_output,
      // Params.
      .decision_constraint_config = &decision_constraint_config,
      .initializer_params = input.initializer_params,
      .trajectory_optimizer_params = input.trajectory_optimizer_params,
      .motion_constraint_params = &motion_constraint_params,
      .planner_functions_params = input.planner_functions_params,
      .vehicle_models_params = input.vehicle_models_params,
      .trajectory_optimizer_lc_radical_params =
          input.trajectory_optimizer_lc_radical_params,
      .trajectory_optimizer_lc_normal_params =
          input.trajectory_optimizer_lc_normal_params,
      .trajectory_optimizer_lc_conservative_params =
          input.trajectory_optimizer_lc_conservative_params};
  StPathPlannerOutput path_output;
  auto path_status =
      RunStPathPlanner(std::move(st_path_input), &path_output, thread_pool);

  // Fill path out to est output.
  for (const auto& stop_line : path_output.constraint_manager.StopLine()) {
    if (stop_line.source().type_case() !=
            SourceProto::TypeCase::kEndOfPathBoundary &&
        !est_output->first_stop_s.has_value()) {
      est_output->first_stop_s = stop_line.s();
    }

    if (stop_line.source().type_case() ==
            SourceProto::TypeCase::kTrafficLight &&
        !est_output->redlight_lane_id.has_value()) {
      est_output->redlight_lane_id =
          mapping::ElementId(stop_line.source().traffic_light().lane_id());
    }
  }
  chart_data->MergeFrom(path_output.chart_data);
  est_output->unsafe_object_ids = std::move(path_output.unsafe_object_ids);
  // Set cross-frame state of the decider.
  est_output->decider_state = std::move(path_output.decider_state);
  // Set cross-frame state of the initializer.
  est_output->initializer_state = std::move(path_output.initializer_state);
  debug_info->initializer_debug_proto =
      std::move(path_output.initializer_debug_proto);
  // Optimizer state.
  est_output->trajectory_optimizer_state_proto =
      std::move(path_output.trajectory_optimizer_state_proto);
  // Optimizer Auto Tuning
  est_output->candidate_auto_tuning_traj_proto =
      std::move(path_output.candidate_auto_tuning_traj_proto);
  est_output->expert_auto_tuning_traj_proto =
      std::move(path_output.expert_auto_tuning_traj_proto);
  //  Optimizer hmi
  est_output->nudge_object_info = std::move(path_output.nudge_object_info);

  debug_info->optimizer_debug_proto =
      std::move(path_output.optimizer_debug_proto);
  debug_info->lane_change_safety_debug_proto =
      std::move(path_output.lane_change_safety_debug_proto);
  est_output->st_path_points = std::move(path_output.st_path_points);
  // Set cross-frame state of spacetime planner objects.
  for (const auto& st_planner_traj_info :
       path_output.st_planner_object_traj.trajectory_infos) {
    auto* st_planner_traj_proto =
        est_output->st_planner_object_trajectories.add_trajectory();
    st_planner_traj_proto->set_reason(st_planner_traj_info.reason);
    st_planner_traj_proto->set_id(st_planner_traj_info.object_id);
    st_planner_traj_proto->set_index(st_planner_traj_info.traj_index);
  }
  debug_info->st_planner_object_trajectories =
      est_output->st_planner_object_trajectories;
  // Export filtered object to debug message.
  debug_info->filtered_prediction_trajectories.mutable_filtered()->Reserve(
      input.st_traj_mgr->ignored_trajectories().size());
  for (const auto& ignored : input.st_traj_mgr->ignored_trajectories()) {
    auto* filtered =
        debug_info->filtered_prediction_trajectories.add_filtered();
    filtered->set_reason(ignored.reason);
    filtered->set_id(ignored.object_id);
    filtered->set_index(ignored.traj->index());
  }
  est_output->scheduler_output = std::move(path_output.scheduler_output);
  std::move(fill_scheduler_output).Cancel();
  if (!path_status.ok()) {
    FillDecisionConstraintDebugInfo(path_output.constraint_manager,
                                    &debug_info->decision_constraints);
    return path_status;
  }

  est_output->path = std::move(path_output.path);
  est_output->leading_trajs = std::move(path_output.leading_trajs);
  est_output->follower_set = std::move(path_output.follower_set);
  est_output->follower_max_decel = path_output.follower_max_decel;

  // Run speed.
  const SpeedFinderInput speed_input{
      .base_name = "est",
      .driving_map_topo = input.driving_map_topo,
      .psmm = input.planner_semantic_map_manager,
      .traj_mgr = input.st_traj_mgr,
      .constraint_mgr = &path_output.constraint_manager,
      .leading_trajs = &est_output->leading_trajs,
      .follower_set = &est_output->follower_set,
      .drive_passage = &est_output->scheduler_output.drive_passage,
      .path_sl_boundary = &est_output->scheduler_output.sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &est_output->path,
      .st_path_points = &est_output->st_path_points,
      .time_aligned_prev_traj = input.time_aligned_prev_traj,
      // speed planning always starts at plan start point
      .plan_start_v = input.start_point_info->start_point.v(),
      .plan_start_a = input.start_point_info->start_point.a(),
      .plan_start_j = input.start_point_info->start_point.j(),
      .plan_time = input.start_point_info->plan_time,
      .planner_av_context = input.planner_av_context,
      .real_objects = input.real_objects,
      .virtual_objects = input.virtual_objects,
      .planner_model_pool = input.planner_model_pool,
  };

  // Modify style settings for StPathPlanner.
  auto speed_finder_params = *input.speed_finder_params;
  if (FLAGS_planner_enable_lc_style_params) {
    ModifySpeedFinderParamsStyle(
        *input.speed_finder_lc_radical_params,
        *input.speed_finder_lc_conservative_params,
        est_output->scheduler_output.lane_change_state.stage(),
        input.lane_change_style, &speed_finder_params);
  }

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geom_params, vehicle_drive_params,
                motion_constraint_params, speed_finder_params, thread_pool),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  // Fill speed out to est output.
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
  debug_info->speed_finder_debug = std::move(speed_output.speed_finder_proto);
  FillDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                  &debug_info->decision_constraints);
  // Move final trajectory to output.
  est_output->traj_points = std::move(speed_output.trajectory_points);
  est_output->considered_st_objects =
      std::move(speed_output.considered_st_objects);
  est_output->trajectory_end_info = std::move(speed_output.trajectory_end_info);

  // Fill alerted front vehicle and collision warning for HMI.
  const auto alerted_front_vehicle = GetAlertedFrontVehicle(
      est_output->scheduler_output.drive_passage, *input.obj_mgr,
      input.start_point_info->start_point, vehicle_geom_params,
      speed_finder_params);
  if (input.planner_functions_params->enable_front_vehicle_alert() &&
      alerted_front_vehicle.has_value()) {
    est_output->alerted_front_vehicle = alerted_front_vehicle->obj_id;
  }
  est_output->collision_warning_request = GetCollisionWarningRequest(
      input.prev_collision_warning_request, alerted_front_vehicle,
      input.start_point_info->start_point.v(),
      speed_finder_params.follow_time_headway());

  const bool valid = ValidateEstTrajectory(
      *input.planner_semantic_map_manager, est_output->considered_st_objects,
      input.start_point_info->full_stop, est_output->scheduler_output,
      vehicle_geom_params, vehicle_drive_params, motion_constraint_params,
      est_output->traj_points, &debug_info->traj_validation_result,
      thread_pool);
  if (!valid) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat("Validation failed: ",
                     debug_info->traj_validation_result.DebugString()));
  }

  DestroyContainerAsyncMarkSource(std::move(path_output),
                                  "est_planner:path_output");

  return OkPlannerStatus();
}

}  // namespace planner
}  // namespace qcraft
