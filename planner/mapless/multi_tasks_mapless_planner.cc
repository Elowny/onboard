#include "onboard/planner/mapless/multi_tasks_mapless_planner.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"

#include "onboard/async/parallel_for.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/geometry/util.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/lcc_map_builder.h"
#include "onboard/planner/assist/vision_lane_path_filter.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/mapless/mapless_scheduler.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

// NOLINTNEXTLINE(readability-function-size)
PlannerStatus RunMultiTasksMaplessPlanner(
    const MultiTasksMaplessPlannerInput& input,
    PathBoundedEstPlannerOutput* output, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  const auto& vehicle_geometry =
      input.vehicle_params->vehicle_geometry_params();
  const auto& ext_cmd_status = *input.ext_cmd_status;
  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& online_map = *input.online_semantic_map;

  const auto& plan_start_point = input.plan_start_point_info->start_point;
  const auto plan_start_point_xy =
      Vec2dFromApolloTrajectoryPointProto(plan_start_point);

  auto prev_target_lane_path = *input.prev_target_lane_path;
  if (input.prev_low_freq_psmm != nullptr && !prev_target_lane_path.IsEmpty()) {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        prev_target_lane_path,
        ProjectLanePathToCurrentOnlineMap(
            psmm, online_map, *input.prev_low_freq_psmm, prev_target_lane_path,
            plan_start_point_xy, plan_start_point.v(),
            /*check_preview_length=*/0.0, thread_pool),
        PlannerStatusProto::PROJECT_TO_ONLINE_MAP_FAILED);
  }

  // Step 1: Intialization or reset
  if (prev_target_lane_path.IsEmpty() ||
      !IS_AUTO_DRIVE(input.autonomy_state->autonomy_state())) {
    // locate ego pose on semantic map and find current lane
    // path.
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        prev_target_lane_path,
        FindNearestLanePathFromEgoPose(*input.pose, psmm,
                                       kMaplessReferenceLineRequiredLength),
        PlannerStatusProto::REFERENCE_PATH_UNAVAILABLE);
    prev_target_lane_path = BackwardExtendLanePath(
        psmm, prev_target_lane_path, kDrivePassageKeepBehindLength);
  }

  // Step 2: Build driving map.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto dm,
      BuildDrivingMapByOnlineMap(psmm, online_map, plan_start_point_xy),
      PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);

  mapping::LanePath prev_target_lane_path_from_start;
  if (!prev_target_lane_path.IsEmpty()) {
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        prev_target_lane_path_from_start,
        ProjectLanePathToDrivingMap(prev_target_lane_path, dm, psmm),
        PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);
  }

  // Step 3: Find canditate target lane paths.
  const std::vector<mapping::LanePath> target_lp_vec = {
      prev_target_lane_path_from_start};

  // Step 4: Run scheduler
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto scheduler_results,
      RunMaplessScheduler(
          MaplessSchedulerInput{
              .psmm = &psmm,
              .vehicle_geom = &vehicle_geometry,
              .plan_start_point = &input.plan_start_point_info->start_point,
              .st_traj_mgr = input.st_traj_mgr.get(),
              .target_lp_vec = &target_lp_vec,
              .prev_target_lane_path = &prev_target_lane_path_from_start,
              .prev_lc_state = input.lane_change_state,
              .cruising_speed_mps = ext_cmd_status.noa_cruising_speed_limit,
              .autonomy_state = input.autonomy_state->autonomy_state(),
          },
          thread_pool),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  if (FLAGS_planner_drive_passage_debug_level) {
    for (int i = 0; i < scheduler_results.size(); ++i) {
      SendDrivePassageToCanvas(scheduler_results[i].drive_passage,
                               absl::StrFormat("mapless_drive_passage_%d", i));
    }
  }

  // Step 5: Build spacetime trajectory manager
  std::vector<SpacetimeTrajectoryManager> st_traj_mgr_lists;
  st_traj_mgr_lists.reserve(scheduler_results.size());
  const bool on_vision_map = psmm.IsOnVisionMap();
  for (int i = 0; i < scheduler_results.size(); ++i) {
    st_traj_mgr_lists.emplace_back(BuildSpacetimeTrajectoryManager(
        SpacetimeTrajectoryManagerBuilderInput{
            .passage = &scheduler_results[i].drive_passage,
            .sl_boundary = &scheduler_results[i].sl_boundary,
            .obj_mgr = input.object_manager.get(),
            .on_vision_map = on_vision_map},
        thread_pool));
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

  // Step 6: Parallel for running est planner on both branches
  std::vector<PlannerStatus> status_list(scheduler_results.size());
  std::vector<EstPlannerOutput> est_outputs(scheduler_results.size());
  std::vector<EstPlannerDebug> est_debugs(scheduler_results.size());
  std::vector<vis::vantage::ChartDataBundleProto> chart_data_bundles(
      scheduler_results.size());

  const auto& planner_params = *input.planner_params;
  ParallelFor(0, scheduler_results.size(), thread_pool, [&](int i) {
    status_list[i] = RunEstPlanner(
        EstPlannerInput{
            .driving_map_topo = &dm,
            .semantic_map_manager = psmm.semantic_map_manager(),
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
            .prev_target_lane_path_from_start =
                &prev_target_lane_path_from_start,
            .time_aligned_prev_traj = input.time_aligned_prev_traj_points,
            .lane_change_style = ext_cmd_status.lane_change_style,
            .enable_pull_over = ext_cmd_status.enable_pull_over,
            .enable_traffic_light_stopping =
                ext_cmd_status.enable_traffic_light_stopping,
            .brake_to_stop = ext_cmd_status.brake_to_stop,
            .enable_force_stop = false,
            .st_traj_mgr = &st_traj_mgr_lists[i],
            .log_av_trajectory = input.log_av_trajectory,
            .captain_net_output = &empty_captain_net_output,
            .decision_constraint_config =
                &planner_params.decision_constraint_config(),
            .initializer_params = &planner_params.initializer_params(),
            .trajectory_optimizer_params =
                &planner_params.trajectory_optimizer_params(),
            .speed_finder_params = &planner_params.speed_finder_params(),
            .motion_constraint_params =
                &planner_params.motion_constraint_params(),
            .planner_functions_params =
                &planner_params.planner_functions_params(),
            .vehicle_models_params = &planner_params.vehicle_models_params(),
            .speed_finder_lc_radical_params =
                &planner_params.speed_finder_lc_radical_params(),
            .speed_finder_lc_conservative_params =
                &planner_params.speed_finder_lc_conservative_params(),
            .trajectory_optimizer_lc_radical_params =
                &planner_params.trajectory_optimizer_lc_radical_params(),
            .trajectory_optimizer_lc_normal_params =
                &planner_params.trajectory_optimizer_lc_normal_params(),
            .trajectory_optimizer_lc_conservative_params =
                &input.planner_params
                     ->trajectory_optimizer_lc_conservative_params(),
            .spacetime_planner_object_trajectories_params =
                &input.planner_params
                     ->spacetime_planner_object_trajectories_params()},
        SchedulerOutput{
            .drive_passage = std::move(scheduler_results[i].drive_passage),
            .sl_boundary = std::move(scheduler_results[i].sl_boundary),
            .lane_change_state =
                std::move(scheduler_results[i].lane_change_state),
            .borrow_lane = scheduler_results[i].borrow_lane,
            .av_frenet_box_on_drive_passage =
                scheduler_results[i].av_frenet_box_on_drive_passage},
        &est_outputs[i], &est_debugs[i], &chart_data_bundles[i], thread_pool);
  });

  // Collect speed-considered object ids by all est planners.
  std::set<std::string> speed_considered_object_ids;
  if (FLAGS_planner_export_all_prediction_to_speed_considered) {
    for (const auto& planner_object : input.object_manager->planner_objects()) {
      speed_considered_object_ids.insert(planner_object.id());
    }
  } else {
    for (int i = 0; i < est_outputs.size(); ++i) {
      for (const auto& part_st_traj : est_outputs[i].considered_st_objects) {
        speed_considered_object_ids.insert(
            std::string(part_st_traj.st_traj().object_id()));
      }
    }
  }

  ObjectsPredictionProto speed_considered_objects_prediction;
  speed_considered_objects_prediction.mutable_objects()->Reserve(
      speed_considered_object_ids.size());
  for (const auto& object_id : speed_considered_object_ids) {
    const auto* object =
        QCHECK_NOTNULL(input.object_manager->FindObjectById(object_id));
    object->prediction().ToProto(
        speed_considered_objects_prediction.add_objects());
  }

  // Step 7: Select the better branch by checking status and lane change safety.

  output->est_status_list = std::move(status_list);
  output->est_planner_output_list = std::move(est_outputs);
  output->est_planner_debug_list = std::move(est_debugs);
  output->chart_data_list = std::move(chart_data_bundles);

  output->speed_considered_objects_prediction =
      std::move(speed_considered_objects_prediction);
  output->path_start_relative_index =
      path_start_point_info.relative_index_from_plan_start_point;

  output->online_map_id = input.online_semantic_map->update_id();
  output->low_freq_psmm = input.planner_semantic_map_manager;
  output->driving_map_topo =
      std::make_shared<const DrivingMapTopo>(std::move(dm));

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
