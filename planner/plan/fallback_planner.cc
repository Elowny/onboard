#include "onboard/planner/plan/fallback_planner.h"

#include <algorithm>
#include <map>
#include <optional>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/object/drive_passage_filter.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

namespace qcraft::planner {

PlannerStatus RunFallbackPlanner(
    const FallbackPlannerInput& input, const VehicleParamApi& vehicle_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const DecisionConstraintConfigProto& decision_constraint_config,
    const FallbackPlannerParamsProto& fallback_planner_params,
    FallbackPlannerOutput* output, EstPlannerDebug* debug,
    vis::vantage::ChartDataBundleProto* chart_data, ThreadPool* thread_pool) {
  SCOPED_QTRACE("FallbackPlan");

  // Input sanity checks.
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.start_point_info);
  QCHECK_NOTNULL(input.time_aligned_prev_trajectory);
  QCHECK_NOTNULL(input.prev_target_lane_path_from_start);
  QCHECK_NOTNULL(input.prev_lane_path_before_lc);
  QCHECK_NOTNULL(input.route_sections_info_from_start);
  QCHECK_NOTNULL(input.obj_mgr);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.stalled_objects);
  QCHECK_NOTNULL(input.scene_reasoning);
  QCHECK_NOTNULL(input.prev_lc_state);
  QCHECK_NOTNULL(input.traffic_light_states);
  QCHECK_NOTNULL(input.pre_decider_state);
  QCHECK_NOTNULL(input.tl_info_map);
  QCHECK_NOTNULL(input.smooth_result_map);
  QCHECK_NOTNULL(input.prev_route_sections);

  const auto& time_aligned_prev_trajectory =
      *input.time_aligned_prev_trajectory;
  if (time_aligned_prev_trajectory.size() != kTrajectorySteps) {
    return PlannerStatus(PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
                         "Previous trajectory not available.");
  }
  if (input.prev_target_lane_path_from_start->IsEmpty()) {
    return PlannerStatus(PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
                         "Previous target lane path not available.");
  }

  const auto& plan_start_point = input.start_point_info->start_point;
  const auto& psmm = *input.psmm;
  const auto& vehicle_geometry_params =
      vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto clamped_prev_target_lane_path_from_start,
      TrimTrailingNotFoundLanes(psmm, *input.prev_target_lane_path_from_start),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto clamped_route_section,
      ClampRouteSectionsBeforeArcLength(
          psmm, *input.prev_route_sections,
          kDrivePassageKeepBehindLength + kMaxTravelDistanceBetweenFrames),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto backward_extended_lane_path,
      BackwardExtendLanePathOnRouteSections(
          psmm, clamped_route_section, clamped_prev_target_lane_path_from_start,
          kDrivePassageKeepBehindLength),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  // Here we assume the drive passage's center stations are all virtual to build
  // a wide enough path boundary to contain the previous-trajectory path.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto drive_passage,
      BuildDrivePassage(
          psmm, /*vision_map_ptr=*/nullptr,
          clamped_prev_target_lane_path_from_start, backward_extended_lane_path,
          *input.station_anchor,
          input.route_sections_info_from_start->planning_horizon(),
          input.route_sections_info_from_start->destination(),
          /*all_lanes_virtual=*/true,
          /*override_speed_limit_mps=*/input.cruising_speed_limit),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto path_sl_boundary,
      BuildPathBoundaryFromPose(
          psmm, drive_passage, plan_start_point, vehicle_geometry_params,
          *input.st_traj_mgr, *input.prev_lc_state, *input.smooth_result_map,
          /*borrow_lane_boundary=*/false, input.prev_smooth_state,
          /*unsafe_object_ids=*/{}),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  // Extend previous trajectory to a path that has a minimum length.
  const double min_fallback_path_length = 10.0;  // m.
  const double max_curvature =
      ComputeCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  RETURN_PLANNER_STATUS_OR_ASSIGN(auto path_extension_output,
                                  ExtendPathAndDeleteUnreasonablePart(
                                      time_aligned_prev_trajectory,
                                      min_fallback_path_length, max_curvature),
                                  PlannerStatusProto::PATH_EXTENSION_FAILED);
  auto extended_path = DiscretizedPath::CreateResampledPath(
      path_extension_output, kPathSampleInterval);

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto ego_frenet_box,
      drive_passage.QueryFrenetBoxAt(ComputeAvBox(
          Vec2dFromApolloTrajectoryPointProto(plan_start_point),
          plan_start_point.path_point().theta(), vehicle_geometry_params)),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  // Fake a scheduler output for fallback planner, to be moved later.
  output->scheduler_output =
      SchedulerOutput{.is_fallback = true,
                      .drive_passage = std::move(drive_passage),
                      .sl_boundary = std::move(path_sl_boundary),
                      .lane_change_state = *input.prev_lc_state,
                      .lane_path_before_lc = *input.prev_lane_path_before_lc,
                      .length_along_route = input.prev_length_along_route,
                      .max_reach_length = input.prev_max_reach_length,
                      .av_frenet_box_on_drive_passage = ego_frenet_box};

  // Filter the space-time object trajectories.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const DrivePassageFilter drive_passage_filter(
      &output->scheduler_output.drive_passage,
      &output->scheduler_output.sl_boundary, psmm.IsOnVisionMap());
  output->filtered_traj_mgr = SpacetimeTrajectoryManager(
      absl::Span<const TrajectoryFilter* const>(
          {&low_likelihood_filter, &drive_passage_filter}),
      input.obj_mgr->planner_objects(), thread_pool);

  // Build constraint manager. Fallback planner and Est planner share the same
  // set of decision constraints.
  // TODO(yumeng): Serialize this decision context if it need to be persist
  // across iterations.
  // We don't have clearance check output for previous trajectory but it is ok
  // to set it to null because it will be eventually discarded in the future.
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geometry_params,
      .motion_constraint_params = &motion_constraint_params,
      .config = &decision_constraint_config,
      .planner_semantic_map_manager = &psmm,
      .lc_state = input.prev_lc_state,
      .plan_start_point = &plan_start_point,
      .lane_path_before_lc = input.prev_lane_path_before_lc,
      .passage = &output->scheduler_output.drive_passage,
      .sl_boundary = &output->scheduler_output.sl_boundary,
      .ego_frenet_box = &ego_frenet_box,
      .obj_mgr = input.obj_mgr,
      .st_traj_mgr = &output->filtered_traj_mgr,
      .tl_info_map = input.tl_info_map,
      .pre_decider_state = input.pre_decider_state,
      .parking_brake_release_time = input.parking_brake_release_time,
      .teleop_enable_traffic_light_stop =
          input.teleop_enable_traffic_light_stop,
      .enable_pull_over = input.enable_pull_over,
      .brake_to_stop = input.brake_to_stop,
      .max_reach_length = output->scheduler_output.max_reach_length,
      .vehicle_model = vehicle_params.vehicle_params().model(),
      .plan_time = input.start_point_info->plan_time,
      .scene_reasoning = input.scene_reasoning,
      .enable_stop_polyline_stopping = false,
      .is_engage_steer_only = false,
      .enable_force_stop = input.enable_force_stop};

  // To be moved later.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);
  output->decider_state = std::move(decider_output.decider_state);

  // Build driving map by route sections.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto route_section_in_horizon,
      ClampRouteSectionsBeforeArcLength(
          psmm, *input.route_sections_info_from_start->route_sections(),
          input.route_sections_info_from_start->planning_horizon()),
      PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto driving_map_topo,
      BuildDrivingMapByRouteOnOfflineMap(psmm, route_section_in_horizon),
      PlannerStatusProto::BUILD_DRIVING_MAP_FAILED);

  // Pass empty leading trajectory set to speed finder and leave the leading
  // decisions to itself.
  const std::map<std::string, ConstraintProto::LeadingObjectProto>
      leading_trajs;
  const absl::flat_hash_set<std::string> follower_set;
  SpeedFinderInput speed_input{
      .base_name = "est",
      .driving_map_topo = &driving_map_topo,
      .psmm = &psmm,
      .traj_mgr = &output->filtered_traj_mgr,
      .constraint_mgr = &decider_output.constraint_manager,
      .leading_trajs = &leading_trajs,
      .follower_set = &follower_set,
      .drive_passage = &output->scheduler_output.drive_passage,
      .path_sl_boundary = &output->scheduler_output.sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &extended_path,
      .st_path_points = &path_extension_output,
      .time_aligned_prev_traj = nullptr,
      .plan_start_v = plan_start_point.v(),
      .plan_start_a = plan_start_point.a(),
      .plan_start_j = plan_start_point.j(),
      .plan_time = input.start_point_info->plan_time,
  };

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geometry_params, vehicle_drive_params,
                motion_constraint_params,
                fallback_planner_params.speed_finder_params(), thread_pool),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  // Send charts.
  chart_data->add_charts()->Swap(&speed_output.st_graph_chart);
  chart_data->add_charts()->Swap(&speed_output.vt_graph_chart);
  chart_data->add_charts()->Swap(&speed_output.traj_chart);
  if (FLAGS_planner_send_speed_path_chart_data) {
    chart_data->add_charts()->Swap(&speed_output.path_chart);
  }
  FillDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                  &debug->decision_constraints);
  debug->speed_finder_debug = std::move(speed_output.speed_finder_proto);
  output->trajectory_points = std::move(speed_output.trajectory_points);
  output->considered_st_objects = std::move(speed_output.considered_st_objects);
  output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  output->path = std::move(extended_path);
  output->st_path_points = std::move(path_extension_output);

  if (!ValidateEstTrajectory(psmm, output->considered_st_objects,
                             input.start_point_info->full_stop,
                             output->scheduler_output, vehicle_geometry_params,
                             vehicle_drive_params, motion_constraint_params,
                             output->trajectory_points,
                             &debug->traj_validation_result, thread_pool)) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat("Validation failed: ",
                     debug->traj_validation_result.DebugString()));
  }

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
