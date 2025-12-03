#include "onboard/planner/ml/expert/expert_planner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <utility>
#include <valarray>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"
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
#include "onboard/planner/scheduler/multi_tasks_scheduler.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> GetExpertTrajectory(
    const ExpertPlannerInput& input) {
  if (input.log_av_trajectory->trajectory_point().empty()) {
    QLOG(ERROR) << "log_av_trajectory is empty.";
    return absl::FailedPreconditionError("log_av_trajectory is empty.");
  }
  // Interpolate the trajectory from plan_time with kTrajectoryTimeStep
  const double planner_start_timestamp_sec =
      ToUnixDoubleSeconds(input.start_point_info->plan_time);
  const double log_av_trajectory_start_timestamp_sec =
      input.log_av_trajectory->trajectory_start_timestamp() +
      input.log_av_trajectory->trajectory_point(0).relative_time();
  const double log_av_trajectory_end_timestamp_sec =
      input.log_av_trajectory->trajectory_start_timestamp() +
      input.log_av_trajectory
          ->trajectory_point(input.log_av_trajectory->trajectory_point_size() -
                             1)
          .relative_time();
  if (input.log_av_trajectory->trajectory_start_timestamp() >
          planner_start_timestamp_sec ||
      input.log_av_trajectory->trajectory_start_timestamp() +
              input.log_av_trajectory
                  ->trajectory_point(
                      input.log_av_trajectory->trajectory_point_size() - 1)
                  .relative_time() <
          planner_start_timestamp_sec +
              (kTrajectorySteps - 1) * kTrajectoryTimeStep) {
    QLOG(ERROR) << absl::StrFormat(
        "log_av_trajectory time range is limited: "
        "input.log_av_trajectory start time [%f], "
        "planner_start_timestamp_sec [%f], "
        "input.log_av_trajectory end time [%f], "
        "planner_start_timestamp_sec end time [%f], ",
        log_av_trajectory_start_timestamp_sec, planner_start_timestamp_sec,
        log_av_trajectory_end_timestamp_sec,
        planner_start_timestamp_sec +
            (kTrajectorySteps - 1) * kTrajectoryTimeStep);
    return absl::FailedPreconditionError(
        "Fail to interpolate log_av_trajectory.");
  }

  std::vector<ApolloTrajectoryPointProto> expert_trajs;
  for (int i = 0; i < kTrajectorySteps; ++i) {
    const double query_t = planner_start_timestamp_sec +
                           i * kTrajectoryTimeStep -
                           log_av_trajectory_start_timestamp_sec;
    auto point = QueryApolloTrajectoryPointByT(
        input.log_av_trajectory->trajectory_point().begin(),
        input.log_av_trajectory->trajectory_point().end(), query_t);
    point.mutable_path_point()->set_theta(
        NormalizeAngle(point.path_point().theta()));
    point.set_relative_time(i * kTrajectoryTimeStep);
    expert_trajs.push_back(std::move(point));
  }

  return expert_trajs;
}

bool IsDrivePassageValid(
    const std::vector<ApolloTrajectoryPointProto>& trajectory,
    const DrivePassage& drive_passage) {
  for (const auto& point : trajectory) {
    const auto pos = Vec2dFromApolloTrajectoryPointProto(point);
    const auto s = drive_passage.FindNearestStation(pos).accumulated_s();
    ASSIGN_OR_RETURN(const auto offset_or, drive_passage.QueryCurbOffsetAtS(s),
                     false);
    ASSIGN_OR_RETURN(const auto l_or, drive_passage.QueryFrenetLatOffsetAt(pos),
                     false);
    if (l_or < offset_or.first || l_or > offset_or.second) {
      return false;
    }
  }
  return true;
}

bool IsPathSlBoundaryValid(
    const std::vector<ApolloTrajectoryPointProto>& trajectory,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary) {
  for (const auto& point : trajectory) {
    const auto pos = Vec2dFromApolloTrajectoryPointProto(point);
    ASSIGN_OR_RETURN(const auto l_or, drive_passage.QueryFrenetLatOffsetAt(pos),
                     false);
    auto s = drive_passage.FindNearestStation(pos).accumulated_s();
    auto offset = path_sl_boundary.QueryBoundaryL(s);
    if (l_or < offset.first || l_or > offset.second) {
      return false;
    }
  }
  return true;
}

LanePathInfo ChooseLeastLateralOffsetLanePath(
    const std::vector<LanePathInfo>& lp_infos,
    const std::vector<ApolloTrajectoryPointProto>& trajectory) {
  const int traj_size = trajectory.size();
  const int lp_infos_size = lp_infos.size();
  std::vector<std::valarray<double>> lp_offsets(
      lp_infos_size, std::valarray<double>(traj_size));

  for (int i = 0; i < lp_infos_size; ++i) {
    for (int j = 0; j < traj_size; ++j) {
      lp_offsets[i][j] = std::abs(
          lp_infos[i]
              .ProjectionSL(Vec2dFromApolloTrajectoryPointProto(trajectory[j]))
              .l);
    }
  }
  std::vector<double> lp_offsets_sums;
  lp_offsets_sums.reserve(lp_infos_size);
  std::for_each(lp_offsets.begin(), lp_offsets.end(),
                [&lp_offsets_sums](const auto& offsets) {
                  lp_offsets_sums.push_back(offsets.sum());
                });
  const auto min_element =
      std::min_element(lp_offsets_sums.begin(), lp_offsets_sums.end());
  return lp_infos[std::distance(lp_offsets_sums.begin(), min_element)];
}

}  // namespace

PlannerStatus RunExpertPlanner(const ExpertPlannerInput& input,
                               ExpertPlannerOutput* output,
                               ThreadPool* thread_pool) {
  SCOPED_QTRACE("ExpertPlan");

  // Input sanity checks.
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.start_point_info);
  QCHECK_NOTNULL(input.prev_target_lane_path_from_start);
  QCHECK_NOTNULL(input.sections_info_from_start);
  QCHECK_NOTNULL(input.obj_mgr);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.stalled_objects);
  QCHECK_NOTNULL(input.scene_reasoning);
  QCHECK_NOTNULL(input.traffic_light_states);
  QCHECK_NOTNULL(input.pre_decider_state);
  QCHECK_NOTNULL(input.tl_info_map);
  QCHECK_NOTNULL(input.lp_infos);
  QCHECK_NOTNULL(input.planner_params);
  QCHECK_NOTNULL(input.vehicle_params);
  QCHECK_NOTNULL(input.station_anchor);
  QCHECK_NOTNULL(input.smooth_result_map);
  QCHECK_NOTNULL(input.prev_route_sections);

  // Load expert trajectory from file.
  auto trajectory_or = GetExpertTrajectory(input);
  if (!trajectory_or.ok()) {
    return PlannerStatus(PlannerStatusProto::EXPERT_TRAJ_UNAVAILABLE,
                         "Expert traj reading failed.");
  }
  output->trajectory_points = std::move(*trajectory_or);

  const auto& plan_start_point = input.start_point_info->start_point;
  const auto& psmm = *input.psmm;
  const auto& vehicle_geometry_params =
      input.vehicle_params->vehicle_geometry_params();
  const auto& vehicle_drive_params =
      input.vehicle_params->vehicle_drive_params();

  const auto target_lane_path_info = ChooseLeastLateralOffsetLanePath(
      *input.lp_infos, output->trajectory_points);
  const auto& target_lane_path = target_lane_path_info.lane_path();

  const auto clamped_route_section = ClampRouteSectionsBeforeArcLength(
      *input.psmm, *input.prev_route_sections,
      kDrivePassageKeepBehindLength + kMaxTravelDistanceBetweenFrames);
  if (!clamped_route_section.ok()) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        "Drive passage reconstruction for expert trajectory failed.");
  }
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(*input.psmm, *clamped_route_section,
                                            target_lane_path,
                                            kDrivePassageKeepBehindLength);
  if (!backward_extended_lane_path.ok()) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        "Drive passage reconstruction for expert trajectory failed.");
  }
  auto drive_passage_or = BuildDrivePassage(
      *input.psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
      *backward_extended_lane_path, *input.station_anchor,
      input.sections_info_from_start->planning_horizon(),
      input.sections_info_from_start->destination(),
      FLAGS_planner_consider_all_lanes_virtual,
      /*override_speed_limit_mps=*/input.cruising_speed_limit);
  if (!drive_passage_or.ok()) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        "Drive passage reconstruction for expert trajectory failed.");
  }

  if (!IsDrivePassageValid(output->trajectory_points, *drive_passage_or)) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        "Drive passage not valid for expert trajectory.");
  }

  const bool should_smooth = ShouldSmoothRefLane(
      *input.tl_info_map, *drive_passage_or, input.prev_smooth_state);

  auto no_borrow_scheduler_output_or = MakeSchedulerOutput(
      *input.psmm, *input.sections_info_from_start, *input.lp_infos,
      *drive_passage_or, target_lane_path_info, vehicle_geometry_params,
      *input.st_traj_mgr, input.start_point_info->start_point,
      *input.smooth_result_map, *input.prev_target_lane_path_from_start,
      *input.prev_lane_path_before_lc_from_start, *input.prev_lc_state,
      *input.route_navi_info, /*borrow=*/false, should_smooth,
      /*planner_is_l4_mode=*/true, input.autonomy_state);

  if (no_borrow_scheduler_output_or.ok() &&
      IsPathSlBoundaryValid(output->trajectory_points, *drive_passage_or,
                            no_borrow_scheduler_output_or->sl_boundary)) {
    output->scheduler_output = *no_borrow_scheduler_output_or;
  } else {
    auto borrow_scheduler_output_or = MakeSchedulerOutput(
        *input.psmm, *input.sections_info_from_start, *input.lp_infos,
        *drive_passage_or, target_lane_path_info, vehicle_geometry_params,
        *input.st_traj_mgr, input.start_point_info->start_point,
        *input.smooth_result_map, *input.prev_target_lane_path_from_start,
        *input.prev_lane_path_before_lc_from_start, *input.prev_lc_state,
        *input.route_navi_info, /*borrow=*/true, should_smooth,
        /*planner_is_l4_mode=*/true, input.autonomy_state);
    if (borrow_scheduler_output_or.ok() &&
        IsPathSlBoundaryValid(output->trajectory_points, *drive_passage_or,
                              borrow_scheduler_output_or->sl_boundary)) {
      output->scheduler_output = *borrow_scheduler_output_or;
    } else {
      return PlannerStatus(
          PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
          "Scheduler output reconstruction for expert trajectory failed.");
    }
  }

  const auto& drive_passage = output->scheduler_output.drive_passage;
  const auto& path_sl_boundary = output->scheduler_output.sl_boundary;
  output->scheduler_output.is_expert = true;

  // Filter the space-time object trajectories.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const DrivePassageFilter drive_passage_filter(
      &drive_passage, &path_sl_boundary, psmm.IsOnVisionMap());
  output->filtered_traj_mgr = SpacetimeTrajectoryManager(
      absl::Span<const TrajectoryFilter* const>(
          {&low_likelihood_filter, &drive_passage_filter}),
      input.obj_mgr->planner_objects(), thread_pool);

  // Build constraint manager.
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geometry_params,
      .motion_constraint_params =
          &input.planner_params->motion_constraint_params(),
      .config = &input.planner_params->decision_constraint_config(),
      .planner_semantic_map_manager = &psmm,
      .lc_state = &output->scheduler_output.lane_change_state,
      .plan_start_point = &plan_start_point,
      .lane_path_before_lc = &output->scheduler_output.lane_path_before_lc,
      .passage = &drive_passage,
      .sl_boundary = &path_sl_boundary,
      .ego_frenet_box =
          &output->scheduler_output.av_frenet_box_on_drive_passage,
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
      .vehicle_model = input.vehicle_params->vehicle_params().model(),
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

  // TODO(Jinyun): Skip path extension when speed is too low.
  constexpr double kRequiredMinPathLength = 10.0;
  const double max_curvature =
      ComputeCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto path_extension_output,
      ExtendPathAndDeleteUnreasonablePart(output->trajectory_points,
                                          /*min_length=*/kRequiredMinPathLength,
                                          max_curvature),
      PlannerStatusProto::PATH_EXTENSION_FAILED);
  auto extended_path = DiscretizedPath::CreateResampledPath(
      path_extension_output, kPathSampleInterval);

  // Build driving map by route sections.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto route_section_in_horizon,
      ClampRouteSectionsBeforeArcLength(
          psmm, *input.sections_info_from_start->route_sections(),
          input.sections_info_from_start->planning_horizon()),
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
      .base_name = "expert_speed_recalculated",
      .driving_map_topo = &driving_map_topo,
      .psmm = &psmm,
      .traj_mgr = &output->filtered_traj_mgr,
      .constraint_mgr = &decider_output.constraint_manager,
      .leading_trajs = &leading_trajs,
      .follower_set = &follower_set,
      .drive_passage = &drive_passage,
      .path_sl_boundary = &path_sl_boundary,
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
                input.planner_params->motion_constraint_params(),
                input.planner_params->speed_finder_params(), thread_pool),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  output->considered_st_objects = std::move(speed_output.considered_st_objects);
  output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  output->path = std::move(extended_path);
  output->st_path_points = std::move(path_extension_output);

  TrajectoryValidationResultProto traj_validation;
  if (!ValidateEstTrajectory(
          psmm, output->considered_st_objects,
          input.start_point_info->full_stop, output->scheduler_output,
          vehicle_geometry_params, vehicle_drive_params,
          input.planner_params->motion_constraint_params(),
          speed_output.trajectory_points, &traj_validation, thread_pool)) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_SPEED_IN_CONTRADICTORY_WITH_SPEED_FINDER,
        absl::StrCat("Validation failed: ", traj_validation.DebugString()));
  }

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
