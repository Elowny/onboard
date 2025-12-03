#include "onboard/planner/planner_state_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <deque>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "glog/logging.h"

#include "common/proto/lane_point.pb.h"
#include "common/proto/qalc.pb.h"

#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/async/thread_pool.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/plan_task_helper.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/proto/scheduler.pb.h"
#include "onboard/planner/scheduler/proto/smooth_reference_line.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/selector/proto/selector_state.pb.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

// TODO(zixuan): move it to path boundary builder
absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromProto(
    const DrivePassage& drive_passage,
    const ScheduledPathBoundaryProto& path_boundary_proto) {
  const int n = path_boundary_proto.left_boundary().size();
  if (n > drive_passage.size()) {
    return absl::FailedPreconditionError(
        "Invalid path boundary proto: longer than drive passage.");
  }

  std::vector<Vec2d> left_boundary_xy, right_boundary_xy, reference_center_xy,
      target_left_boundary_xy, target_right_boundary_xy;
  std::vector<double> s_vec, left_l, right_l, ref_center_l, target_left_l,
      target_right_l;
  left_boundary_xy.reserve(n);
  right_boundary_xy.reserve(n);
  reference_center_xy.reserve(n);
  target_left_boundary_xy.reserve(n);
  target_right_boundary_xy.reserve(n);
  s_vec.reserve(n);
  left_l.reserve(n);
  right_l.reserve(n);
  ref_center_l.reserve(n);
  target_left_l.reserve(n);
  target_right_l.reserve(n);

  for (int i = 0; i < n; ++i) {
    left_boundary_xy.emplace_back(
        Vec2dFromProto(path_boundary_proto.left_boundary(i)));
    right_boundary_xy.emplace_back(
        Vec2dFromProto(path_boundary_proto.right_boundary(i)));
    reference_center_xy.emplace_back(
        Vec2dFromProto(path_boundary_proto.reference_center(i)));
    target_left_boundary_xy.emplace_back(
        Vec2dFromProto(path_boundary_proto.target_left_boundary(i)));
    target_right_boundary_xy.emplace_back(
        Vec2dFromProto(path_boundary_proto.target_right_boundary(i)));
    s_vec.push_back(drive_passage.station(StationIndex(i)).accumulated_s());

    FrenetCoordinate coord;
    ASSIGN_OR_BREAK(coord, drive_passage.QueryUnboundedFrenetCoordinateAt(
                               left_boundary_xy.back()));
    left_l.push_back(coord.l);

    ASSIGN_OR_BREAK(coord, drive_passage.QueryUnboundedFrenetCoordinateAt(
                               right_boundary_xy.back()));
    right_l.push_back(coord.l);

    ASSIGN_OR_BREAK(coord, drive_passage.QueryUnboundedFrenetCoordinateAt(
                               reference_center_xy.back()));
    ref_center_l.push_back(coord.l);

    ASSIGN_OR_BREAK(coord, drive_passage.QueryUnboundedFrenetCoordinateAt(
                               target_left_boundary_xy.back()));
    target_left_l.push_back(coord.l);

    ASSIGN_OR_BREAK(coord, drive_passage.QueryUnboundedFrenetCoordinateAt(
                               target_right_boundary_xy.back()));
    target_right_l.push_back(coord.l);
  }

  return PathSlBoundary(s_vec, ref_center_l, right_l, left_l, target_right_l,
                        target_left_l, reference_center_xy, right_boundary_xy,
                        left_boundary_xy, target_right_boundary_xy,
                        target_left_boundary_xy);
}

absl::StatusOr<DiscretizedPath> RecoverPathFromProto(
    const ::google::protobuf::RepeatedPtrField<PathPoint>& traj_pts,
    bool should_resample) {
  if (traj_pts.empty()) {
    return absl::FailedPreconditionError(
        "Empty trajectry: unable to recover path.");
  }

  std::vector<PathPoint> points;
  points.reserve(traj_pts.size());
  for (const auto& pt : traj_pts) {
    points.push_back(pt);
  }

  if (!should_resample) {
    return DiscretizedPath(std::move(points));
  }

  points[0].set_s(0.0);
  const double s_offset =
      points[1].s() - Vec2d(points[1].x(), points[1].y())
                          .DistanceTo(Vec2d(points[0].x(), points[0].y()));
  for (int i = 1; i < points.size(); ++i) {
    points[i].set_s(points[i].s() - s_offset);
  }
  DiscretizedPath path(std::move(points));

  double s = 0.0;
  DiscretizedPath resampled_path;
  resampled_path.reserve(CeilToInt(path.length() / kPathSampleInterval));
  while (s < path.length()) {
    resampled_path.push_back(path.Evaluate(s));
    s += kPathSampleInterval;
  }

  return resampled_path;
}

absl::StatusOr<DiscretizedPath> RecoverPathFromProto(
    const ::google::protobuf::RepeatedPtrField<ApolloTrajectoryPointProto>&
        traj_pts,
    bool should_resample) {
  if (traj_pts.empty()) {
    return absl::FailedPreconditionError(
        "Empty trajectry: unable to recover path.");
  }

  std::vector<PathPoint> points;
  points.reserve(traj_pts.size());
  for (const auto& pt : traj_pts) {
    points.push_back(pt.path_point());
  }

  if (!should_resample) {
    return DiscretizedPath(std::move(points));
  }

  points[0].set_s(0.0);
  const double s_offset =
      points[1].s() - Vec2d(points[1].x(), points[1].y())
                          .DistanceTo(Vec2d(points[0].x(), points[0].y()));
  for (int i = 1; i < points.size(); ++i) {
    points[i].set_s(points[i].s() - s_offset);
  }
  DiscretizedPath path(std::move(points));

  double s = 0.0;
  DiscretizedPath resampled_path;
  resampled_path.reserve(CeilToInt(path.length() / kPathSampleInterval));
  while (s < path.length()) {
    resampled_path.push_back(path.Evaluate(s));
    s += kPathSampleInterval;
  }

  return resampled_path;
}

absl::Status RecoverAsyncStateFromProto(const PlannerDebugProto& planner_debug,
                                        const PlannerSemanticMapManager& psmm,
                                        PlannerState* state) {
  if (planner_debug.est_planner_debugs().empty()) {
    return absl::FailedPreconditionError("The prev debug proto is empty");
  }
  const auto& selected_est_debug = planner_debug.est_planner_debugs(0);

  PlannerStatus est_status;
  est_status.FromProto(selected_est_debug.planner_status());

  // Planning horizon.
  double planning_horizon;
  if (planner_debug.has_planning_horizon()) {
    planning_horizon = planner_debug.planning_horizon();
  } else {
    if (!selected_est_debug.has_speed_finder() ||
        selected_est_debug.speed_finder().trajectory().empty()) {
      return absl::FailedPreconditionError(
          "No speed trajectory available as plan start point.");
    }

    const auto plan_start_point = Vec2dFromApolloTrajectoryPointProto(
        selected_est_debug.speed_finder().trajectory(0));
    ASSIGN_OR_RETURN(
        auto route_sections_proj,
        ProjectPointToRouteSections(
            psmm, state->prev_route_sections, plan_start_point,
            kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
            kDrivePassageKeepBehindLength));
    planning_horizon = std::get<0>(route_sections_proj).planning_horizon(psmm);
  }

  // Scheduler.
  std::vector<SchedulerOutput> scheduler_outputs;
  scheduler_outputs.reserve(planner_debug.est_planner_debugs().size());
  for (const auto& est_debug_proto : planner_debug.est_planner_debugs()) {
    auto& scheduler_output = scheduler_outputs.emplace_back();

    // TODO(zixuan): For backward compatibility, use
    // trajectory_proto.target_lane_path_from_current
    if (!est_debug_proto.has_scheduler() ||
        !est_debug_proto.scheduler().has_target_lane_path()) {
      continue;
    }

    const auto& scheduler_proto = est_debug_proto.scheduler();
    if (scheduler_proto.has_lc_state()) {
      scheduler_output.lane_change_state = scheduler_proto.lc_state();
    }
    if (scheduler_proto.has_length_along_route()) {
      scheduler_output.length_along_route =
          scheduler_proto.length_along_route();
    }
    if (scheduler_proto.has_max_reach_length()) {
      scheduler_output.max_reach_length = scheduler_proto.max_reach_length();
    }
    if (scheduler_proto.has_should_smooth()) {
      scheduler_output.should_smooth = scheduler_proto.should_smooth();
    }
    if (scheduler_proto.has_lane_path_before_lc()) {
      scheduler_output.lane_path_before_lc.FromProto(
          psmm.semantic_map_manager(), scheduler_proto.lane_path_before_lc());
    }

    mapping::LanePath target_lane_path;
    target_lane_path.FromProto(psmm.semantic_map_manager(),
                               scheduler_proto.target_lane_path());

    mapping::LanePath backward_extended_lane_path;
    if (scheduler_proto.has_backward_extended_lane_path() &&
        planner_debug.has_planning_horizon()) {
      backward_extended_lane_path.FromProto(
          psmm.semantic_map_manager(),
          scheduler_proto.backward_extended_lane_path());
    } else {
      ASSIGN_OR_CONTINUE(
          const auto clamped_route_section,
          ClampRouteSectionsBeforeArcLength(
              psmm, state->prev_route_sections,
              kDrivePassageKeepBehindLength + kMaxTravelDistanceBetweenFrames));
      ASSIGN_OR_CONTINUE(backward_extended_lane_path,
                         BackwardExtendLanePathOnRouteSections(
                             psmm, clamped_route_section, target_lane_path,
                             kDrivePassageKeepBehindLength));
    }

    const auto anchor_point =
        scheduler_proto.has_anchor_point()
            ? mapping::LanePoint(scheduler_proto.anchor_point())
            : state->station_anchor;

    ASSIGN_OR_CONTINUE(
        scheduler_output.drive_passage,
        BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
                          backward_extended_lane_path, anchor_point,
                          planning_horizon,
                          state->prev_route_sections.destination(),
                          FLAGS_planner_consider_all_lanes_virtual));

    ASSIGN_OR_CONTINUE(
        scheduler_output.sl_boundary,
        BuildPathBoundaryFromProto(scheduler_output.drive_passage,
                                   scheduler_proto.path_boundary()));
  }

  if (scheduler_outputs.empty() || scheduler_outputs[0].sl_boundary.IsEmpty()) {
    return absl::InternalError("Failed to recover scheduler from proto.");
  }

  // Route target info.
  // TODO(zixuan): recover all route_target_info members.
  std::optional<RouteTargetInfo> route_target_info = std::nullopt;

  auto est_output =
      EstPlannerOutput{.scheduler_output = std::move(scheduler_outputs[0])};

  // Path and st path points.
  if (!selected_est_debug.st_path_points().empty()) {
    ASSIGN_OR_RETURN(est_output.path,
                     RecoverPathFromProto(selected_est_debug.st_path_points(),
                                          /*should_resample=*/false));
    est_output.st_path_points = {selected_est_debug.st_path_points().begin(),
                                 selected_est_debug.st_path_points().end()};
  } else {
    if (!selected_est_debug.has_speed_finder() ||
        selected_est_debug.speed_finder().trajectory().empty()) {
      return absl::FailedPreconditionError(
          "No speed trajectory available as path.");
    }
    ASSIGN_OR_RETURN(
        est_output.path,
        RecoverPathFromProto(selected_est_debug.speed_finder().trajectory(),
                             /*should_resample=*/true));
    est_output.st_path_points = {est_output.path.begin(),
                                 est_output.path.end()};
  }

  const auto& leading_trajs =
      selected_est_debug.initializer().leading_objects();
  est_output.leading_trajs.clear();
  for (const auto& traj : leading_trajs) {
    est_output.leading_trajs.emplace(traj.traj_id(), traj);
  }
  const auto& follower_set =
      selected_est_debug.initializer().follower_objects();
  est_output.follower_set.clear();
  for (const auto& object_id : follower_set) {
    est_output.follower_set.insert(object_id);
  }

  // est debug
  auto est_debug = EstPlannerDebug{
      .st_planner_object_trajectories =
          selected_est_debug.st_planner_object_trajectories(),
      .filtered_prediction_trajectories = selected_est_debug.filtered(),
      .decision_constraints = selected_est_debug.constraint(),
      .initializer_debug_proto = selected_est_debug.initializer(),
      .optimizer_debug_proto = selected_est_debug.trajectory_optimizer(),
      .capnet_traj_debug = selected_est_debug.capnet_traj(),
      .speed_finder_debug = selected_est_debug.speed_finder(),
      .traj_validation_result = selected_est_debug.traj_validation()};

  std::vector<PathPoint> st_path_points_global_including_past;
  st_path_points_global_including_past.reserve(
      planner_debug.st_path_points_global_including_past_size());

  for (const auto& path_point :
       planner_debug.st_path_points_global_including_past()) {
    st_path_points_global_including_past.push_back(path_point);
  }

  // Driving map.
  ASSIGN_OR_RETURN(const auto route_section_in_horizon,
                   ClampRouteSectionsBeforeArcLength(
                       psmm, state->prev_route_sections, planning_horizon));
  ASSIGN_OR_RETURN(auto driving_map_topo, BuildDrivingMapByRouteOnOfflineMap(
                                              psmm, route_section_in_horizon));

  if (state->async_planner_state.counter + 1 ==
      FLAGS_planner_async_low_freq_cycle_iterations) {
    state->async_planner_state.future_multi_task_est_status =
        ScheduleFuture(static_cast<ThreadPool*>(nullptr),
                       [&est_status]() -> PlannerStatus { return est_status; });
    state->async_planner_state.future_multi_task_est_result =
        std::make_shared<PathBoundedEstPlannerOutput>(
            PathBoundedEstPlannerOutput{
                .est_status_list = {std::move(est_status)},
                .est_planner_output_list = {std::move(est_output)},
                .est_planner_debug_list = {std::move(est_debug)},
                .chart_data_list = {vis::vantage::ChartDataBundleProto()},
                .st_path_points_global_including_past =
                    std::move(st_path_points_global_including_past),
                .route_target_info = std::move(route_target_info),
                .driving_map_topo = std::make_shared<const DrivingMapTopo>(
                    std::move(driving_map_topo))});
  } else {
    state->async_planner_state.latest_multi_task_est_result =
        std::make_shared<AsyncMultiTaskEstOutput>(AsyncMultiTaskEstOutput{
            .est_status = std::move(est_status),
            .est_output = std::move(est_output),
            .est_debug = std::move(est_debug),
            .st_path_points_global_including_past =
                std::move(st_path_points_global_including_past),
            .route_target_info = std::move(route_target_info),
            .driving_map_topo = std::make_shared<const DrivingMapTopo>(
                std::move(driving_map_topo))});
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<PlannerState> RecoverPlannerStateFromProto(
    const PlannerInput& input, bool recover_async_state) {
  if (input.route_mgr_output == nullptr) {
    return absl::FailedPreconditionError(
        "route output message is not available.");
  }
  if (input.planner_state_proto == nullptr) {
    return absl::FailedPreconditionError("The planner_state_proto is empty");
  }

  const auto& state_proto = *input.planner_state_proto;
  const auto& planner_semantic_map_manager =
      *input.planner_semantic_map_manager;
  const auto& semantic_map_manager =
      *planner_semantic_map_manager.semantic_map_manager();
  const auto& vehicle_geom = input.vehicle_params.vehicle_geometry_params();

  PlannerState state;
  state.FromProto(state_proto);
  bool can_upgrade = state.Upgrade();
  if (!can_upgrade) {
    VLOG(1) << "Cannot upgrade, old version: " << state.version;
  }
  state.prev_lane_path_before_lc.FromProto(
      &semantic_map_manager, state_proto.prev_lane_path_before_lc());
  state.prev_target_lane_path.FromProto(&semantic_map_manager,
                                        state_proto.prev_target_lane_path());
  state.preferred_lane_path.FromProto(&semantic_map_manager,
                                      state_proto.preferred_lane_path());

  state.smooth_result_map.Clear();
  if (state_proto.has_smooth_result_map()) {
    for (const auto& lane_id_vec :
         state_proto.smooth_result_map().lane_id_vec()) {
      std::vector<mapping::ElementId> lane_ids;
      lane_ids.reserve(lane_id_vec.lane_id_size());
      for (const auto& lane_id : lane_id_vec.lane_id()) {
        lane_ids.push_back(mapping::ElementId(lane_id));
      }
      const double half_av_width = vehicle_geom.width() * 0.5;
      auto smoothed_result = SmoothLanePathByLaneIds(
          planner_semantic_map_manager, lane_ids, half_av_width);
      if (smoothed_result.ok()) {
        state.smooth_result_map.AddResult(lane_ids,
                                          std::move(smoothed_result).value());
      }
    }
  }

  // Backward compatibility: No plan task queue in earlier log.
  if (state_proto.plan_task_queue().empty()) {
    RouteManagerOutput route_output;
    route_output.FromProto(*input.route_mgr_output);
    state.plan_task_queue = CreatePlanTasksQueueFromRoutingResult(
        route_output, planner_semantic_map_manager);
  }

  if (recover_async_state) {
    if (input.prev_planner_debug.est_planner_debugs().empty()) {
      return absl::FailedPreconditionError("The prev debug proto is empty");
    }
    RETURN_IF_ERROR(RecoverAsyncStateFromProto(
        input.prev_planner_debug, planner_semantic_map_manager, &state));
  }

  return state;
}

void ResetAlccAssistPlanState(AssistPlanStateProto* assist_plan_state) {
  assist_plan_state->set_alc_state(QALCState::ALC_STANDBY_ENABLE);
  assist_plan_state->set_lc_direction(LaneChangeDirection::LCD_NONE);
  assist_plan_state->clear_origin_lane_path();
  assist_plan_state->clear_target_lane_path();
  assist_plan_state->clear_lane_change_target_point();
}

void ResetAccAssistPlanState(AssistPlanStateProto* assist_plan_state) {
  assist_plan_state->clear_acc_task();
}

PlannerState::HdMapState ObtainHdMapState(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& section_seq_proto) {
  PlannerState::HdMapState hd_map_state;
  hd_map_state.load_distance = CalcRouteSectionsLength(smm, section_seq_proto);
  hd_map_state.has_destination = false;
  if (section_seq_proto.has_destination()) {
    const auto destination = smm.FindLane(
        mapping::ElementId(section_seq_proto.destination().lane_id()));
    hd_map_state.has_destination = (destination != nullptr);
  }
  return hd_map_state;
}

}  // namespace planner
}  // namespace qcraft
