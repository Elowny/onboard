#include "onboard/planner/freespace/freespace_planner.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ostream>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/types/span.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/freespace/freespace_constraint_builder.h"
#include "onboard/planner/freespace/freespace_stop_finder.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/path_manager.h"
#include "onboard/planner/freespace/tob_path_smoother.h"
#include "onboard/planner/object/freespace_region_filter.h"
#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/speed/freespace_speed_finder.h"
#include "onboard/planner/speed/freespace_speed_finder_input.h"
#include "onboard/planner/speed/freespace_speed_finder_output.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
// NOTE: Functions below are copied from freespace_planner.cc.
struct StopSInfo {
  double stop_s = 0.0;
  double stationary_object_stop_s = 0.0;
  std::string nearest_stationary_object_id = "";
};

ApolloTrajectoryPointProto MaybeProcessAndResetPlanStartPoint(
    const ApolloTrajectoryPointProto& external_plan_start_point,
    const PoseProto& pose, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params, bool external_reset,
    ResetReasonProto::Reason external_reset_reason, bool is_forward,
    bool is_new_path) {
  auto start_point = external_plan_start_point;
  // Clamp plan start v by current path direction if it was reset.
  if (external_reset &&
      external_reset_reason != ResetReasonProto::LAT_ERROR_TOO_LARGE) {
    start_point.set_v(is_forward ? std::max(0.0, pose.vel_body().x())
                                 : std::min(0.0, pose.vel_body().x()));
  }

  // Reset plan start point on receiving a new path. This can happen when path
  // finder replans, or path manager switches to next directional path.
  if (is_new_path) {
    start_point = ComputePlanStartPointAfterReset(
        /*prev_reset_planned_point=*/std::nullopt, pose, chassis,
        motion_constraint_params, veh_geo_params, veh_drive_params,
        /*is_forward_task=*/is_forward);
  }

  return start_point;
}

std::vector<ApolloTrajectoryPointProto>
GenerateStationaryTrajectoryWithRefKappa(const PathPoint& path_point,
                                         double ref_kappa, bool is_forward) {
  PathPoint start_path_point = path_point;
  if (!is_forward) {
    ref_kappa = -ref_kappa;
  }
  start_path_point.set_kappa(ref_kappa);
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = start_path_point;
    traj_point.set_relative_time(i * kTrajectoryTimeStep);
    traj_points.push_back(std::move(traj_point));
  }
  return traj_points;
}

DirectionalPath GenerateStationaryPathWithRefKappa(const PathPoint& path_point,
                                                   double ref_kappa,
                                                   bool is_forward,
                                                   bool /*is_stationary*/) {
  PathPoint start_path_point = path_point;
  start_path_point.set_s(0.0);
  start_path_point.set_kappa(ref_kappa);
  start_path_point.set_theta(is_forward
                                 ? path_point.theta()
                                 : NormalizeAngle(path_point.theta() + M_PI));

  return DirectionalPath{DiscretizedPath({start_path_point}), is_forward};
}

// TODO(opt): Pass by end parking gear instead of task type.
Chassis::GearPosition GenerateOutputGearPosition(
    FreespaceTaskProto::TaskType task_type,
    const PathManagerOutput& path_output,
    PathManagerStateProto::DriveState drive_state,
    const PoseProto& /*ego_pose*/, bool safe_stop, bool is_stationary) {
  Chassis::GearPosition gear_position;

  if (safe_stop && is_stationary) {
    gear_position = Chassis::GEAR_PARKING;
    return gear_position;
  }

  switch (drive_state) {
    case PathManagerStateProto::UNKNOWN: {
      QLOG(FATAL) << "Unexpected UNKNOWN state.";
    }
    case PathManagerStateProto::SWITCHING_TO_NEXT:
    case PathManagerStateProto::DRIVING: {
      gear_position = path_output.path.forward ? Chassis::GEAR_DRIVE
                                               : Chassis::GEAR_REVERSE;
      break;
    }
    case PathManagerStateProto::REACH_FINAL_GOAL:
    case PathManagerStateProto::CENTER_STEER: {
      switch (task_type) {
        case FreespaceTaskProto::UNKNOWN_TASK:
          QLOG(FATAL) << "Unexpected UNKNOWN_TASK.";
        case FreespaceTaskProto::PERPENDICULAR_PARKING:
        case FreespaceTaskProto::PARALLEL_PARKING:
        case FreespaceTaskProto::CUSTOM_PARKING:
          gear_position = Chassis::GEAR_PARKING;
          break;
        case FreespaceTaskProto::THREE_POINT_TURN:
        case FreespaceTaskProto::DRIVING_TO_LANE:
        case FreespaceTaskProto::FREE_DRIVING:
          gear_position = Chassis::GEAR_DRIVE;
          break;
      }
      break;
    }
  }
  return gear_position;
}

absl::StatusOr<DirectionalPath> ExtendLocalPath(
    const DirectionalPath& directional_path) {
  if (directional_path.path.empty()) {
    return absl::InternalError("Discretized path empty.");
  }
  std::vector<PathPoint> raw_path_points = {directional_path.path.begin(),
                                            directional_path.path.end()};
  constexpr double kExtendPathLength = 5.0;         // m.
  constexpr double kExtendPathPointIntervel = 0.1;  // m.
  raw_path_points.reserve(
      raw_path_points.size() +
      CeilToInt(kExtendPathLength / kExtendPathPointIntervel));
  const auto& last_path_point = directional_path.path.back();
  for (double s = kExtendPathPointIntervel; s < kExtendPathLength;
       s += kExtendPathPointIntervel) {
    raw_path_points.push_back(GetPathPointAlongCircle(last_path_point, s));
  }
  // Resample raw path to final path.
  DiscretizedPath raw_path(std::move(raw_path_points));
  double s = 0.0;
  std::vector<PathPoint> path_points;
  path_points.reserve(CeilToInt(raw_path.length() / kPathSampleInterval));
  while (s < raw_path.length()) {
    path_points.push_back(raw_path.Evaluate(s));
    s += kPathSampleInterval;
  }

  DirectionalPath extend_directional_path;
  extend_directional_path.path = DiscretizedPath(std::move(path_points));
  extend_directional_path.forward = directional_path.forward;

  return extend_directional_path;
}

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateDrivingOutputTrajectory(
    const DirectionalPath& smooth_directional_path,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerClusterObjectManager& cluster_obj_mgr,
    const PlannerSemanticMapManager* psmm,
    const ConstraintManager& constraint_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
        stalled_cluster_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::Time& plan_time,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, StopSInfo* stop_s_info,
    SpeedFinderDebugProto* speed_finder_debug,
    vis::vantage::ChartsDataProto* charts_data, ThreadPool* thread_pool) {
  // Run a speed finder on the local smooth path.
  FreespaceSpeedFinderInput speed_input;
  speed_input.planner_semantic_map_manager = psmm;
  speed_input.obj_mgr = &st_traj_mgr;
  speed_input.cluster_obj_mgr = &cluster_obj_mgr;
  speed_input.constraint_mgr = &constraint_mgr;
  speed_input.stalled_objects = &stalled_objects;
  speed_input.stalled_cluster_objects = &stalled_cluster_objects;
  speed_input.path = &smooth_directional_path.path;
  speed_input.forward = smooth_directional_path.forward;
  speed_input.plan_start_v = plan_start_point.v();
  speed_input.plan_start_a = plan_start_point.a();
  speed_input.plan_start_j = plan_start_point.j();
  speed_input.plan_time = plan_time;
  ASSIGN_OR_RETURN(
      auto speed_output,
      FindFreespaceSpeed(speed_input, vehicle_geometry_params,
                         vehicle_drive_params, vehicle_model_params,
                         motion_constraint_params, speed_finder_params,
                         thread_pool));
  stop_s_info->stop_s = speed_output.stop_s;
  stop_s_info->stationary_object_stop_s = speed_output.stationary_object_stop_s;
  stop_s_info->nearest_stationary_object_id =
      std::move(speed_output.nearest_stationary_object_id);
  charts_data->add_charts()->Swap(&speed_output.st_graph_chart);
  charts_data->add_charts()->Swap(&speed_output.vt_graph_chart);
  charts_data->add_charts()->Swap(&speed_output.traj_chart);
  speed_finder_debug->CopyFrom(speed_output.speed_finder_proto);
  return speed_output.trajectory_points;
}

absl::StatusOr<StopSInfo> GenerateDrivingOutputStopS(
    const DirectionalPath& smooth_directional_path,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerClusterObjectManager& cluster_obj_mgr,
    const PlannerSemanticMapManager* psmm,
    const ConstraintManager& constraint_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
        stalled_cluster_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const absl::Time& plan_time,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedFinderDebugProto* speed_finder_debug,
    vis::vantage::ChartsDataProto* charts_data, ThreadPool* thread_pool) {
  // Run a stop finder on the local smooth path.
  ASSIGN_OR_RETURN(
      auto stop_output,
      FindFreespaceStop(
          smooth_directional_path.path, smooth_directional_path.forward,
          plan_start_point.v(), plan_time, psmm, st_traj_mgr, cluster_obj_mgr,
          constraint_mgr, stalled_objects, stalled_cluster_objects,
          vehicle_geometry_params, vehicle_model_params,
          motion_constraint_params, speed_finder_params, thread_pool));
  charts_data->add_charts()->Swap(&stop_output.st_graph_chart);
  speed_finder_debug->CopyFrom(stop_output.stop_finder_debug);
  return StopSInfo{
      .stop_s = stop_output.stop_s,
      .stationary_object_stop_s = stop_output.stationary_object_stop_s,
      .nearest_stationary_object_id =
          std::move(stop_output.nearest_stationary_object_id)};
}

void UpdatePathUnsafeAndBlockedTime(
    const PoseProto& /*ego_pose*/,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const std::string& nearest_stationary_object_id,
    double stationary_object_stop_s, double path_end_s, bool force_stop,
    bool is_stationary, double time_interval,
    FreespacePlannerStateProto* state) {
  constexpr double kCloseToObstacleThres = 0.1;  // m.
  constexpr double kCloseToPathEndThres = 0.2;   // m.

  // Check if path unsafe.
  if (!force_stop && stalled_objects.contains(nearest_stationary_object_id) &&
      stationary_object_stop_s < path_end_s - kCloseToPathEndThres &&
      state->path_manager_state().drive_state() ==
          PathManagerStateProto::DRIVING) {
    state->set_path_unsafe_time(state->path_unsafe_time() + time_interval);
  } else {
    state->set_path_unsafe_time(0.0);
  }

  // Check if path block.
  if (is_stationary && stationary_object_stop_s < kCloseToObstacleThres &&
      path_end_s > kCloseToPathEndThres &&
      state->path_manager_state().drive_state() ==
          PathManagerStateProto::DRIVING) {
    state->set_path_blocked_time(state->path_blocked_time() + time_interval);
  } else {
    state->set_path_blocked_time(0.0);
  }
}

inline bool IsPathBlocked(double path_blocked_time) {
  constexpr int kPathBlockedTimeThres = 5.0;  // s.
  return path_blocked_time > kPathBlockedTimeThres;
}

FreespaceReplanReasonProto::ReplanReason MaybeReplan(
    bool new_task, FreespaceTaskProto::TaskType task_type,
    const VehicleGeometryParamsProto& /*veh_geo_params*/,
    const FreespacePathFinderParamsProto& /*path_finder_params*/,
    const VehicleOctagonModelParamsProto& /*vehicle_model_params*/,
    absl::Span<const SpacetimeObjectTrajectory* const> /*stalled_object_trajs*/,
    const FreespaceMap& /*freespace_map*/, const PoseProto& ego_pose,
    const PathPoint& goal, double /*time_interval*/, bool is_stationary,
    FreespacePlannerStateProto* state, FreespaceReplanReasonProto* debug_info) {
  constexpr int kReplanPathUnsafeTimeThres = 1.0;  // s.
  FreespaceReplanReasonProto::ReplanReason replan_reason =
      FreespaceReplanReasonProto::NONE;
  if (new_task) {
    replan_reason = FreespaceReplanReasonProto::NEW_TASK;
  } else if (state->path_manager_state().paths().size() == 0) {
    // 1. If we have a valid goal but haven't a plan yet, replan.
    replan_reason = FreespaceReplanReasonProto::NO_PATHS;
  } else if (state->path_manager_state().paths().size() > 0 && is_stationary &&
             state->path_unsafe_time() >= kReplanPathUnsafeTimeThres) {
    // 2. Replan if path is not safe.
    replan_reason = FreespaceReplanReasonProto::PATH_NOT_SAFE;
    state->set_path_unsafe_time(0.0);
  } else if (task_type == FreespaceTaskProto::PARALLEL_PARKING &&
             state->path_manager_state().curr_path_idx() + 1 ==
                 state->path_manager_state().paths().size() &&
             state->path_manager_state().drive_state() ==
                 PathManagerStateProto::SWITCHING_TO_NEXT &&
             is_stationary) {
    // 3. Replan before executing the last path of parallel parking.
    const double distance_to_goal = Hypot(ego_pose.pos_smooth().x() - goal.x(),
                                          ego_pose.pos_smooth().y() - goal.y());
    constexpr double kReplanMaxDistanceToGoal = 2.0;  // m
    if (distance_to_goal < kReplanMaxDistanceToGoal) {
      replan_reason = FreespaceReplanReasonProto::PARALLEL_PARKING_FINAL_PATH;
    }
  }

  // TODO(Yunfeng): Replan if control error is big.

  debug_info->set_replan_reason(replan_reason);

  return replan_reason;
}

void FillFreespaceMapDebugProto(const FreespaceMap& freespace_map,
                                FreespaceMapDebugProto* debug_info) {
  QCHECK_NOTNULL(debug_info);
  debug_info->mutable_region()->Clear();
  debug_info->add_region(freespace_map.region.min_x());
  debug_info->add_region(freespace_map.region.min_y());
  debug_info->add_region(freespace_map.region.max_x());
  debug_info->add_region(freespace_map.region.max_y());
  debug_info->mutable_boundary_ids()->Clear();
  for (const auto& boundary : freespace_map.boundaries) {
    debug_info->add_boundary_ids(boundary.id);
  }
  debug_info->mutable_special_boundaries()->Clear();
  for (const auto& boundary : freespace_map.special_boundaries) {
    if (boundary.type == FreespaceMapProto::CROSSABLE_LANE_LINE) {
      continue;
    }
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      auto seg_proto = debug_info->add_special_boundaries();
      if (boundary.points.size() == 2) {
        seg_proto->set_id(boundary.id);
      } else {
        seg_proto->set_id(absl::StrFormat("%s_%d", boundary.id, i));
      }
      seg_proto->set_type(boundary.type);
      Vec2dToProto(boundary.points[i], seg_proto->mutable_start());
      Vec2dToProto(boundary.points[i + 1], seg_proto->mutable_end());
    }
  }
}

bool CheckLowSpeed(
    bool forward, const MotionConstraintParamsProto& motion_constraint_params) {
  // Should always return true in path-stop mode.
  constexpr double kLowSpeedThreshold = 3.91;  // ~6.3km/h
  return FLAGS_planner_freespace_path_stop_mode || !forward ||
         motion_constraint_params.default_speed_limit() < kLowSpeedThreshold;
}

}  // namespace

absl::StatusOr<FreespacePlannerOutput> RunFreespacePlanner(
    const FreespacePlannerInput& input, FreespacePlannerStateProto* state,
    FreespacePlannerDebugProto* debug_info,
    vis::vantage::ChartsDataProto* charts_data, ThreadPool* thread_pool) {
  SCOPED_QTRACE("FreespacePlanner");
  const auto start_time = absl::Now();

  const auto& freespace_map = *input.freespace_map;
  FillFreespaceMapDebugProto(freespace_map,
                             debug_info->mutable_freespace_map_debug());

  // TODO(renjie): Replace with BuildSpacetimeTrajectoryManager.
  AABox2d filter_region(
      freespace_map.region.half_length() + input.veh_geo_params->length(),
      freespace_map.region.half_width() + input.veh_geo_params->length(),
      freespace_map.region.center());
  const FreespaceRegionFilter freespace_region_filter(&filter_region);
  SpacetimeTrajectoryManager st_traj_mgr(
      /*filters=*/{&freespace_region_filter}, input.obj_mgr->planner_objects(),
      thread_pool);

  const auto& stalled_object_ids = *input.stalled_object_ids;
  std::vector<const SpacetimeObjectTrajectory*> stalled_object_trajs;
  stalled_object_trajs.reserve(st_traj_mgr.trajectories().size());
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (stalled_object_ids.contains(traj.object_id())) {
      stalled_object_trajs.push_back(&traj);
    }
  }

  // Restore goal to current smooth.
  const PathPoint goal = RestoreSmoothGoalFromGlobalRef(
      state->global_goal_ref(), input.coordinate_converter,
      input.parking_spot_info);

  constexpr double kStationarySpeedThres = 0.1;  // m/s.
  const bool is_stationary =
      std::abs(input.ego_pose->vel_body().x()) < kStationarySpeedThres;

  // Check if replan.
  const auto replan_reason = MaybeReplan(
      input.new_task, state->task_type(), *input.veh_geo_params,
      input.freespace_params->path_finder_params(),
      input.vehicle_models_params->freespace_vehicle_octagon_model_params(),
      stalled_object_trajs, freespace_map, *input.ego_pose, goal,
      input.time_interval, is_stationary, state,
      debug_info->mutable_freespace_replan_reason_debug());

  ASSIGN_OR_RETURN(
      const auto path_output,
      GeneratePath(
          replan_reason, state->task_type(), *input.ego_pose, *input.chassis,
          input.freespace_params->path_finder_params(),
          input.freespace_params->sqp_smoother_params(), *input.veh_geo_params,
          *input.veh_drive_params,
          input.vehicle_models_params->freespace_vehicle_octagon_model_params(),
          stalled_object_trajs, *input.cluster_obj_mgr,
          *input.stalled_cluster_object_ids, freespace_map, goal,
          input.parking_spot_info, state->mutable_path_manager_state(),
          debug_info->mutable_path_finder_debug()));

  const auto start_point = MaybeProcessAndResetPlanStartPoint(
      *input.plan_start_point, *input.ego_pose, *input.chassis,
      input.freespace_params->motion_constraint_params(), *input.veh_geo_params,
      *input.veh_drive_params, input.start_point_reset, input.reset_reason,
      path_output.path.forward, path_output.is_new_path);

  std::vector<ApolloTrajectoryPointProto> trajectory;
  bool enable_stationary_steering = false;
  DirectionalPath smooth_directional_path;
  double stop_s = 0.0;
  double stationary_object_stop_s = std::numeric_limits<double>::infinity();
  std::string nearest_stationary_object_id = "";
  switch (state->path_manager_state().drive_state()) {
    case PathManagerStateProto::UNKNOWN: {
      QLOG(FATAL) << "Unexpected UNKNOWN state.";
    }
    case PathManagerStateProto::SWITCHING_TO_NEXT: {
      if (!FLAGS_planner_freespace_path_stop_mode) {
        trajectory = GenerateStationaryTrajectoryWithRefKappa(
            start_point.path_point(), path_output.path.path.front().kappa(),
            path_output.path.forward);
      }
      smooth_directional_path = GenerateStationaryPathWithRefKappa(
          start_point.path_point(), path_output.path.path.front().kappa(),
          path_output.path.forward, is_stationary);
      enable_stationary_steering = true;
      state->mutable_prev_local_smoother_path()->Clear();
      // TODO(yunfeng): We should also support pause function in
      // SWITCHING_TO_NEXT state.
      state->clear_prev_force_stop_point();
      break;
    }
    case PathManagerStateProto::DRIVING: {
      ASSIGN_OR_RETURN(
          smooth_directional_path,
          SmoothLocalPath(*input.veh_geo_params, *input.veh_drive_params,
                          input.freespace_params->motion_constraint_params(),
                          input.freespace_params->local_smoother_params(),
                          input.vehicle_models_params
                              ->freespace_local_smoother_vehicle_model_params(),
                          /*owner=*/"freespace_local_smoother", freespace_map,
                          st_traj_mgr, *input.cluster_obj_mgr,
                          stalled_object_ids, *input.stalled_cluster_object_ids,
                          path_output.path, TrajectoryPoint(start_point),
                          (path_output.is_new_path || input.start_point_reset),
                          {state->prev_local_smoother_path().begin(),
                           state->prev_local_smoother_path().end()},
                          debug_info->mutable_local_smoother_debug(),
                          charts_data));
      *state->mutable_prev_local_smoother_path() = {
          smooth_directional_path.path.begin(),
          smooth_directional_path.path.end()};

      ASSIGN_OR_RETURN(
          auto constraint_mgr,
          BuildFreespacePlannerConstraint(
              *input.veh_geo_params, smooth_directional_path, *input.ego_pose,
              input.force_stop || input.safe_stop, state));

      FillDecisionConstraintDebugInfo(
          constraint_mgr, debug_info->mutable_decision_constraint());

      ASSIGN_OR_RETURN(const auto extend_smooth_directional_path,
                       ExtendLocalPath(smooth_directional_path));
      if (!FLAGS_planner_freespace_path_stop_mode) {
        StopSInfo stop_s_info;
        ASSIGN_OR_RETURN(
            trajectory,
            GenerateDrivingOutputTrajectory(
                extend_smooth_directional_path, st_traj_mgr,
                *input.cluster_obj_mgr, input.psmm, constraint_mgr,
                stalled_object_ids, *input.stalled_cluster_object_ids,
                start_point, input.plan_time, *input.veh_geo_params,
                *input.veh_drive_params,
                input.vehicle_models_params
                    ->freespace_vehicle_octagon_model_params(),
                input.freespace_params->motion_constraint_params(),
                input.freespace_params->speed_finder_params(), &stop_s_info,
                debug_info->mutable_speed_finder_debug(), charts_data,
                thread_pool));
        stop_s =
            std::min(stop_s_info.stop_s, smooth_directional_path.path.length());
        stationary_object_stop_s = stop_s_info.stationary_object_stop_s;
        nearest_stationary_object_id =
            std::move(stop_s_info.nearest_stationary_object_id);
      } else {
        ASSIGN_OR_RETURN(
            auto stop_s_info,
            GenerateDrivingOutputStopS(
                extend_smooth_directional_path, st_traj_mgr,
                *input.cluster_obj_mgr, input.psmm, constraint_mgr,
                stalled_object_ids, *input.stalled_cluster_object_ids,
                start_point, input.plan_time, *input.veh_geo_params,
                input.vehicle_models_params
                    ->freespace_vehicle_octagon_model_params(),
                input.freespace_params->motion_constraint_params(),
                input.freespace_params->speed_finder_params(),
                debug_info->mutable_speed_finder_debug(), charts_data,
                thread_pool));
        stop_s =
            std::min(stop_s_info.stop_s, smooth_directional_path.path.length());
        stationary_object_stop_s = stop_s_info.stationary_object_stop_s;
        nearest_stationary_object_id =
            std::move(stop_s_info.nearest_stationary_object_id);
      }
      break;
    }
    case PathManagerStateProto::REACH_FINAL_GOAL:
    case PathManagerStateProto::CENTER_STEER: {
      // Note: In `CENTER_STEER` and `REACH_FINAL_GOAL` state, current path is
      // the last path segment.
      // TODO(yunfeng): We should also support pause function in CENTER_STEER
      // state.
      if (!FLAGS_planner_freespace_path_stop_mode) {
        trajectory = GenerateStationaryTrajectoryWithRefKappa(
            start_point.path_point(), /*ref_kappa=*/0.0,
            path_output.path.forward);
      }
      smooth_directional_path = GenerateStationaryPathWithRefKappa(
          start_point.path_point(), /*ref_kappa=*/0.0, path_output.path.forward,
          is_stationary);
      enable_stationary_steering = true;
      state->mutable_prev_local_smoother_path()->Clear();
      break;
    }
  }

  const auto freespace_plan_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  constexpr double kFreespacePlaneTimeoutThresholdMs = 100.0;  // ms.
  if (freespace_plan_time > kFreespacePlaneTimeoutThresholdMs) {
    QEVENT("yuhang", "freespace_plan_timeout", [&](QEvent* qevent) {
      qevent->AddField("freespace_plan_time[ms]", freespace_plan_time)
          .AddField("time_limit[ms]", kFreespacePlaneTimeoutThresholdMs);
    });
  }

  // TODO(renjie): Add a path validator in path-stop mode.
  if (!FLAGS_planner_freespace_path_stop_mode) {
    TrajectoryValidationResultProto traj_validation_result;
    const bool valid = ValidateFreespaceTrajectory(
        *input.veh_geo_params, *input.veh_drive_params,
        input.freespace_params->motion_constraint_params(), trajectory,
        &traj_validation_result);
    if (!valid) {
      return absl::UnavailableError(
          absl::StrCat("Freesapce trajectory validation failed: ",
                       traj_validation_result.DebugString()));
    }
  }

  UpdatePathUnsafeAndBlockedTime(
      *input.ego_pose, stalled_object_ids, nearest_stationary_object_id,
      stationary_object_stop_s, smooth_directional_path.path.length(),
      input.force_stop, is_stationary, input.time_interval, state);

  return FreespacePlannerOutput{
      .traj_points = std::move(trajectory),
      .gear_position = GenerateOutputGearPosition(
          state->task_type(), path_output,
          state->path_manager_state().drive_state(), *input.ego_pose,
          input.safe_stop, is_stationary),
      .low_speed_freespace =
          CheckLowSpeed(path_output.path.forward,
                        input.freespace_params->motion_constraint_params()),
      .enable_stationary_steering = enable_stationary_steering,
      .reset = path_output.is_new_path,
      .reset_reason = path_output.is_new_path
                          ? ResetReasonProto::NEW_FREESPACE_PATH
                          : ResetReasonProto::NONE,
      .smooth_directional_path = std::move(smooth_directional_path),
      .stop_s = stop_s,
      .is_path_blocked = IsPathBlocked(state->path_blocked_time())};
}
};  // namespace qcraft::planner
