#include "onboard/planner/planner_main_loop_internal.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include "Eigen/Core"
// IWYU pragma: no_include "onboard/lite/qissue_trans.h"

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/container/strong_int.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/utils/objects_view.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

bool MaybeReset(const PathPoint& pre_reset_planned_path_point,
                const Vec2d& current_pos, double longitudinal_reset_error,
                double lateral_reset_error, const std::string& planner_name,
                ResetReasonProto::Reason* reset_reason) {
  const Vec2d pos_diff = current_pos - Vec2d(pre_reset_planned_path_point.x(),
                                             pre_reset_planned_path_point.y());
  const Vec2d planned_tangent =
      Vec2d::FastUnitFromAngle(pre_reset_planned_path_point.theta());

  const double longitudinal_error = std::abs(pos_diff.Dot(planned_tangent));
  if (longitudinal_error > longitudinal_reset_error) {
    QEVENT("renjie", "planner_reset_lon_error", [&](QEvent* qevent) {
      qevent->AddField("lon_error", longitudinal_error)
          .AddField("planner_name", planner_name);
    });
    QLOG(ERROR) << "Resetting due to longitudinal error.";
    *reset_reason = ResetReasonProto::LON_ERROR_TOO_LARGE;
    return true;
  }

  const double lateral_error = std::abs(pos_diff.Dot(planned_tangent.Perp()));
  if (lateral_error > lateral_reset_error) {
    QEVENT("renjie", absl::StrCat("planner_reset_lat_error"),
           [&](QEvent* qevent) {
             qevent->AddField("lat_error", lateral_error)
                 .AddField("planner_name", planner_name);
           });
    QLOG(ERROR) << "Resetting due to lateral error.";
    *reset_reason = ResetReasonProto::LAT_ERROR_TOO_LARGE;
    return true;
  }

  return false;
}

std::optional<int> InterpolatePointFromPrevTrajectory(
    absl::Time time, const TrajectoryProto& prev_traj) {
  const double t = ToUnixDoubleSeconds(time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      t > prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      t < prev_traj_start_time) {
    return std::nullopt;
  }

  // Previous trajectory should have equal time interval of kTrajectoryTimeStep.
  return RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
}

bool InterpolatePointFromPrevTrajectoryIncludingPast(
    absl::Time time, const TrajectoryProto& prev_traj,
    ApolloTrajectoryPointProto* point) {
  if (prev_traj.trajectory_point().empty()) return false;
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  const double prev_traj_end_time =
      prev_traj_start_time +
      prev_traj.trajectory_point().rbegin()->relative_time();
  const double prev_traj_begin_time =
      prev_traj.past_points().empty()
          ? prev_traj_start_time +
                prev_traj.trajectory_point().begin()->relative_time()
          : prev_traj_start_time +
                prev_traj.past_points().begin()->relative_time();
  const double t = ToUnixDoubleSeconds(time);
  if (t > prev_traj_end_time || t < prev_traj_begin_time) {
    return false;
  }

  // Both previous trajectory and previous past trajectory should have equal
  // time interval of kTrajectoryTimeStep.
  const int relative_time_index =
      RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
  QCHECK_GE(relative_time_index, -prev_traj.past_points_size());
  QCHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  *point = relative_time_index < 0
               ? prev_traj.past_points(relative_time_index +
                                       prev_traj.past_points_size())
               : prev_traj.trajectory_point(relative_time_index);

  return true;
}

double CalLongitudinalErrorOnPrevTrajectory(const PoseProto& pose,
                                            const TrajectoryProto& prev_traj) {
  const double relative_time =
      pose.timestamp() - prev_traj.trajectory_start_timestamp();
  const auto time_matched_trajectory_pt =
      relative_time >= 0.0
          ? QueryApolloTrajectoryPointByT(prev_traj.trajectory_point().begin(),
                                          prev_traj.trajectory_point().end(),
                                          relative_time)
          : QueryApolloTrajectoryPointByT(prev_traj.past_points().begin(),
                                          prev_traj.past_points().end(),
                                          relative_time);
  const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  const Vec2d pos_diff =
      current_pos - Vec2d(time_matched_trajectory_pt.path_point().x(),
                          time_matched_trajectory_pt.path_point().y());
  const Vec2d planned_tangent =
      Vec2d::FastUnitFromAngle(time_matched_trajectory_pt.path_point().theta());
  return pos_diff.Dot(planned_tangent);
}

ResetReasonProto::Reason ACCResetReasonByAutonomyState(
    const AutonomyStateProto& prev_autonomy_state,
    const AutonomyStateProto& now_autonomy_state) {
  const auto prev_state = prev_autonomy_state.autonomy_state();
  const auto cur_state = now_autonomy_state.autonomy_state();
  const bool is_non_autonomy = !IS_AUTO_DRIVE(cur_state) &&
                               cur_state != AutonomyStateProto::AUTO_SPEED_ONLY;
  const bool is_first_engage =
      (prev_state == AutonomyStateProto::READY_TO_AUTO_DRIVE) &&
      (cur_state == AutonomyStateProto::AUTO_SPEED_ONLY);

  if (is_non_autonomy) {
    return ResetReasonProto::NON_AUTONOMY;
  } else if (is_first_engage) {
    return ResetReasonProto::FIRST_ENGAGE;
  } else if (cur_state == AutonomyStateProto::AUTO_SPEED_ONLY) {
    return ResetReasonProto::SPEED_ONLY;
  }
  // Default case is non-autonomy.
  return ResetReasonProto::NON_AUTONOMY;
}

bool MaybeResetByAutonomyState(const AutonomyStateProto& prev_autonomy_state,
                               const AutonomyStateProto& now_autonomy_state,
                               ResetReasonProto::Reason* reset_reason) {
  if (!IsAutoDrive(now_autonomy_state.autonomy_state()) &&
      now_autonomy_state.autonomy_state() !=
          AutonomyStateProto::AUTO_STEER_ONLY) {
    *reset_reason = ResetReasonProto::NON_AUTONOMY;
    return true;
  } else if (IS_ENGAGE(prev_autonomy_state.autonomy_state(),
                       now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::FIRST_ENGAGE;
    return true;
  } else if (IsAutoSteerOnlyToAutoDrive(prev_autonomy_state.autonomy_state(),
                                        now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::STEER_ONLY_ENGAGE;
    return true;
  } else if (IsAutoSpeedOnlyToAutoDrive(prev_autonomy_state.autonomy_state(),
                                        now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::SPEED_ONLY_ENGAGE;
    return true;
  } else if (now_autonomy_state.autonomy_state() ==
             AutonomyStateProto::AUTO_STEER_ONLY) {
    *reset_reason = ResetReasonProto::STEER_ONLY;
    return true;
  }

  return false;
}

bool MaybeResetFreespaceByAutonomyState(
    const AutonomyStateProto& prev_autonomy_state,
    const AutonomyStateProto& now_autonomy_state,
    ResetReasonProto::Reason* reset_reason) {
  if (!IsAutoDrive(now_autonomy_state.autonomy_state()) &&
      now_autonomy_state.autonomy_state() !=
          AutonomyStateProto::AUTO_STEER_ONLY) {
    *reset_reason = ResetReasonProto::NON_AUTONOMY;
    return true;
  } else if (IS_ENGAGE(prev_autonomy_state.autonomy_state(),
                       now_autonomy_state.autonomy_state())) {
    *reset_reason = ResetReasonProto::FIRST_ENGAGE;
    return true;
  }
  return false;
}

bool NeedForceResetEstPlanner(bool previously_triggered_aeb, bool full_stopped,
                              ResetReasonProto::Reason* reset_reason) {
  if (full_stopped) {
    *reset_reason = ResetReasonProto::FULL_STOP;
    return true;
  }

  if (previously_triggered_aeb) {
    VLOG(2) << "Planner resetting when previously triggered emergency stop";
    *reset_reason = ResetReasonProto::PREVIOUS_AEB;
    return true;
  }

  return false;
}

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByDrivePassage(
    const PlannerSemanticMapManager& psmm, const RouteSections& sections,
    const Vec2d& query_point) {
  ASSIGN_OR_RETURN(const auto nearest_lane_path,
                   FindClosestLanePathOnRouteSectionsToSmoothPoint(
                       psmm, sections, query_point));

  const double step_s = std::min(1.0, 0.5 * nearest_lane_path.length());
  ASSIGN_OR_RETURN(const auto drive_passage,
                   BuildDrivePassageFromLanePath(
                       psmm, nearest_lane_path, step_s,
                       /*avoid_loop=*/false, /*backward_extend_len=*/0.0,
                       /*required_planning_horizon=*/0.0,
                       /*required_backward_len=*/0.0,
                       /*override_speed_limit_mps=*/std::nullopt,
                       FrenetFrameType::kBruteFroce),
                   _ << "FindSmoothPointOnRouteSectionsByDrivePassage: "
                        "BuildDrivePassageFromLanePath on "
                     << nearest_lane_path.DebugString() << " failed.");

  ASSIGN_OR_RETURN(
      const auto sl, drive_passage.QueryFrenetCoordinateAt(query_point),
      _ << "FindSmoothPointOnRouteSectionsByDrivePassage: Fail to project ego "
           "pos ("
        << query_point.x() << ", " << query_point.y()
        << ") on drive passage from lane path "
        << nearest_lane_path.DebugString());

  const auto start_lane_point = nearest_lane_path.AfterArclength(sl.s).front();
  for (int i = 0; i < sections.size(); ++i) {
    SMM_ASSIGN_SECTION_OR_CONTINUE_ISSUE(section_info, psmm,
                                         sections.section_ids()[i]);

    if (std::find(section_info.lane_ids.begin(), section_info.lane_ids.end(),
                  start_lane_point.lane_id()) != section_info.lane_ids.end()) {
      return PointOnRouteSections{.accum_s = sl.s,
                                  .section_idx = i,
                                  .fraction = start_lane_point.fraction(),
                                  .lane_id = start_lane_point.lane_id()};
    }
  }

  return absl::NotFoundError(
      absl::StrCat("FindSmoothPointOnRouteSectionsByDrivePassage: Point (",
                   query_point.x(), ", ", query_point.y(),
                   ") is not on route sections:", sections.DebugString()));
}

PathPoint ResamplePathPoint(const DiscretizedPath& current_smooth_path,
                            double sample_s) {
  if (sample_s > current_smooth_path.length()) {
    return GetPathPointAlongCircle(current_smooth_path.back(),
                                   sample_s - current_smooth_path.length());
  } else if (sample_s < 0.0) {
    return GetPathPointAlongCircle(current_smooth_path.front(), sample_s);
  } else {
    return current_smooth_path.Evaluate(sample_s);
  }
}

bool MaybeResetEstPlanner(
    const ApolloTrajectoryPointProto& pre_reset_planned_point,
    const Vec2d& current_pos, ResetReasonProto::Reason* reset_reason) {
  constexpr double kLongitudinalErrorForReset = 3.0;  // m.
  return MaybeReset(pre_reset_planned_point.path_point(), current_pos,
                    kLongitudinalErrorForReset,
                    FLAGS_planner_lateral_reset_error, "est_planner",
                    reset_reason);
}

bool MaybeResetFreespacePlanner(
    const ApolloTrajectoryPointProto& pre_reset_planned_point,
    const Vec2d& current_pos, ResetReasonProto::Reason* reset_reason) {
  // TODO(yunfeng): May need to tune this, but keep it the same with EstPlanner
  // currently.
  constexpr double kLongitudinalErrorForReset = 3.0;  // m.
  constexpr double kLateralErrorForReset = 0.35;      // m.
  return MaybeReset(pre_reset_planned_point.path_point(), current_pos,
                    kLongitudinalErrorForReset, kLateralErrorForReset,
                    "freespace_planner", reset_reason);
}

bool MaybeResetPathStopFreespacePlanner(
    const PathPoint& pre_reset_planned_path_point, const Vec2d& current_pos,
    ResetReasonProto::Reason* reset_reason) {
  // TODO(yunfeng): May need to tune this, but keep it the same with EstPlanner
  // currently.
  constexpr double kLongitudinalErrorForReset = 3.0;  // m.
  constexpr double kLateralErrorForReset = 0.35;      // m.
  return MaybeReset(pre_reset_planned_path_point, current_pos,
                    kLongitudinalErrorForReset, kLateralErrorForReset,
                    "freespace_planner", reset_reason);
}

PlanStartPointInfo ComputeAutoDriveFreespacePlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  bool reset = false;
  ResetReasonProto::Reason reset_reason = ResetReasonProto::NONE;
  absl::Time plan_start_time = predicted_plan_time;
  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point =
      std::nullopt;
  double path_s_increment_from_previous_frame = 0.0;
  std::optional<int> start_index_on_prev_traj = std::nullopt;

  // Reset logic.
  start_index_on_prev_traj =
      InterpolatePointFromPrevTrajectory(predicted_plan_time, prev_trajectory);
  if (!start_index_on_prev_traj.has_value()) {
    reset = true;
    reset_reason = ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND;
  } else {
    prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
        prev_trajectory.trajectory_point(*start_index_on_prev_traj));
    plan_start_time =
        FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                              prev_trajectory.trajectory_start_timestamp());
    prev_planned_traj_point->set_relative_time(0.0);
    path_s_increment_from_previous_frame =
        prev_planned_traj_point->path_point().s();
    // Trajectory s is decreasing for reverse driving, so we should flip the
    // sign to get path s increment from previous directional path.
    if (prev_trajectory.has_directional_path() &&
        !prev_trajectory.directional_path().forward()) {
      path_s_increment_from_previous_frame =
          -path_s_increment_from_previous_frame;
    }
    prev_planned_traj_point->mutable_path_point()->set_s(0.0);
    // Check if we need reset.
    // Trajectory point from previous trajectory at Clock::Now().
    ApolloTrajectoryPointProto prev_planned_now_point;
    if (!InterpolatePointFromPrevTrajectoryIncludingPast(
            FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
            &prev_planned_now_point)) {
      // Reset if prev_planned_now_point not found.
      reset = true;
      reset_reason = ResetReasonProto::PREV_NOW_POINT_NOT_FOUND;
    } else {
      const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
      // Reset if control error too large.
      reset = MaybeResetFreespacePlanner(prev_planned_now_point, current_pos,
                                         &reset_reason);
    }
  }
  // Reset if fully stopped.
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  const bool full_stop =
      prev_planned_traj_point.has_value() &&
      prev_planned_traj_point->v() == 0.0 &&
      std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;
  if (!reset && full_stop) {
    reset = true;
    reset_reason = ResetReasonProto::FULL_STOP;
  }

  if (!reset) {
    QCHECK(prev_planned_traj_point.has_value());
  } else {
    start_index_on_prev_traj = std::nullopt;
  }

  // Compute plan start point based on reset result.
  ApolloTrajectoryPointProto start_point;
  if (reset && reset_reason == ResetReasonProto::LAT_ERROR_TOO_LARGE) {
    start_point = ComputePlanStartPointAfterLateralReset(
        prev_planned_traj_point, pose, chassis, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset) {
    start_point = ComputePlanStartPointAfterReset(
        prev_planned_traj_point, pose, chassis, motion_constraint_params,
        vehicle_geom_params, vehicle_drive_params,
        /*is_forward_task=*/std::nullopt);
  } else {
    start_point = *prev_planned_traj_point;
  }

  return PlanStartPointInfo{
      .reset = reset,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          reset ? 0.0 : path_s_increment_from_previous_frame,
      .plan_time =
          reset ? FromUnixDoubleSeconds(pose.timestamp()) : plan_start_time,
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

PlanStartPointInfo ComputeAutoDrivePathStopFreespacePlanStartPoint(
    const TrajectoryProto& prev_trajectory, const PoseProto& pose,
    const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  bool reset = false;
  ResetReasonProto::Reason reset_reason = ResetReasonProto::NONE;
  std::optional<PathPoint> prev_planned_path_point = std::nullopt;
  double path_s_increment_from_previous_frame = 0.0;

  // Reset logic.
  if (prev_trajectory.directional_path().path().empty()) {
    reset = true;
    reset_reason = ResetReasonProto::PREV_PATH_EMPTY;
  } else {
    // Find prev planned path point on previous directional path including past.
    std::vector<PathPoint> path_points_including_past;
    path_points_including_past.reserve(
        prev_trajectory.past_directional_path_points().size() +
        prev_trajectory.directional_path().path().size());
    for (const auto& p : prev_trajectory.past_directional_path_points()) {
      path_points_including_past.push_back(p);
    }
    for (const auto& p : prev_trajectory.directional_path().path()) {
      path_points_including_past.push_back(p);
    }
    const double front_s = path_points_including_past.front().s();
    for (auto& p : path_points_including_past) {
      p.set_s(p.s() - front_s);
    }
    if (path_points_including_past.size() == 1) {
      path_points_including_past.push_back(path_points_including_past.front());
    }
    DiscretizedPath path =
        DiscretizedPath(std::move(path_points_including_past));
    const Vec2d current_pos =
        Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
    const auto pose_sl = path.XYToSL(current_pos);
    path_s_increment_from_previous_frame = pose_sl.s + front_s;
    prev_planned_path_point = path.Evaluate(pose_sl.s);
    // The plan start point heading and curvature corresponds to AV state.
    if (!prev_trajectory.directional_path().forward()) {
      prev_planned_path_point->set_theta(
          NormalizeAngle(prev_planned_path_point->theta() + M_PI));
      prev_planned_path_point->set_kappa(-prev_planned_path_point->kappa());
    }
    reset = MaybeResetPathStopFreespacePlanner(*prev_planned_path_point,
                                               current_pos, &reset_reason);
    if (reset) {
      prev_planned_path_point = std::nullopt;
    }
  }

  // Reset if fully stopped.
  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  const bool full_stop =
      std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;
  if (!reset && full_stop) {
    reset = true;
    reset_reason = ResetReasonProto::FULL_STOP;
    prev_planned_path_point = std::nullopt;
  }

  // Compute plan start point based on reset result.
  ApolloTrajectoryPointProto start_point;
  if (reset) {
    start_point = ComputePlanStartPointAfterReset(
        /*prev_reset_planned_point=*/std::nullopt, pose, chassis,
        motion_constraint_params, vehicle_geom_params, vehicle_drive_params,
        /*is_forward_task=*/std::nullopt);
  } else {
    QCHECK(prev_planned_path_point.has_value());
    start_point = ComputePlanStartPointFromPrevPlannedPathPoint(
        *prev_planned_path_point, pose);
  }

  return PlanStartPointInfo{
      .reset = reset,
      .start_index_on_prev_traj = std::nullopt,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          reset ? 0.0 : path_s_increment_from_previous_frame,
      .plan_time = FromUnixDoubleSeconds(pose.timestamp()),
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

}  // namespace

std::shared_ptr<const ObjectsProto> GetAllObjects(
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects) {
  SCOPED_QTRACE("GetAllObjects");

  ObjectsView objects_view;
  if (real_objects != nullptr) {
    objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, real_objects);
  }
  if (virtual_objects != nullptr) {
    objects_view.UpdateObjects(ObjectsProto::SCOPE_VIRTUAL, virtual_objects);
  }
  return objects_view.ExportAllObjectsProto();
}

void FillTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const mapping::LanePath& target_lane_path_from_current,
    const LaneChangeStateProto& lane_change_state, TurnSignal turn_signal,
    const DoorDecision& door_decision, bool is_aeb_triggered,
    const DrivingStateProto& driving_state,
    const TrajectoryValidationResultProto& validate_result,
    TrajectoryProto* trajectory) {
  SCOPED_QTRACE("FillTrajectoryProto");

  trajectory->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory->add_trajectory_point() = planned_trajectory[i];
  }

  // NOTE: past_points are designed for controller.
  for (const auto& past_point : past_points) {
    *trajectory->add_past_points() = past_point;
  }

  target_lane_path_from_current.ToProto(
      trajectory->mutable_target_lane_path_from_current());

  trajectory->set_turn_signal(turn_signal);

  // TODO(renjie): redesign it after onboard freespace planner.
  trajectory->set_gear(Chassis::GEAR_DRIVE);

  *(trajectory->mutable_door_decision()) = door_decision;

  trajectory->set_aeb_triggered(is_aeb_triggered);

  trajectory->mutable_driving_state()->CopyFrom(driving_state);

  trajectory->set_lane_change_stage(lane_change_state.stage());
  if (lane_change_state.stage() != LCS_NONE) {
    trajectory->set_lane_change_left(lane_change_state.lc_left());
  }
  trajectory->set_entered_target_lane(lane_change_state.entered_target_lane());
  trajectory->set_low_speed_freespace(false);
  trajectory->set_enable_stationary_steering(false);
  // TODO(guoqiang): Validate trajectories in planner_module.
  trajectory->mutable_traj_validation_result()->CopyFrom(validate_result);
}

void ConvertPreviousTrajectoryToCurrentSmoothLateral(
    absl::Time predicted_plan_time, std::vector<PathPoint> previous_path,
    TrajectoryProto* previous_trajectory) {
  std::optional<int> start_index_on_prev_traj = std::nullopt;
  start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
      predicted_plan_time, *previous_trajectory);
  if (!start_index_on_prev_traj.has_value() || previous_path.empty()) {
    return;
  }

  DiscretizedPath current_smooth_path(std::move(previous_path));

  // previous trajectory resample.
  std::vector<double> past_points_s;
  std::vector<double> points_s;

  if (!previous_trajectory->past_points().empty()) {
    for (int i = 0; i < previous_trajectory->past_points_size(); ++i) {
      const auto& current_path_point =
          previous_trajectory->past_points(i).path_point();
      past_points_s.push_back(current_path_point.s());
    }
  }

  if (!previous_trajectory->trajectory_point().empty()) {
    for (int i = 0; i < previous_trajectory->trajectory_point_size(); ++i) {
      const auto& current_path_point =
          previous_trajectory->trajectory_point(i).path_point();
      points_s.push_back(current_path_point.s());
    }
  }

  const auto& prev_planned_traj_point =
      previous_trajectory->trajectory_point(*start_index_on_prev_traj);
  const FrenetCoordinate prev_pt_sl = current_smooth_path.XYToSL(
      Vec2d(prev_planned_traj_point.path_point().x(),
            prev_planned_traj_point.path_point().y()));
  const double prev_planned_s_current_smooth = prev_pt_sl.s;
  const double delta_s =
      points_s[*start_index_on_prev_traj] - prev_planned_s_current_smooth;

  if (!previous_trajectory->trajectory_point().empty()) {
    for (int i = 0; i < previous_trajectory->trajectory_point_size(); ++i) {
      auto* path_point = previous_trajectory->mutable_trajectory_point(i)
                             ->mutable_path_point();
      const double sample_s = points_s[i] - delta_s;
      *path_point = ResamplePathPoint(current_smooth_path, sample_s);
      path_point->set_s(points_s[i]);
    }
  }
  if (!previous_trajectory->past_points().empty()) {
    for (int i = 0; i < previous_trajectory->past_points_size(); ++i) {
      auto* path_point =
          previous_trajectory->mutable_past_points(i)->mutable_path_point();
      const double sample_s = past_points_s[i] - delta_s;
      *path_point = ResamplePathPoint(current_smooth_path, sample_s);
      path_point->set_s(past_points_s[i]);
    }
  }
}

void ConvertSmoothPathToGlobalCoordinates(
    const CoordinateConverter& coordinate_converter,
    std::vector<PathPoint>* path) {
  for (auto& path_point : *path) {
    const Vec2d global_point = coordinate_converter.SmoothToGlobal(
        Vec2d(path_point.x(), path_point.y()));
    const double global_yaw =
        coordinate_converter.SmoothYawToGlobalNoNormalize(path_point.theta());
    path_point.set_x(global_point.x());
    path_point.set_y(global_point.y());
    path_point.set_theta(global_yaw);
  }
}

void ConvertPreviousPathToCurrentSmooth(
    const CoordinateConverter& coordinate_converter,
    const std::vector<PathPoint>& prev_path_global,
    std::vector<PathPoint>* prev_path) {
  *prev_path = prev_path_global;
  if (!prev_path->empty()) {
    for (int i = 0; i < prev_path->size(); ++i) {
      auto& path_point = (*prev_path)[i];
      auto& path_point_global = prev_path_global[i];
      const Vec2d smooth_point = coordinate_converter.GlobalToSmooth(
          Vec2d(path_point_global.x(), path_point_global.y()));
      const double smooth_yaw =
          coordinate_converter.GlobalYawToSmoothNoNormalize(
              path_point_global.theta());
      path_point.set_x(smooth_point.x());
      path_point.set_y(smooth_point.y());
      path_point.set_theta(smooth_yaw);
    }
  }
}

void ReportCandidateTrafficLightInfo(
    const TrafficLightInfoMap& traffic_light_map, PlannerDebugProto* debug) {
  QCHECK_NOTNULL(debug);
  for (auto iter = traffic_light_map.begin(); iter != traffic_light_map.end();
       ++iter) {
    iter->second.ToProto(debug->add_candidate_traffic_light_info());
  }
}

void ReportSelectedTrafficLightInfo(
    const TrafficLightInfoMap& traffic_light_map,
    const mapping::LanePath& lane_path, TrafficLightInfoProto* proto) {
  for (const auto id : lane_path.lane_ids()) {
    const auto iter = traffic_light_map.find(id);
    if (iter != traffic_light_map.end()) {
      iter->second.ToProto(proto);
      return;
    }
  }
}

std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto& previous_trajectory,
    const MotionConstraintParamsProto& motion_constraint_params, bool reset) {
  SCOPED_QTRACE("CreatePrevTrajectory");

  if (previous_trajectory.trajectory_point().empty() || reset) return {};

  const double now_in_sec = ToUnixDoubleSeconds(plan_time);
  const double time_advancement = std::max(
      0.0, now_in_sec - previous_trajectory.trajectory_start_timestamp());
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory_points(
      previous_trajectory.trajectory_point().begin(),
      previous_trajectory.trajectory_point().end());
  return ShiftTrajectoryByTime(time_advancement, previous_trajectory_points,
                               motion_constraint_params.max_decel_jerk(),
                               motion_constraint_params.max_accel_jerk());
}

PlanStartPointInfo ComputeACCPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,
    bool previously_triggered_aeb, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  SCOPED_QTRACE("ComputeACCPlanStartPoint");

  absl::Time plan_start_time = predicted_plan_time;
  // The below variables may have meaningful values only in auto drive.
  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point =
      std::nullopt;
  double path_s_increment_from_previous_frame = 0.0;
  bool full_stop = false;
  // Only has value if not reset.
  std::optional<int> start_index_on_prev_traj = std::nullopt;
  // Autonomy state based reset logic.
  ResetReasonProto::Reason reset_reason =
      ACCResetReasonByAutonomyState(prev_autonomy_state, now_autonomy_state);
  // We always have one reset reason.
  QCHECK(reset_reason != ResetReasonProto::NONE);
  if (reset_reason == ResetReasonProto::SPEED_ONLY) {
    // Reset logic in AUTO_SPEED_ONLY drive.
    start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
        predicted_plan_time, prev_trajectory);
    if (!start_index_on_prev_traj.has_value()) {
      reset_reason = ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND;
    } else {
      prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
          prev_trajectory.trajectory_point(*start_index_on_prev_traj));
      plan_start_time =
          FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                                prev_trajectory.trajectory_start_timestamp());
      prev_planned_traj_point->set_relative_time(0.0);
      path_s_increment_from_previous_frame =
          prev_planned_traj_point->path_point().s();
      prev_planned_traj_point->mutable_path_point()->set_s(0.0);
      // Check if we need reset.
      // Trajectory point from previous trajectory at Clock::Now().
      ApolloTrajectoryPointProto prev_planned_now_point;
      if (!InterpolatePointFromPrevTrajectoryIncludingPast(
              FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
              &prev_planned_now_point)) {
        // Reset if prev_planned_now_point not found.
        reset_reason = ResetReasonProto::PREV_NOW_POINT_NOT_FOUND;
      } else {
        const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
        // Reset if control error too large.
        ResetReasonProto::Reason control_reset_reason = ResetReasonProto::NONE;
        constexpr double kAccLongitudinalErrorForReset = 5.0;  // m.
        const auto control_reset = MaybeReset(
            prev_planned_now_point.path_point(), current_pos,
            kAccLongitudinalErrorForReset, FLAGS_planner_lateral_reset_error,
            "acc_planner", &control_reset_reason);
        if (control_reset &&
            control_reset_reason == ResetReasonProto::LON_ERROR_TOO_LARGE) {
          reset_reason = control_reset_reason;
        }
      }
    }
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    full_stop = prev_planned_traj_point.has_value() &&
                prev_planned_traj_point->v() == 0.0 &&
                std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;
    // Check if we need to force reset if current reset reason is SPEED_ONLY.
    if (reset_reason == ResetReasonProto::SPEED_ONLY) {
      ResetReasonProto::Reason force_reset_reason = ResetReasonProto::NONE;
      const bool need_force_reset = NeedForceResetEstPlanner(
          previously_triggered_aeb, full_stop, &force_reset_reason);
      if (need_force_reset) {
        reset_reason = force_reset_reason;
      }
    }
  }

  // We only need to perform lateral reset or full reset.
  const bool only_lat_reset = (reset_reason == ResetReasonProto::SPEED_ONLY);
  if (only_lat_reset) {
    QCHECK(prev_planned_traj_point.has_value());
  } else {
    start_index_on_prev_traj = std::nullopt;
  }
  ApolloTrajectoryPointProto start_point;
  if (only_lat_reset) {
    const double prev_longitudinal_error =
        CalLongitudinalErrorOnPrevTrajectory(pose, prev_trajectory);
    VLOG(2) << "prev_longitudinal_error: " << prev_longitudinal_error;
    start_point = ComputeACCPlanStartPointAfterLateralReset(
        prev_planned_traj_point, pose, chassis, vehicle_geom_params,
        vehicle_drive_params,
        ToUnixDoubleSeconds(plan_start_time) - pose.timestamp(),
        prev_longitudinal_error);
  } else {
    start_point = ComputePlanStartPointAfterReset(
        prev_planned_traj_point, pose, chassis, motion_constraint_params,
        vehicle_geom_params, vehicle_drive_params, /*is_forward_task=*/true);
  }
  VLOG(2) << "ACC reset reason: " << ResetReasonProto_Reason_Name(reset_reason)
          << '\n'
          << " pre state: "
          << AutonomyStateProto_State_Name(prev_autonomy_state.autonomy_state())
          << '\n'
          << " now state: "
          << AutonomyStateProto_State_Name(now_autonomy_state.autonomy_state());
  return PlanStartPointInfo{
      .reset = true,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          only_lat_reset ? path_s_increment_from_previous_frame : 0.0,
      .plan_time = only_lat_reset ? plan_start_time
                                  : FromUnixDoubleSeconds(pose.timestamp()),
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

PlanStartPointInfo ComputeEstPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,
    bool previously_triggered_aeb, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  SCOPED_QTRACE("ComputeEstPlanStartPoint");

  bool reset = false;
  ResetReasonProto::Reason reset_reason = ResetReasonProto::NONE;
  absl::Time plan_start_time = predicted_plan_time;
  // The below variables may have meaningful values only in auto drive.
  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point =
      std::nullopt;
  double path_s_increment_from_previous_frame = 0.0;
  bool full_stop = false;
  // Only has value if not reset.
  std::optional<int> start_index_on_prev_traj = std::nullopt;
  // Autonomy state based reset logic.
  reset = MaybeResetByAutonomyState(prev_autonomy_state, now_autonomy_state,
                                    &reset_reason);
  if (!reset) {
    // Reset logic in auto drive.
    start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
        predicted_plan_time, prev_trajectory);
    if (!start_index_on_prev_traj.has_value()) {
      reset = true;
      reset_reason = ResetReasonProto::PREV_PLAN_POINT_NOT_FOUND;
    } else {
      prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
          prev_trajectory.trajectory_point(*start_index_on_prev_traj));
      plan_start_time =
          FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                                prev_trajectory.trajectory_start_timestamp());
      prev_planned_traj_point->set_relative_time(0.0);
      path_s_increment_from_previous_frame =
          prev_planned_traj_point->path_point().s();
      prev_planned_traj_point->mutable_path_point()->set_s(0.0);
      // Check if we need reset.
      // Trajectory point from previous trajectory at Clock::Now().
      ApolloTrajectoryPointProto prev_planned_now_point;
      if (!InterpolatePointFromPrevTrajectoryIncludingPast(
              FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
              &prev_planned_now_point)) {
        // Reset if prev_planned_now_point not found.
        reset = true;
        reset_reason = ResetReasonProto::PREV_NOW_POINT_NOT_FOUND;
      } else {
        const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
        // Reset if control error too large.
        reset = MaybeResetEstPlanner(prev_planned_now_point, current_pos,
                                     &reset_reason);
      }
    }
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    full_stop = prev_planned_traj_point.has_value() &&
                prev_planned_traj_point->v() == 0.0 &&
                std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;

    if (!reset) {
      reset = NeedForceResetEstPlanner(previously_triggered_aeb, full_stop,
                                       &reset_reason);
    }
  }

  if (!reset) {
    QCHECK(prev_planned_traj_point.has_value());
  } else {
    start_index_on_prev_traj = std::nullopt;
  }
  ApolloTrajectoryPointProto start_point;
  if (reset && reset_reason == ResetReasonProto::STEER_ONLY) {
    start_point = ComputePlanStartPointAfterLongitudinalResetFromPrevTrajectory(
        prev_trajectory, pose, chassis, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset && reset_reason == ResetReasonProto::LAT_ERROR_TOO_LARGE) {
    start_point = ComputePlanStartPointAfterLateralReset(
        prev_planned_traj_point, pose, chassis, vehicle_geom_params,
        vehicle_drive_params);
  } else if (reset) {
    start_point = ComputePlanStartPointAfterReset(
        prev_planned_traj_point, pose, chassis, motion_constraint_params,
        vehicle_geom_params, vehicle_drive_params, /*is_forward_task=*/true);
  } else {
    start_point = *prev_planned_traj_point;
  }

  return PlanStartPointInfo{
      .reset = reset,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = start_point,
      .path_s_increment_from_previous_frame =
          reset ? 0.0 : path_s_increment_from_previous_frame,
      .plan_time =
          reset ? FromUnixDoubleSeconds(pose.timestamp()) : plan_start_time,
      .full_stop = full_stop,
      .reset_reason = reset_reason};
}

PlanStartPointInfo ComputeFreespacePlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params) {
  SCOPED_QTRACE("ComputeFreespacePlanStartPoint");

  bool reset = false;
  ResetReasonProto::Reason reset_reason = ResetReasonProto::NONE;
  // Autonomy state based reset logic.
  reset = MaybeResetFreespaceByAutonomyState(prev_autonomy_state,
                                             now_autonomy_state, &reset_reason);
  if (reset) {
    return PlanStartPointInfo{
        .reset = true,
        .start_index_on_prev_traj = std::nullopt,
        .start_point = ComputePlanStartPointAfterReset(
            /*prev_reset_planned_point=*/std::nullopt, pose, chassis,
            motion_constraint_params, vehicle_geom_params, vehicle_drive_params,
            /*is_forward_task=*/std::nullopt),
        .path_s_increment_from_previous_frame = 0.0,
        .plan_time = FromUnixDoubleSeconds(pose.timestamp()),
        .full_stop = false,
        .reset_reason = reset_reason};
  }

  // Compute plan start point in auto drive.
  if (!FLAGS_planner_freespace_path_stop_mode) {
    return ComputeAutoDriveFreespacePlanStartPoint(
        predicted_plan_time, prev_trajectory, pose, chassis,
        motion_constraint_params, vehicle_geom_params, vehicle_drive_params);
  } else {
    return ComputeAutoDrivePathStopFreespacePlanStartPoint(
        prev_trajectory, pose, chassis, motion_constraint_params,
        vehicle_geom_params, vehicle_drive_params);
  }
}

absl::Duration GetStPathPlanLookAheadTime(
    const PlanStartPointInfo& plan_start_point_info, const PoseProto& pose,
    absl::Duration planned_look_ahead_time,
    const TrajectoryProto& previous_trajectory) {
  absl::Duration look_ahead_time = planned_look_ahead_time;
  // Since we have spacetime planning but lateral and longitudinal control,
  // first reference point for laterl control may have large time diff with
  // plan start point. Notice that after getting spacetime trajectory, speed
  // planning using spacetime result as path will be running. If we start plan
  // path from point close to control ref point, there will be a performance
  // improvement on lateral control. So We check the closest point of pose on
  // previous trajectory and check the diff between it and plan start point,
  // if too large, set the path plan start point to this point.
  // https://docs.google.com/presentation/d/1oXnBZ95lRy_X1U9U_dPWJ_a8MPZSGTSRJuPeqzrrhiM/edit#slide=id.p1
  if (FLAGS_enable_path_start_point_look_ahead &&
      plan_start_point_info.start_index_on_prev_traj.has_value() &&
      !previous_trajectory.trajectory_point().empty()) {
    const auto& previous_trajectory_points =
        previous_trajectory.trajectory_point();
    const Vec2d plan_start_pos =
        Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
    const auto closest_iter = absl::c_min_element(
        previous_trajectory_points,
        [&plan_start_pos](const ApolloTrajectoryPointProto& p1,
                          const ApolloTrajectoryPointProto& p2) {
          const Vec2d pos1(p1.path_point().x(), p1.path_point().y());
          const Vec2d pos2(p2.path_point().x(), p2.path_point().y());
          return (pos1 - plan_start_pos).squaredNorm() <
                 (pos2 - plan_start_pos).squaredNorm();
        });
    const int closest_index_on_prev_traj =
        std::distance(previous_trajectory_points.begin(), closest_iter);
    const auto diff_time =
        static_cast<double>((closest_index_on_prev_traj -
                             *plan_start_point_info.start_index_on_prev_traj) *
                            kTrajectoryTimeStep);
    if (diff_time > FLAGS_planner_path_start_point_time_diff_limit) {
      constexpr double kLookAheadTimeMaxTime = 5.0;  // s
      look_ahead_time +=
          absl::Seconds(std::min(kLookAheadTimeMaxTime, diff_time));
    }
  }
  return look_ahead_time;
}

StPathPlanStartPointInfo GetStPathPlanStartPointInfo(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo& plan_start_point_info,
    const TrajectoryProto& previous_trajectory,
    std::optional<double> trajectory_optimizer_time_step,
    std::optional<absl::Time> last_st_path_plan_start_time) {
  absl::Time path_planning_time =
      plan_start_point_info.plan_time + look_ahead_time;

  // Increase path_planning_time,
  // Making the time duration between last_st_path_plan_start_time and
  // path_planning_time to be integer times of trajectory_optimizer_time_step.
  if (last_st_path_plan_start_time.has_value() &&
      FLAGS_st_path_planner_lookahead_for_trajectory_optimizer_synchronization) {  // NOLINT
    QCHECK(trajectory_optimizer_time_step.has_value());
    QCHECK_GT(*trajectory_optimizer_time_step, 0.0);
    constexpr double kTimeEpsilon = 0.001;  // In seconds.
    const double delta_t =
        std::max(absl::ToDoubleSeconds(path_planning_time -
                                       *last_st_path_plan_start_time),
                 0.0);
    const double res = std::fmod(delta_t, *trajectory_optimizer_time_step);
    if (res > kTimeEpsilon &&
        res < (*trajectory_optimizer_time_step) - kTimeEpsilon) {
      path_planning_time +=
          absl::Seconds((*trajectory_optimizer_time_step) - res);
    }
  }

  const auto path_start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
      path_planning_time, previous_trajectory);
  if (plan_start_point_info.reset ||
      !path_start_index_on_prev_traj.has_value()) {
    VLOG(3) << "Do not change path plan start time because planner reset or "
               "path planning time not found in previous trajectory.";
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point = 0,
            .start_point = plan_start_point_info.start_point,
            .plan_time = plan_start_point_info.plan_time};
  } else {
    VLOG(3) << "path_planning_time:" << path_planning_time;
    ApolloTrajectoryPointProto path_plan_start_point =
        previous_trajectory.trajectory_point(*path_start_index_on_prev_traj);
    path_plan_start_point.set_relative_time(0.0);
    path_plan_start_point.mutable_path_point()->set_s(0.0);
    // If planner not reset, plan_start_point_info.start_index_on_prev_traj
    // should have value.
    QCHECK(plan_start_point_info.start_index_on_prev_traj.has_value());
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point =
                *path_start_index_on_prev_traj -
                *plan_start_point_info.start_index_on_prev_traj,
            .start_point = path_plan_start_point,
            .plan_time = path_planning_time};
  }
}

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const RouteNaviInfo& route_navi_info, mapping::ElementId ego_proj_lane_id,
    DriverAction::LaneChangeCommand lc_cmd) {
  if (lc_cmd == DriverAction::LC_CMD_NONE) {
    return absl::NotFoundError("Empty lane change command!");
  }

  SMM_ASSIGN_SECTION_OR_ERROR_ISSUE(section_info, psmm,
                                    route_sections_from_start.front().id);
  const auto& lane_ids = section_info.lane_ids;
  const auto current_it =
      std::find(lane_ids.begin(), lane_ids.end(), ego_proj_lane_id);
  if (current_it == lane_ids.end()) {
    return absl::NotFoundError(
        "Current lane not found in the current route section!");
  }

  constexpr double kForwardInitLength = 60.0;  // m.
  const auto short_route_sections = *ClampRouteSectionsBeforeArcLength(
      psmm, route_sections_from_start, kForwardInitLength);
  const RouteSectionsInfo short_sections_info(psmm, &short_route_sections);
  const double start_frac = short_sections_info.start_fraction();

  if (lc_cmd == DriverAction::LC_CMD_STRAIGHT) {
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *current_it, start_frac,
        short_sections_info.length());
  }

  if (lc_cmd == DriverAction::LC_CMD_LEFT) {
    if (current_it == lane_ids.begin()) {
      return absl::NotFoundError("Already on the leftmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *std::prev(current_it),
        start_frac, short_sections_info.length());
  } else {
    if (std::next(current_it) == lane_ids.end()) {
      return absl::NotFoundError("Already on the rightmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, route_navi_info, *std::next(current_it),
        start_frac, short_sections_info.length());
  }
}

void UpdatePreferredLanePath(const PlannerSemanticMapManager& psmm,
                             const RouteSections& route_sections_from_start,
                             const RouteNaviInfo& route_navi_info,
                             mapping::LanePath* preferred_lane_path,
                             QALCState* alc_state,
                             DriverAction::LaneChangeCommand* lc_cmd_state) {
  if (preferred_lane_path->IsEmpty()) return;

  // Extend preferred lane path to keep trying lane change.
  constexpr double kPreferredLaneLength = 50.0;  // m.
  const RouteSectionsInfo short_sections_info(psmm, &route_sections_from_start);
  absl::flat_hash_set<mapping::ElementId> preferred_lanes(
      preferred_lane_path->lane_ids().begin(),
      preferred_lane_path->lane_ids().end());
  for (const auto& lane_id : short_sections_info.front().lane_ids) {
    if (preferred_lanes.contains(lane_id)) {
      *preferred_lane_path = FindLanePathFromLaneAlongRouteSections(
          psmm, short_sections_info, route_navi_info, lane_id,
          short_sections_info.start_fraction(), kPreferredLaneLength);
      break;
    }
  }
  if (preferred_lane_path->length() < 0.5 * kMinLcLaneLength) {
    // Clear preferred lane path if its end is nearly reached.
    preferred_lane_path->Clear();
    *alc_state =
        *alc_state == ALC_RETURNING ? ALC_RETURN_COMPLETED : ALC_COMPLETED;
    *lc_cmd_state = DriverAction::LC_CMD_NONE;
    QLOG(INFO) << "Teleop lane change state released!";
  }
}

void HandleNewTeleopCommand(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections_from_start,
                            const RouteNaviInfo& route_navi_info,
                            mapping::ElementId ego_proj_lane_id,
                            DriverAction::LaneChangeCommand new_lc_cmd,
                            const mapping::LanePath& prev_target_lane_path,
                            const mapping::LanePath& prev_lp_before_lc,
                            const LaneChangeStateProto& prev_lc_state,
                            TurnSignal selector_prep_turn_signal,
                            mapping::LanePath* preferred_lane_path,
                            QALCState* alc_state,
                            DriverAction::LaneChangeCommand* lc_cmd_state) {
  if (new_lc_cmd == DriverAction::LC_CMD_NONE) return;

  if (new_lc_cmd == DriverAction::LC_CMD_CANCEL) {
    if (prev_lc_state.stage() == LCS_NONE) {
      if (selector_prep_turn_signal != TurnSignal::TURN_SIGNAL_NONE) {
        *preferred_lane_path = prev_target_lane_path;
        *alc_state = ALC_RETURNING;
        *lc_cmd_state =
            selector_prep_turn_signal == TurnSignal::TURN_SIGNAL_LEFT
                ? DriverAction::LC_CMD_RIGHT
                : DriverAction::LC_CMD_LEFT;
      } else {
        QLOG(INFO) << "Cleared teleop lane change state!";
        if (*alc_state == ALC_PREPARE) {
          QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                     QRunEvent::PLC_CANCEL_BEFORE_START);
        }
        preferred_lane_path->Clear();
        *alc_state = ALC_STANDBY_ENABLE;
        *lc_cmd_state = DriverAction::LC_CMD_NONE;
      }
      return;
    }
    // If previously performing lane change, try going back.
    if (*alc_state == ALC_CROSSING_LANE || prev_lc_state.crossed_boundary()) {
      QLOG(WARNING) << "Cannot drive back to the lane path before lane "
                       "change: already crossed boundary!";
      QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                                 QRunEvent::PLC_REJECT_RETURN_CROSSED_LANE);
      return;
    }
    if (prev_lp_before_lc.IsEmpty()) {
      QLOG(WARNING) << "Cannot drive back to the lane path before lane "
                       "change: rejected by route!";
      QRUNEVENT_WITH_ENUM_NOTICE(
          QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
          QRunEvent::PLC_REJECT_RETURN_ORIGIN_UNAVAILABLE);
      return;
    }
    *preferred_lane_path = prev_lp_before_lc;
    *alc_state = ALC_RETURNING;
    *lc_cmd_state = prev_lc_state.lc_left() ? DriverAction::LC_CMD_RIGHT
                                            : DriverAction::LC_CMD_LEFT;
    QLOG(INFO) << "Driving back to the lane path before lane change!";
    return;
  }

  if (!preferred_lane_path->IsEmpty()) {
    QLOG(WARNING)
        << "Teleop lane change rejected: previous teleop not completed!";
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_REJECT_PREV_CMD_INCOMPLETE);
    return;
  }
  if (prev_lc_state.stage() != LCS_NONE) {
    QLOG(WARNING)
        << "Teleop lane change rejected: currently performing lane change!";
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_REJECT_PERFORMING_LANE_CHANGE);
    return;
  }

  auto preferred_lane_path_or = FindPreferredLanePathFromTeleop(
      psmm, route_sections_from_start, route_navi_info, ego_proj_lane_id,
      new_lc_cmd);
  if (!preferred_lane_path_or.ok()) {
    QLOG(WARNING) << "Setting teleop lane change state failed: "
                  << preferred_lane_path_or.status().message();
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_REJECT_TARGET_NOT_FOUND);
    return;
  }
  if (preferred_lane_path_or->length() < kMinLcLaneLength) {
    QLOG(WARNING)
        << "Teleop lane change rejected: Length on route section not enough!";
    QRUNEVENT_WITH_ENUM_NOTICE(QRunEvent::KEY_QEVENT_PLC_REPORT_SIGNAL,
                               QRunEvent::PLC_REJECT_NOT_ON_ROUTE);
    return;
  }

  // New teleop command incoming, set lane path and state.
  *preferred_lane_path = std::move(*preferred_lane_path_or);
  *alc_state =
      new_lc_cmd == DriverAction::LC_CMD_STRAIGHT ? ALC_OFF : ALC_ONGOING;
  *lc_cmd_state = new_lc_cmd;
  QLOG(INFO) << "Processed new teleop lane change command: "
             << DriverAction_LaneChangeCommand_Name(new_lc_cmd);
}

void HandleAlcUserResponse(const PlannerSemanticMapManager& psmm,
                           const RouteSections& route_sections_from_start,
                           const RouteNaviInfo& route_navi_info,
                           const mapping::LanePath& prev_target_lane_path,
                           const LaneChangeStateProto& prev_lc_state,
                           std::optional<bool> alc_confirmation,
                           std::optional<bool> alc_request_lc_left,
                           mapping::LanePath* preferred_lane_path,
                           QALCState* alc_state,
                           DriverAction::LaneChangeCommand* lc_cmd_state) {
  if (!alc_confirmation.has_value() || !*alc_confirmation) return;

  if (*lc_cmd_state != DriverAction::LC_CMD_NONE ||
      !preferred_lane_path->IsEmpty()) {
    QLOG(WARNING) << "ALC Confirm ignored: previous teleop not completed!";
    return;
  }
  if (prev_lc_state.stage() != LCS_NONE) {
    QLOG(WARNING) << "ALC Confirm ignored: currently performing lane change!";
    return;
  }
  if (!alc_request_lc_left.has_value()) {
    QLOG(WARNING) << "ALC Confirm ignored: No auto lane change request!";
    return;
  }

  const auto new_lc_cmd = *alc_request_lc_left ? DriverAction::LC_CMD_LEFT
                                               : DriverAction::LC_CMD_RIGHT;

  constexpr double kPreviewLength = 15.0;  // m.
  const auto previewed_lane_index_point =
      prev_target_lane_path.ArclengthToLaneIndexPoint(kPreviewLength);
  const auto previewed_sections = RouteSections::BuildFromLanePath(
      psmm,
      prev_target_lane_path.AfterLaneIndexPoint(previewed_lane_index_point));

  auto preferred_lane_path_or = FindPreferredLanePathFromTeleop(
      psmm, previewed_sections, route_navi_info,
      prev_target_lane_path.lane_id(previewed_lane_index_point.first),
      new_lc_cmd);
  if (!preferred_lane_path_or.ok()) {
    QLOG(WARNING)
        << "Setting auto lane change state failed in finding target lane: "
        << preferred_lane_path_or.status().message();
    return;
  }
  const auto clamped_route_sections_or = ClampRouteSectionsBeforeArcLength(
      psmm, route_sections_from_start, 2.0 * kPreviewLength);
  if (!clamped_route_sections_or.ok()) {
    QLOG(WARNING)
        << "Setting auto lane change state failed in clamping route sections: "
        << clamped_route_sections_or.status().message();
    return;
  }
  auto backward_ext_preferred_lane_path_or =
      BackwardExtendLanePathOnRouteSections(psmm, *clamped_route_sections_or,
                                            *preferred_lane_path_or,
                                            kPreviewLength);
  if (!backward_ext_preferred_lane_path_or.ok()) {
    QLOG(WARNING) << "Setting auto lane change state failed in backward "
                     "extending target lane: "
                  << preferred_lane_path_or.status().message();
    return;
  }
  // New auto lc command incoming, set lane path and state.
  *preferred_lane_path = std::move(*backward_ext_preferred_lane_path_or);
  *alc_state = ALC_ONGOING;
  *lc_cmd_state = new_lc_cmd;
  QLOG(INFO) << "Processed auto lane change response: "
             << DriverAction_LaneChangeCommand_Name(new_lc_cmd);
}

absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections,
                            const Vec2d& pos, double projection_range,
                            double keep_behind_length) {
  SCOPED_QTRACE("ProjectPointToRouteSections");

  ASSIGN_OR_RETURN(const auto project_route_sections,
                   ClampRouteSectionsBeforeArcLength(psmm, route_sections,
                                                     projection_range));

  ASSIGN_OR_RETURN(auto point_proj,
                   FindSmoothPointOnRouteSectionsByDrivePassage(
                       psmm, project_route_sections, pos));

  std::vector<mapping::SectionId> sec_ids;
  for (int i = point_proj.section_idx; i < project_route_sections.size(); ++i) {
    sec_ids.push_back(project_route_sections.route_section_segment(i).id);
  }
  const RouteSections projected_route_sections_from_start(
      point_proj.fraction, project_route_sections.end_fraction(),
      std::move(sec_ids), project_route_sections.destination());

  ASSIGN_OR_RETURN(
      auto sections_from_start,
      AlignRouteSections(route_sections, projected_route_sections_from_start));

  RouteSections sections_with_behind;

  if (point_proj.accum_s > keep_behind_length) {
    ASSIGN_OR_RETURN(
        sections_with_behind,
        ClampRouteSectionsAfterArcLength(
            psmm, route_sections, point_proj.accum_s - keep_behind_length));
  } else {
    sections_with_behind = route_sections;
  }

  return std::make_tuple(std::move(sections_from_start),
                         std::move(sections_with_behind), point_proj);
}

}  // namespace planner
}  // namespace qcraft
