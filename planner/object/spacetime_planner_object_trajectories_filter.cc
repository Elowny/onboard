#include "onboard/planner/object/spacetime_planner_object_trajectories_filter.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/util/perception_util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

absl::Status IsMaybeCutInVehicleTrajectory(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const Box2d& av_box, const FrenetBox& /*av_sl_box*/, double av_speed,
    ObjectType object_type, const Box2d& object_box,
    std::string_view object_traj_id,
    const SecondOrderTrajectoryPoint& object_start_pose,
    const SecondOrderTrajectoryPoint& object_end_pose) {
  if (object_type != ObjectType::OT_VEHICLE &&
      object_type != ObjectType::OT_LARGE_VEHICLE) {
    return absl::InternalError("Object is not vehicle.");
  }
  const PiecewiseLinearFunction<double, double> av_speed_close_moving_dist_plf =
      {{0.0, 10.0, 20.0}, {0.3, 0.7, 0.9}};
  const double object_dist_threshold = av_speed_close_moving_dist_plf(av_speed);
  const double dist_to_av_box = object_box.DistanceTo(av_box);
  if (dist_to_av_box < object_dist_threshold) {
    VLOG(2) << object_traj_id << " dist too close " << dist_to_av_box << "m.";
    return absl::InternalError("Object too close to av.");
  }

  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   drive_passage.QueryFrenetBoxAt(object_box));
  ASSIGN_OR_RETURN(const auto current_object_sl_pt,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       object_start_pose.pos()));
  ASSIGN_OR_RETURN(const auto current_lane_heading,
                   drive_passage.QueryTangentAngleAtS(current_object_sl_pt.s));
  ASSIGN_OR_RETURN(const auto final_object_sl_pt,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       object_end_pose.pos()));

  // Just pay attention to object outside lane.
  // TODO(runbing): Use prev info to check cut in state for object inside lane.
  ASSIGN_OR_RETURN(
      const auto current_lane_boundary_offset,
      drive_passage.QueryNearestBoundaryLateralOffset(current_object_sl_pt.s));
  const auto current_path_sl_boundary =
      path_sl_boundary.QueryTargetBoundaryL(current_object_sl_pt.s);
  const double left_boundary = std::min(current_path_sl_boundary.second,
                                        current_lane_boundary_offset.second);
  const double right_boundary = std::max(current_path_sl_boundary.first,
                                         current_lane_boundary_offset.first);
  if (object_frenet_box.l_max <= left_boundary &&
      object_frenet_box.l_min >= right_boundary) {
    VLOG(2) << object_traj_id << " object in lane. ";
    return absl::InternalError("Object in lane");
  }

  // If traj is slope to center.
  constexpr double kCenterOffsetThreshold = 0.8;  // m
  bool is_traj_slope_to_center = false;
  if (object_frenet_box.l_min < right_boundary) {
    is_traj_slope_to_center = final_object_sl_pt.l > right_boundary &&
                              final_object_sl_pt.l > current_object_sl_pt.l;
  } else if (object_frenet_box.l_max > left_boundary) {
    is_traj_slope_to_center = final_object_sl_pt.l < left_boundary &&
                              final_object_sl_pt.l < current_object_sl_pt.l;
  }
  if (is_traj_slope_to_center) {
    // Heading matching: If toward center ref line diff is large, object may
    // be cut-in.
    constexpr double kCutInThetaThreshold = 0.05;  // rad
    const double toward_center_theta_diff =
        current_object_sl_pt.l > 0.0
            ? NormalizeAngle(current_lane_heading - object_start_pose.theta())
            : NormalizeAngle(object_start_pose.theta() - current_lane_heading);
    if (toward_center_theta_diff > kCutInThetaThreshold) {
      return absl::OkStatus();
    }

    const double toward_center_offset =
        current_object_sl_pt.l > 0.0
            ? current_object_sl_pt.l - final_object_sl_pt.l
            : final_object_sl_pt.l - current_object_sl_pt.l;
    VLOG(3) << object_traj_id
            << " toward_center_offset: " << toward_center_offset;
    // If vehicle is predicted to get closer to center line and heading toward
    // center line, object might be cutting in.
    if (toward_center_offset > kCenterOffsetThreshold) {
      VLOG(2) << object_traj_id << " prediction toward center offset "
              << toward_center_offset << "m > " << kCenterOffsetThreshold
              << "m, maybe cutin traj, refuse to nudge.";
      return absl::OkStatus();
    }
  }
  return absl::InternalError("Object lane keeping");
}

bool IsCrossingTrajectory(const DrivePassage& drive_passage,
                          const SpacetimeObjectTrajectory& traj,
                          const SpacetimePlannerObjectTrajectoriesParamsProto::
                              CrossingFilterParamsProto& params) {
  if (traj.is_stationary() || IsStaticObjectType(traj.object_type())) {
    return false;
  }
  const auto& traj_start_point = *traj.states().front().traj_point;
  const auto frenet_start_point =
      drive_passage.QueryUnboundedFrenetCoordinateAt(traj_start_point.pos());
  if (!frenet_start_point.ok()) {
    return false;
  }
  const auto lane_theta_at_pose =
      drive_passage.QueryTangentAngleAtS(frenet_start_point->s);
  if (!lane_theta_at_pose.ok()) {
    return false;
  }
  // Compute thetas perpendicular to current lane.
  const double lane_normal_theta_at_pose =
      NormalizeAngle(*lane_theta_at_pose + M_PI_2);
  const double lane_negative_normal_theta_at_pose =
      NormalizeAngle(*lane_theta_at_pose - M_PI_2);

  // If all angles between trajectory points theta and lane theta is less
  // than 1.0rad(60°), most(60%) angles is less than 0.78rad(45°) and a
  // little((40%)) angles is less than 0.52(30°), we think it is a crossing
  // trajectory.
  const auto& theta_diff_thresholds = params.theta_diff_thresholds();
  const auto& ratio_limits = params.ratio_limits();
  const int level_size = theta_diff_thresholds.size();
  std::vector<int> level_counts(level_size, 0);

  constexpr double kCheckTime = 5.0;  // s.
  const auto& states = traj.states();
  int state_count = 0;
  for (const auto& state : states) {
    if (state.traj_point->t() > kCheckTime) break;
    ++state_count;
    const double theta = state.traj_point->theta();
    const double abs_theta_diff_to_lane_normal =
        std::abs(NormalizeAngle(theta - lane_normal_theta_at_pose));
    const double abs_theta_diff_to_negative_lane_normal =
        std::abs(NormalizeAngle(theta - lane_negative_normal_theta_at_pose));
    for (int j = 0; j < level_size; ++j) {
      const double theta_diff_threshold = theta_diff_thresholds[j];
      if (abs_theta_diff_to_lane_normal < theta_diff_threshold ||
          abs_theta_diff_to_negative_lane_normal < theta_diff_threshold) {
        level_counts[j]++;
      }
    }
  }
  for (int j = 0; j < level_size; ++j) {
    const double ratio =
        static_cast<double>(level_counts[j]) / static_cast<double>(state_count);
    if (ratio < ratio_limits[j]) {
      return false;
    }
  }
  return true;
}

bool IsTrajectoryBeyondStopLine(const DrivePassage& drive_passage,
                                double first_stop_line_s,
                                const SpacetimeObjectTrajectory& traj) {
  if (std::isinf(first_stop_line_s)) {
    return false;
  }
  if (traj.is_stationary()) {
    const auto frenet_box = drive_passage.QueryFrenetBoxAt(traj.bounding_box());
    if (!frenet_box.ok()) {
      return false;
    }
    if (frenet_box->s_min < first_stop_line_s) {
      return false;
    }
  } else {
    const auto& states = traj.states();
    for (const auto& state : states) {
      const auto frenet_box = drive_passage.QueryFrenetBoxAt(state.box);
      if (!frenet_box.ok()) {
        continue;
      }
      if (frenet_box->s_min < first_stop_line_s) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

CutInVehicleSpacetimePlannerObjectTrajectoriesFilter::
    CutInVehicleSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const PathSlBoundary* path_sl_boundary, const Box2d& av_box,
        double av_speed)
    : drive_passage_(drive_passage),
      path_sl_boundary_(path_sl_boundary),
      av_box_(av_box),
      av_speed_(av_speed) {
  const auto av_sl_box_or = drive_passage_->QueryFrenetBoxAt(av_box_);
  if (av_sl_box_or.ok()) {
    av_sl_box_ = *av_sl_box_or;
  }
}

bool CutInVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!av_sl_box_.has_value()) {
    VLOG(2) << "AV box can't be mapped on drive passage, skip.";
    return false;
  }
  const auto res = IsMaybeCutInVehicleTrajectory(
      *drive_passage_, *path_sl_boundary_, av_box_, *av_sl_box_, av_speed_,
      traj.object_type(), traj.bounding_box(), traj.traj_id(), traj.pose(),
      *traj.states().back().traj_point);
  if (!res.ok()) {
    VLOG(2) << res.message();
  }
  return res.ok();
}

CrossingSpacetimePlannerObjectTrajectoriesFilter::
    CrossingSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const CrossingFilterParamsProto* crossing_filter_params)
    : drive_passage_(CHECK_NOTNULL(drive_passage)),
      params_(CHECK_NOTNULL(crossing_filter_params)) {}

bool CrossingSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  return IsCrossingTrajectory(*drive_passage_, traj, *params_);
}

BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter::
    BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        absl::Span<const ConstraintProto::StopLineProto> stop_lines)
    : drive_passage_(drive_passage) {
  for (const auto& stop_line : stop_lines) {
    first_stop_line_s_ =
        std::min(first_stop_line_s_, stop_line.s() - stop_line.standoff());
  }
}

bool BeyondStopLineSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  return IsTrajectoryBeyondStopLine(*drive_passage_, first_stop_line_s_, traj);
}

ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter::
    ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage* drive_passage,
        const PathSlBoundary* path_sl_boundary)
    : drive_passage_(drive_passage), path_sl_boundary_(path_sl_boundary) {}

bool ReverseVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!(traj.object_type() == ObjectType::OT_VEHICLE ||
        traj.object_type() == ObjectType::OT_LARGE_VEHICLE ||
        traj.object_type() == ObjectType::OT_UNKNOWN_MOVABLE)) {
    return false;
  }
  const auto object_frenet_box_output =
      drive_passage_->QueryFrenetBoxAtContour(traj.contour());
  if (!object_frenet_box_output.ok()) return false;
  const auto& object_frenet_box = object_frenet_box_output.value();
  const double object_s = object_frenet_box.center_s();
  const auto& nearest_station = drive_passage_->FindNearestStationAtS(object_s);
  if (nearest_station.is_in_intersection()) {
    return false;
  }
  const auto start_boundary =
      path_sl_boundary_->QueryBoundaryL(object_frenet_box.s_min);
  const auto end_boundary =
      path_sl_boundary_->QueryBoundaryL(object_frenet_box.s_max);

  constexpr double kMaxHalfLaneWidth = 2.0;  // m
  const double left_boundary = std::min(
      kMaxHalfLaneWidth, std::max(start_boundary.second, end_boundary.second));
  const double right_boundary = std::max(
      -kMaxHalfLaneWidth, std::min(start_boundary.first, end_boundary.first));

  constexpr double kThetaThreshold = 0.7;  // rad
  const bool object_overlap_with_boundary =
      (object_frenet_box.l_min > right_boundary &&
       object_frenet_box.l_min < left_boundary) ||
      (object_frenet_box.l_max > right_boundary &&
       object_frenet_box.l_max < left_boundary) ||
      (object_frenet_box.l_max < right_boundary &&
       object_frenet_box.l_max > left_boundary);

  const auto& states = traj.states();
  if (states.size() < 2) {
    return false;
  }
  const double vel_theta =
      (states[1].traj_point->pos() - states[0].traj_point->pos()).Angle();

  if (object_overlap_with_boundary &&
      std::abs(NormalizeAngle(nearest_station.tangent().FastAngle() + M_PI -
                              vel_theta)) < kThetaThreshold) {
    return true;
  }

  return false;
}

OccludedVehicleSpacetimePlannerObjectTrajectoriesFilter::
    OccludedVehicleSpacetimePlannerObjectTrajectoriesFilter() {}

bool OccludedVehicleSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  if (!(traj.object_type() == ObjectType::OT_VEHICLE ||
        traj.object_type() == ObjectType::OT_LARGE_VEHICLE ||
        traj.object_type() == ObjectType::OT_UNKNOWN_MOVABLE)) {
    return false;
  }
  if (IsOccludedCameraObject(traj.planner_object().object_proto()) ||
      IsOccludedLidarObject(traj.planner_object().object_proto())) {
    return true;
  }
  return false;
}

BehaviorConflictSpacetimePlannerObjectTrajectoriesFilter::
    BehaviorConflictSpacetimePlannerObjectTrajectoriesFilter(
        const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
        absl::Span<const PlannerObject> planner_objects, const Box2d& av_box,
        double av_speed) {
  const auto av_sl_box = drive_passage.QueryFrenetBoxAt(av_box);
  if (!av_sl_box.ok()) return;
  for (int idx = 0; idx < planner_objects.size(); ++idx) {
    const auto& planner_object = planner_objects[idx];
    const auto& trajectories = planner_object.prediction().trajectories();
    QCHECK(!trajectories.empty())
        << planner_object.id() << " has no trajectory.";
    bool has_cut_in_traj = false;
    std::vector<int> traj_cut_in_index;
    traj_cut_in_index.reserve(trajectories.size());
    for (int traj_index = 0, s = trajectories.size(); traj_index < s;
         ++traj_index) {
      const auto& traj = trajectories[traj_index];
      const auto res = IsMaybeCutInVehicleTrajectory(
          drive_passage, sl_boundary, av_box, *av_sl_box, av_speed,
          planner_object.type(), planner_object.bounding_box(),
          absl::StrFormat("%s_%d", planner_object.base_id(), traj_index),
          traj.points().front(), traj.points().back());
      if (res.ok()) {
        has_cut_in_traj = true;
        traj_cut_in_index.push_back(traj_index);
      }
    }
    if (has_cut_in_traj) {
      object_cutin_traj_set_[planner_object.id()] =
          std::move(traj_cut_in_index);
    }
  }
}

bool BehaviorConflictSpacetimePlannerObjectTrajectoriesFilter::Filter(
    const SpacetimeObjectTrajectory& traj) const {
  const auto& info_ptr =
      object_cutin_traj_set_.find(std::string(traj.object_id()));
  if (info_ptr != object_cutin_traj_set_.end() &&
      std::find(info_ptr->second.begin(), info_ptr->second.end(),
                traj.traj_index()) != info_ptr->second.end()) {
    return true;
  }
  return false;
}

}  // namespace planner
}  // namespace qcraft
