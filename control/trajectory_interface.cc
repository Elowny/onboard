#include "onboard/control/trajectory_interface.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

// IWYU pragma: no_include <boost/move/utility_core.hpp>  // for move

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include "Eigen/Core"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <ostream>
#include <vector>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/control/control_flags.h"
#include "onboard/control/control_parking/parking_path_to_trajectory.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kTrajPointInterval = 0.1;  // s.
constexpr int kEstopTrajPointSize = 50;
constexpr int kEstopPastPointSize = 20;

double PointDistanceSquare(const ApolloTrajectoryPointProto& point,
                           const Vec2d& xy) {
  const Vec2d point_xy{point.path_point().x(), point.path_point().y()};
  return point_xy.DistanceSquareTo(xy);
}

bool ApproximatelyEqual(double a, double b) {
  return std::fabs(a - b) < kEpsilon;
}

ApolloTrajectoryPointProto ExtrapolateTrajectoryPoint(
    const ApolloTrajectoryPointProto& trajectory_point, double delta_s) {
  const double kappa = trajectory_point.path_point().kappa();
  const double theta0 = trajectory_point.path_point().theta();
  const double delta_theta = kappa * delta_s;

  // Chord length of the circle.
  // Chord_length = 2 * R * sin(0.5 * delta_theta)
  //              ~= delta_s - delta_s * (delta_theta)^2 / 24.0;
  const double chord_length = delta_s - delta_s * Sqr(delta_theta) / 24.0;

  const Vec2d p0(trajectory_point.path_point().x(),
                 trajectory_point.path_point().y());
  const Vec2d p0_to_p1 =
      chord_length * Vec2d::UnitFromAngle(theta0 + 0.5 * delta_theta);
  const Vec2d p1 = p0 + p0_to_p1;

  ApolloTrajectoryPointProto p;
  p.mutable_path_point()->set_x(p1.x());
  p.mutable_path_point()->set_y(p1.y());
  p.mutable_path_point()->set_z(trajectory_point.path_point().z());
  p.mutable_path_point()->set_theta(NormalizeAngle(theta0 + delta_theta));
  p.mutable_path_point()->set_kappa(trajectory_point.path_point().kappa());
  p.mutable_path_point()->set_s(trajectory_point.path_point().s() + delta_s);
  p.set_v(trajectory_point.v());
  p.set_a(trajectory_point.a());
  p.set_relative_time(trajectory_point.relative_time());

  return p;
}

ApolloTrajectoryPointProto GenerateTrajectoryPointBasedOnPathS(
    absl::Span<const ApolloTrajectoryPointProto> traj_points,
    Chassis::GearPosition trajectory_gear, double s) {
  // Reverse driving condition, trajectory point.s() is negative, which needs
  // special processing.
  auto it_lower = std::lower_bound(
      traj_points.begin(), traj_points.end(), s,
      [trajectory_gear](const ApolloTrajectoryPointProto& p, double s) {
        if (trajectory_gear == Chassis::GEAR_REVERSE) {
          return p.path_point().s() > s;
        } else {
          return p.path_point().s() < s;
        }
      });
  if (it_lower == traj_points.begin() || it_lower == traj_points.end()) {
    QLOG_EVERY_N_SEC(WARNING, 5.0)
        << "Queried path point based on position is out of "
        << "planner trajectory range. ";
    if (it_lower == traj_points.end()) {
      it_lower -= 1;
    }
    const double delta_s = s - (*it_lower).path_point().s();
    VLOG(1) << "delta_s: " << delta_s;
    return ExtrapolateTrajectoryPoint(*it_lower, delta_s);
  }

  const auto& p0 = (*(it_lower - 1));
  const auto& p1 = (*it_lower);

  const double s0 = p0.path_point().s();
  const double s1 = p1.path_point().s();
  const double alpha = s0 == s1 ? 0.0 : (s - s0) / (s1 - s0);

  return planner::LerpTrajectoryPoint(p0, p1, alpha);
}

ApolloTrajectoryPointProto GenerateTrajectoryPointBasedOnRelativeTime(
    absl::Span<const ApolloTrajectoryPointProto> traj_points,
    double relative_time) {
  if (!InRange(relative_time, traj_points.begin()->relative_time(),
               traj_points.rbegin()->relative_time())) {
    QLOG_EVERY_N_SEC(WARNING, /*second*/ 1.0)
        << "relative_time, " << relative_time << ", is out of range [ "
        << traj_points.begin()->relative_time() << ", "
        << traj_points.rbegin()->relative_time() << "].";
  }

  relative_time =
      std::clamp(relative_time, traj_points.begin()->relative_time(),
                 traj_points.rbegin()->relative_time());

  auto it_lower =
      std::lower_bound(traj_points.begin(), traj_points.end(), relative_time,
                       [](const ApolloTrajectoryPointProto& p, double t) {
                         return p.relative_time() < t;
                       });
  VLOG(1) << "index_diff: " << it_lower - traj_points.begin() << '\n'
          << "s: " << (*it_lower).path_point().s();

  if (it_lower == traj_points.begin()) {
    return *it_lower;
  } else if (it_lower == traj_points.end()) {
    return *(it_lower - 1);
  }

  const ApolloTrajectoryPointProto& p0 = *(it_lower - 1);
  const ApolloTrajectoryPointProto& p1 = *it_lower;

  const double t0 = p0.relative_time();
  const double t1 = p1.relative_time();

  QCHECK_NE(t0, t1);
  const double alpha = (relative_time - t0) / (t1 - t0);

  return planner::LerpTrajectoryPoint(p0, p1, alpha);
}

std::vector<ApolloTrajectoryPointProto> ComputeEStopTrajectory(
    bool has_steering_wheel, double start_time_diff,
    Chassis::GearPosition trajectory_gear,
    absl::Span<const ApolloTrajectoryPointProto> all_traj_points_previous) {
  // TODO(zhichao): consider emergency stop at R-gear conditions.
  std::vector<ApolloTrajectoryPointProto> stop_trajectory_point;
  stop_trajectory_point.reserve(kEstopPastPointSize + kEstopTrajPointSize);
  double emergency_brake = 0.0;
  double emergency_jerk = 0.0;
  if (!has_steering_wheel) {
    emergency_brake = FLAGS_control_estop_hard_brake_acceleration;
    emergency_jerk = FLAGS_control_estop_hard_brake_jerk;
  } else {
    emergency_brake = FLAGS_control_estop_soft_brake_acceleration;
    emergency_jerk = FLAGS_control_estop_soft_brake_jerk;
  }
  // Compute the past points;
  for (int i = 0; i < kEstopPastPointSize + 1; ++i) {
    const double past_point_relative_time =
        (i - kEstopPastPointSize) * kTrajPointInterval + start_time_diff;
    const ApolloTrajectoryPointProto past_point =
        GenerateTrajectoryPointBasedOnRelativeTime(all_traj_points_previous,
                                                   past_point_relative_time);
    stop_trajectory_point.push_back(past_point);
    stop_trajectory_point[i].set_relative_time((i - kEstopPastPointSize) *
                                               kTrajPointInterval);
  }

  // Compute stop trajectory points with a constant longitudinal jerk, and
  // the minimal acceleration of each trajectory points is
  // control_estop_acceleration from gflag setting;
  const auto based_traj_point = stop_trajectory_point.back();
  double s = based_traj_point.path_point().s();
  double v = based_traj_point.v();
  double a = based_traj_point.a();

  for (int i = 1; i < kEstopTrajPointSize; ++i) {
    const double a_prev =
        std::min(a, 0.0);  // Prevent acceleration during the estop.
    const double v_prev = v;

    a = std::max(emergency_brake, a_prev + emergency_jerk * kTrajPointInterval);
    const double j = (a - a_prev) / kTrajPointInterval;
    v += 0.5 * (a + a_prev) * kTrajPointInterval;
    v = std::max(0.0, v);
    s += 0.5 * (v + v_prev) * kTrajPointInterval;

    // If the emergency trajectory point.s is larger than the original
    // trajectory, just copy the trajectory point from the original trajectory.
    const double relative_time = start_time_diff + i * kTrajPointInterval;
    auto original_traj_point = GenerateTrajectoryPointBasedOnRelativeTime(
        all_traj_points_previous, relative_time);

    if (s > original_traj_point.path_point().s()) {
      original_traj_point.set_relative_time(i * kTrajPointInterval);
      stop_trajectory_point.push_back(original_traj_point);
      continue;
    }

    auto point = GenerateTrajectoryPointBasedOnPathS(all_traj_points_previous,
                                                     trajectory_gear, s);
    point.set_v(v);
    point.set_a(a);
    point.set_j(j);
    point.set_relative_time(i * kTrajPointInterval);

    stop_trajectory_point.push_back(point);
  }

  return stop_trajectory_point;
}

int FindNearestPointIndex(
    absl::Span<const ApolloTrajectoryPointProto> all_traj_points,
    const Vec2d& xy) {
  unsigned int index_min = 0;
  double dist_min = PointDistanceSquare(all_traj_points[0], xy);
  for (int i = 1; i < all_traj_points.size(); ++i) {
    const double dist_temp = PointDistanceSquare(all_traj_points[i], xy);
    if (dist_temp < dist_min) {
      index_min = i;
      dist_min = dist_temp;
    }
  }
  return index_min;
}

bool JudgeIsStationaryTrajectory(
    const TrajectoryProto& planning_published_trajectory) {
  const auto& first_path_point =
      planning_published_trajectory.trajectory_point(0).path_point();
  for (const auto& point : planning_published_trajectory.trajectory_point()) {
    if (!ApproximatelyEqual(point.v(), 0.0) ||
        !ApproximatelyEqual(point.a(), 0.0) ||
        !ApproximatelyEqual(point.j(), 0.0) ||
        !ApproximatelyEqual(point.path_point().x(), first_path_point.x()) ||
        !ApproximatelyEqual(point.path_point().y(), first_path_point.y()) ||
        !ApproximatelyEqual(point.path_point().theta(),
                            first_path_point.theta()) ||
        !ApproximatelyEqual(point.path_point().kappa(),
                            first_path_point.kappa())) {
      return false;
    }
  }
  return true;
}

absl::Status CheckPublishedTrajectory(
    const TrajectoryProto& planning_published_trajectory) {
  if (planning_published_trajectory.trajectory_point().empty()) {
    return absl::InvalidArgumentError(
        "Trajectory check fails: empty trajectory point;");
  }

  if (!planning_published_trajectory.has_gear()) {
    return absl::InvalidArgumentError(
        "Trajectory check fails: no gear command;");
  }

  return absl::OkStatus();
}

TrajectoryInterface::TrajectoryInfo RefreshTrajectoryPoints(
    double header_time, const TrajectoryProto& planning_published_trajectory) {
  TrajectoryInterface::TrajectoryInfo traj_info;
  traj_info.plan_start_time = header_time;

  const int trajectory_point_size =
      planning_published_trajectory.trajectory_point_size();
  const int past_points_size = planning_published_trajectory.past_points_size();
  traj_info.traj_points.reserve(trajectory_point_size + past_points_size);
  VLOG(1) << "all_traj_points_size: " << traj_info.traj_points.size();

  // Merge trajectory points and past points into all traj points.
  for (int i = 0; i < past_points_size; ++i) {
    traj_info.traj_points.push_back(
        planning_published_trajectory.past_points(i));
  }

  for (int i = 0; i < trajectory_point_size; ++i) {
    traj_info.traj_points.push_back(
        planning_published_trajectory.trajectory_point(i));
  }

  // Recalculate s of the trajectory_points.
  double s_update = 0.0;
  traj_info.traj_points[0].mutable_path_point()->set_s(s_update);
  for (int i = 1; i < traj_info.traj_points.size(); ++i) {
    const auto& traj_point_0 = traj_info.traj_points[i - 1];
    const auto& traj_point_1 = traj_info.traj_points[i];
    const Vec2d p0(traj_point_0.path_point().x(),
                   traj_point_0.path_point().y());
    const Vec2d p1(traj_point_1.path_point().x(),
                   traj_point_1.path_point().y());

    s_update += p0.DistanceTo(p1);
    traj_info.traj_points[i].mutable_path_point()->set_s(s_update);
  }

  // Consider reverse driving condition, s in future trajectory point with
  // GEAR_REVERSE should be negative, otherwise positive, it doesn't matter
  // when standstill, only have a valid point with s = 0.0.
  // 1.0 for drive forward, -1.0 for drive backwards.
  const double driving_direction =
      planning_published_trajectory.gear() == Chassis::GEAR_REVERSE ? -1.0
                                                                    : 1.0;
  const double s_base =
      traj_info.traj_points[past_points_size].path_point().s();
  for (int i = 0; i < traj_info.traj_points.size(); ++i) {
    const double s = traj_info.traj_points[i].path_point().s();
    traj_info.traj_points[i].mutable_path_point()->set_s((s - s_base) *
                                                         driving_direction);
    VLOG(1) << "all_traj_points_current[" << i
            << "].s = " << traj_info.traj_points[i].mutable_path_point()->s();
  }
  return traj_info;
}

}  // namespace

const std::vector<ApolloTrajectoryPointProto>&
TrajectoryInterface::GetAllTrajPoints() const {
  return all_traj_points_current_.traj_points;
}

double TrajectoryInterface::GetPlannerStartTime() const {
  return all_traj_points_current_.plan_start_time;
}

absl::Status TrajectoryInterface::Update(
    bool is_emergency_to_stop,
    const TrajectoryProto& planning_published_trajectory,
    ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("TrajectoryInterface::Update");

  TrajectoryProto trajectory = planning_published_trajectory;
  RETURN_IF_ERROR(ParkingPathToTrajectory(vehicle_model_, &trajectory));

  RETURN_IF_ERROR(CheckPublishedTrajectory(trajectory));

  const double header_time_input = trajectory.has_trajectory_start_timestamp()
                                       ? trajectory.trajectory_start_timestamp()
                                       : trajectory.header().timestamp() * 1e-6;

  aeb_triggered_ = trajectory.aeb_triggered();
  is_stationary_trajectory_ = JudgeIsStationaryTrajectory(trajectory);
  trajectory_gear_ = trajectory.gear();
  is_low_speed_freespace_ = trajectory.low_speed_freespace();
  enable_stationary_steering_ = trajectory.enable_stationary_steering();
  is_parking_forward_ = trajectory.directional_path().forward();
  parking_stop_s_ = trajectory.stop_s();

  controller_debug_proto->set_in_emergency_stop_state(is_emergency_to_stop);
  if (is_emergency_to_stop) {
    if (all_traj_points_previous_.traj_points.empty()) {
      all_traj_points_current_ =
          RefreshTrajectoryPoints(header_time_input, trajectory);

      all_traj_points_previous_ = all_traj_points_current_;
    }

    constexpr double kStopTrajUpdatePeriod = 0.2;
    const bool update_estop_traj =
        ToUnixDoubleSeconds(Clock::Now()) >
        all_traj_points_current_.plan_start_time + kStopTrajUpdatePeriod;
    if (update_estop_traj) {
      all_traj_points_current_.plan_start_time =
          ToUnixDoubleSeconds(Clock::Now());
      const double start_time_diff = all_traj_points_current_.plan_start_time -
                                     all_traj_points_previous_.plan_start_time;

      all_traj_points_current_.traj_points = ComputeEStopTrajectory(
          has_physical_steering_wheel_, start_time_diff, trajectory_gear_,
          all_traj_points_previous_.traj_points);

      all_traj_points_previous_ = all_traj_points_current_;
    }
    QCHECK_GT(all_traj_points_current_.traj_points.size(), 0);

    const int curr_traj_size = all_traj_points_current_.traj_points.size();
    const int past_points_size = curr_traj_size > kEstopTrajPointSize
                                     ? curr_traj_size - kEstopTrajPointSize
                                     : 0;

    for (int i = past_points_size; i < curr_traj_size && update_estop_traj;
         ++i) {
      *controller_debug_proto->add_emergency_stop_trajectory() =
          all_traj_points_current_.traj_points[i];
    }

    return absl::OkStatus();
  }

  QCHECK(!trajectory.trajectory_point().empty())
      << "Planner trajectory point is empty.";

  // When obtaining new planner trajectory, update both all_traj_points_current_
  // and all_traj_points_previous_;
  if (all_traj_points_current_.traj_points.empty() ||
      std::fabs(all_traj_points_current_.plan_start_time - header_time_input) >
          kEpsilon) {
    all_traj_points_previous_ = all_traj_points_current_;
    all_traj_points_current_ =
        RefreshTrajectoryPoints(header_time_input, trajectory);
  }

  // In the first cycle, the all_traj_points_previous_ will be empty.
  if (all_traj_points_previous_.traj_points.empty()) {
    all_traj_points_previous_ = all_traj_points_current_;
  }

  return absl::OkStatus();
}

ApolloTrajectoryPointProto TrajectoryInterface::QueryNearestTrajPointByXY(
    const Vec2d& xy) const {
  SCOPED_QTRACE("TrajectoryInterface::QueryNearestTrajPointByXY");
  const int nearest_point_index = FindNearestPointIndex(GetAllTrajPoints(), xy);
  VLOG(1) << "nearest_point_index: " << nearest_point_index;
  const auto closest_path_point =
      all_traj_points_current_.traj_points[nearest_point_index].path_point();

  const Vec2d closest_path_point_xy(closest_path_point.x(),
                                    closest_path_point.y());
  const Vec2d closest_path_point_tangent =
      Vec2d::UnitFromAngle(closest_path_point.theta());
  const double closest_path_point_s = closest_path_point.s();
  const double current_s =
      closest_path_point_s +
      (xy - closest_path_point_xy).dot(closest_path_point_tangent);

  return QueryTrajPointBasedOnPathS(current_s);
}

ApolloTrajectoryPointProto TrajectoryInterface::QueryTrajPointBasedOnPathS(
    double s) const {
  return GenerateTrajectoryPointBasedOnPathS(GetAllTrajPoints(),
                                             trajectory_gear_, s);
}

ApolloTrajectoryPointProto TrajectoryInterface::QueryTrajPointByRelativeTime(
    double relative_time) const {
  return GenerateTrajectoryPointBasedOnRelativeTime(GetAllTrajPoints(),
                                                    relative_time);
}

double TrajectoryInterface::GetMinAccelFromTrajectory() const {
  const auto traj_point = std::min_element(
      all_traj_points_current_.traj_points.begin(),
      all_traj_points_current_.traj_points.end(),
      [](const ApolloTrajectoryPointProto& p0,
         const ApolloTrajectoryPointProto& p1) { return p0.a() < p1.a(); });
  return traj_point->a();
}

}  // namespace qcraft::control
