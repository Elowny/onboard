#ifndef ONBOARD_CONTROL_TRAJECTORY_INTERFACE_H_
#define ONBOARD_CONTROL_TRAJECTORY_INTERFACE_H_

#include <vector>
// IWYU pragma: no_include <algorithm>  // for max
#include "absl/status/status.h"

#include "onboard/control/control_util.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::control {

class TrajectoryInterface {
 public:
  explicit TrajectoryInterface(const VehicleModel& vehicle_model)
      : has_physical_steering_wheel_(HasPhysicalSteeringWheel(vehicle_model)),
        vehicle_model_(vehicle_model) {}

  absl::Status Update(bool is_emergency_to_stop,
                      const TrajectoryProto& planning_published_trajectory,
                      ControllerDebugProto* controller_debug_proto);

  ApolloTrajectoryPointProto QueryNearestTrajPointByXY(const Vec2d& xy) const;

  ApolloTrajectoryPointProto QueryTrajPointByRelativeTime(
      double relative_time) const;

  // It is designed for lateral control. Allow to search a reference path
  // point beyond planner published trajectory range
  ApolloTrajectoryPointProto QueryTrajPointBasedOnPathS(double s) const;

  const std::vector<ApolloTrajectoryPointProto>& GetAllTrajPoints() const;

  double GetPlannerStartTime() const;

  double GetMinAccelFromTrajectory() const;

  bool GetIsStationaryTrajectory() const { return is_stationary_trajectory_; }

  bool aeb_triggered() const { return aeb_triggered_; }

  bool GetIsLowSpeedFreespace() const { return is_low_speed_freespace_; }

  bool GetEnableStationarySteering() const {
    return enable_stationary_steering_;
  }

  double accumulate_s() const {
    return all_traj_points_current_.traj_points.back().path_point().s();
  }

  double GetStationarySteerRefKappa() const {
    return all_traj_points_current_.traj_points[0].path_point().kappa();
  }

  double GetParkingStopS() const { return parking_stop_s_; }

  bool GetIsParkingForward() const { return is_parking_forward_; }

  // Planner start time with the trajectory points.
  struct TrajectoryInfo {
    double plan_start_time;
    std::vector<ApolloTrajectoryPointProto> traj_points;
  };

 private:
  TrajectoryInfo all_traj_points_current_;
  TrajectoryInfo all_traj_points_previous_;

  bool has_physical_steering_wheel_;
  VehicleModel vehicle_model_;
  Chassis::GearPosition trajectory_gear_ = Chassis::GEAR_DRIVE;
  bool aeb_triggered_ = false;
  bool is_stationary_trajectory_ = false;
  bool is_low_speed_freespace_ = false;
  bool enable_stationary_steering_ = false;
  bool is_parking_forward_ = false;
  double parking_stop_s_ = 0.0;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_TRAJECTORY_INTERFACE_H_
