
#include "onboard/control/math/coordinate_transform.h"

#include <cmath>

#include "onboard/math/util.h"

namespace qcraft::control {

TrajectoryProto TrajectorySmoothToVehicle(
    double rear_axle_to_cg, const CoordinatePoint& smooth_pose_point,
    const TrajectoryProto& smooth_trajectory) {
  if (smooth_trajectory.trajectory_point().empty()) return smooth_trajectory;

  TrajectoryProto vehicle_trajectory = smooth_trajectory;
  vehicle_trajectory.mutable_trajectory_point()->Clear();
  for (int i = 0; i < smooth_trajectory.trajectory_point_size(); ++i) {
    const auto& smooth_tp = smooth_trajectory.trajectory_point();
    const CoordinatePoint smooth_point = {
        .x = smooth_tp.Get(i).path_point().x(),
        .y = smooth_tp.Get(i).path_point().y(),
        .yaw = NormalizeAngle(smooth_tp.Get(i).path_point().theta())};
    const CoordinatePoint vehicle_point =
        SmoothToVehicle(rear_axle_to_cg, smooth_pose_point, smooth_point);
    const auto vehicle_tp =
        vehicle_trajectory.mutable_trajectory_point()->Add();
    vehicle_tp->set_relative_time(smooth_tp.Get(i).relative_time());
    vehicle_tp->set_a(smooth_tp.Get(i).a());
    vehicle_tp->set_j(smooth_tp.Get(i).j());
    vehicle_tp->set_v(smooth_tp.Get(i).v());
    vehicle_tp->mutable_path_point()->set_x(vehicle_point.x);
    vehicle_tp->mutable_path_point()->set_y(vehicle_point.y);
    vehicle_tp->mutable_path_point()->set_z(smooth_tp.Get(i).path_point().z());
    vehicle_tp->mutable_path_point()->set_theta(vehicle_point.yaw);
    vehicle_tp->mutable_path_point()->set_s(smooth_tp.Get(i).path_point().s());
    vehicle_tp->mutable_path_point()->set_kappa(
        smooth_tp.Get(i).path_point().kappa());
    vehicle_tp->mutable_path_point()->set_lambda(
        smooth_tp.Get(i).path_point().lambda());
  }
  return vehicle_trajectory;
}

CoordinatePoint SmoothToVehicle(double rear_axle_to_cg,
                                const CoordinatePoint& smooth_pose,
                                const CoordinatePoint& smooth_point) {
  const CoordinatePoint smooth_cg =
      ComputeCGSmooth(rear_axle_to_cg, smooth_pose);

  CoordinatePoint vehicle_point;
  const double delta_x = smooth_point.x - smooth_cg.x;
  const double delta_y = smooth_point.y - smooth_cg.y;

  vehicle_point.x =
      delta_x * std::cos(-smooth_cg.yaw) - delta_y * std::sin(-smooth_cg.yaw);
  vehicle_point.y =
      delta_x * std::sin(-smooth_cg.yaw) + delta_y * std::cos(-smooth_cg.yaw);
  vehicle_point.yaw = NormalizeAngle(smooth_point.yaw - smooth_cg.yaw);
  return vehicle_point;
}

CoordinatePoint VehicleToSmooth(double rear_axle_to_cg,
                                const CoordinatePoint& smooth_pose,
                                const CoordinatePoint& vehicle_point) {
  const CoordinatePoint smooth_cg =
      ComputeCGSmooth(rear_axle_to_cg, smooth_pose);

  CoordinatePoint smooth_point;
  const double delta_x = vehicle_point.x * std::cos(smooth_cg.yaw) -
                         vehicle_point.y * std::sin(smooth_cg.yaw);
  const double delta_y = vehicle_point.x * std::sin(smooth_cg.yaw) +
                         vehicle_point.y * std::cos(smooth_cg.yaw);

  smooth_point.x = smooth_cg.x + delta_x;
  smooth_point.y = smooth_cg.y + delta_y;
  smooth_point.yaw = NormalizeAngle(vehicle_point.yaw + smooth_cg.yaw);
  return smooth_point;
}

CoordinatePoint ComputeCGSmooth(double rear_axle_to_cg,
                                const CoordinatePoint& smooth_pose) {
  CoordinatePoint smooth_cg;
  smooth_cg.x = smooth_pose.x + rear_axle_to_cg * std::cos(smooth_pose.yaw);
  smooth_cg.y = smooth_pose.y + rear_axle_to_cg * std::sin(smooth_pose.yaw);
  smooth_cg.yaw = NormalizeAngle(smooth_pose.yaw);
  return smooth_cg;
}

}  // namespace qcraft::control
