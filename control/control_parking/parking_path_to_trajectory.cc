#include "onboard/control/control_parking/parking_path_to_trajectory.h"

#include <cmath>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/math/util.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

absl::Status ParkingPathToTrajectory(const VehicleModel& vehicle_model,
                                     TrajectoryProto* trajectory) {
  // If not freespace not path to trajectory.
  if (!trajectory->low_speed_freespace() ||
      (vehicle_model != VEHICLE_QCRAFTVEHICLE_SUV &&
       vehicle_model != VEHICLE_SERES_SF5_SUV))
    return absl::OkStatus();

  // Update trajectory: change parking path to trajectory.
  if (!trajectory->has_directional_path() ||
      trajectory->directional_path().path().empty())
    return absl::InternalError("planner parking path is empty !");

  trajectory->clear_trajectory_point();
  const auto& parking_path = trajectory->directional_path().path();
  for (int i = 0; i < parking_path.size(); ++i) {
    auto* trajectory_point = trajectory->mutable_trajectory_point()->Add();
    trajectory_point->set_relative_time(planner::kTrajectoryTimeStep * i);
    trajectory_point->set_a(0.0);
    trajectory_point->set_j(0.0);
    trajectory_point->set_v(0.0);
    trajectory_point->mutable_path_point()->set_x(parking_path[i].x());
    trajectory_point->mutable_path_point()->set_y(parking_path[i].y());
    trajectory_point->mutable_path_point()->set_z(parking_path[i].z());
    trajectory_point->mutable_path_point()->set_s(parking_path[i].s());
    trajectory_point->mutable_path_point()->set_lambda(
        parking_path[i].lambda());
    // If parking path forward, hold on heading value. [0~180)、[-180,0]
    // If parking path backward, change heading value + pi.
    const double heading = trajectory->directional_path().forward()
                               ? NormalizeAngle(parking_path[i].theta())
                               : NormalizeAngle(parking_path[i].theta() + M_PI);
    trajectory_point->mutable_path_point()->set_theta(heading);

    // If parking path backward and not stationary steer, kappa = -kappa.
    const double kappa = !trajectory->directional_path().forward()
                             ? -parking_path[i].kappa()
                             : parking_path[i].kappa();
    trajectory_point->mutable_path_point()->set_kappa(kappa);
  }
  return absl::OkStatus();
}

}  // namespace control
}  // namespace qcraft
