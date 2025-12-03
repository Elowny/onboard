#ifndef ONBOARD_CONTROL_MATH_COORDINATE_TRANSFORM_H_
#define ONBOARD_CONTROL_MATH_COORDINATE_TRANSFORM_H_

#include "onboard/proto/trajectory.pb.h"

// https://qcraft.feishu.cn/docx/VBMrdZgd7ocj6CxiuPGcE80GnYf

namespace qcraft::control {

struct CoordinatePoint {
  double x = 0.0;    // m
  double y = 0.0;    // m
  double yaw = 0.0;  // rad [-pi,pi)
};

TrajectoryProto TrajectorySmoothToVehicle(
    double rear_axle_to_cg, const CoordinatePoint& smooth_pose,
    const TrajectoryProto& smooth_trajectory);

// Smooth to vehicle coordinate:
// smooth_pose_point: vehicle rear axle center in Smooth coordinate.
// smooth_transform_point: tranjectory point in Smooth coordinate.
CoordinatePoint SmoothToVehicle(double rear_axle_to_cg,
                                const CoordinatePoint& smooth_pose,
                                const CoordinatePoint& smooth_point);

// Vehicle to smooth coordinate:
// smooth_pose_point: vehicle rear axle center in Smooth coordinate.
// vehicle_transform_point: predict point in Vehicle coordinate.
CoordinatePoint VehicleToSmooth(double rear_axle_to_cg,
                                const CoordinatePoint& smooth_pose,
                                const CoordinatePoint& vehicle_point);

CoordinatePoint ComputeCGSmooth(double rear_axle_to_cg,
                                const CoordinatePoint& smooth_pose);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_MATH_COORDINATE_TRANSFORM_H_
