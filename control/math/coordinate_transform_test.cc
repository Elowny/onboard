#include "onboard/control/math/coordinate_transform.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/math/util.h"
#include "onboard/proto/chassis.pb.h"

namespace qcraft {
namespace control {
namespace {

TEST(CoordinateTransform, Compute1) {
  constexpr double kEpsilon = 1e-3;
  const double length_rear_axle_to_cg = 1.0;

  const CoordinatePoint smooth_pose = {
      .x = 0.0, .y = 1.0, .yaw = NormalizeAngle(d2r(45.0))};
  const CoordinatePoint smooth_tp = {
      .x = 2.0, .y = 2.0, .yaw = NormalizeAngle(d2r(180.0))};

  const CoordinatePoint vehicle_tp =
      SmoothToVehicle(length_rear_axle_to_cg, smooth_pose, smooth_tp);
  EXPECT_NEAR(vehicle_tp.x, 1.1213, kEpsilon);
  EXPECT_NEAR(vehicle_tp.y, -0.70711, kEpsilon);
  EXPECT_NEAR(vehicle_tp.yaw, NormalizeAngle(d2r(135.0)), kEpsilon);

  const CoordinatePoint smooth_predict =
      VehicleToSmooth(length_rear_axle_to_cg, smooth_pose, vehicle_tp);
  EXPECT_NEAR(smooth_predict.x, smooth_tp.x, kEpsilon);
  EXPECT_NEAR(smooth_predict.y, smooth_tp.y, kEpsilon);
  EXPECT_NEAR(smooth_predict.yaw, smooth_tp.yaw, kEpsilon);
}

TEST(CoordinateTransform, Compute2) {
  constexpr double kEpsilon = 1e-3;
  const double length_rear_axle_to_cg = 1.0;

  const CoordinatePoint smooth_pose = {
      .x = 250.0, .y = 100.0, .yaw = NormalizeAngle(d2r(179.0))};
  const CoordinatePoint smooth_tp = {
      .x = 270.0, .y = 100.0, .yaw = NormalizeAngle(d2r(-179.0))};

  const CoordinatePoint vehicle_tp =
      SmoothToVehicle(length_rear_axle_to_cg, smooth_pose, smooth_tp);
  EXPECT_NEAR(vehicle_tp.x, -20.997, kEpsilon);
  EXPECT_NEAR(vehicle_tp.y, -0.34905, kEpsilon);
  EXPECT_NEAR(vehicle_tp.yaw, NormalizeAngle(d2r(2.0)), kEpsilon);

  const CoordinatePoint smooth_predict =
      VehicleToSmooth(length_rear_axle_to_cg, smooth_pose, vehicle_tp);
  EXPECT_NEAR(smooth_predict.x, smooth_tp.x, kEpsilon);
  EXPECT_NEAR(smooth_predict.y, smooth_tp.y, kEpsilon);
  EXPECT_NEAR(smooth_predict.yaw, smooth_tp.yaw, kEpsilon);
}

TEST(TrajectoryTransform, Compute3) {
  constexpr double kEpsilon = 1e-3;
  const double length_rear_axle_to_cg = 1.0;

  const CoordinatePoint smooth_pose = {
      .x = 0.0, .y = 1.0, .yaw = NormalizeAngle(d2r(45.0))};

  TrajectoryProto smooth_trajectory;
  smooth_trajectory.set_gear(Chassis::GEAR_DRIVE);
  for (int i = 0; i < 20; ++i) {
    const auto smooth_tp = smooth_trajectory.mutable_trajectory_point()->Add();
    smooth_tp->mutable_path_point()->set_x(2.0);
    smooth_tp->mutable_path_point()->set_y(2.0);
    smooth_tp->mutable_path_point()->set_theta(NormalizeAngle(d2r(180.0)));
    smooth_tp->mutable_path_point()->set_kappa(0.01 * i);
    smooth_tp->mutable_path_point()->set_s(0.1 * i);
  }

  const auto vehicle_trajectory = TrajectorySmoothToVehicle(
      length_rear_axle_to_cg, smooth_pose, smooth_trajectory);

  // Check trajectory.
  EXPECT_EQ(vehicle_trajectory.gear(), smooth_trajectory.gear());
  for (int i = 0; i < vehicle_trajectory.trajectory_point_size(); ++i) {
    EXPECT_NEAR(vehicle_trajectory.trajectory_point().Get(i).path_point().x(),
                1.1213, kEpsilon);
    EXPECT_NEAR(vehicle_trajectory.trajectory_point().Get(i).path_point().y(),
                -0.70711, kEpsilon);
    EXPECT_NEAR(
        vehicle_trajectory.trajectory_point().Get(i).path_point().theta(),
        d2r(135.0), kEpsilon);
    EXPECT_NEAR(
        vehicle_trajectory.trajectory_point().Get(i).path_point().kappa(),
        smooth_trajectory.trajectory_point().Get(i).path_point().kappa(),
        kEpsilon);
  }
}

}  // namespace
}  // namespace control
}  // namespace qcraft
