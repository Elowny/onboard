#include "onboard/control/control_parking/parking_path_to_trajectory.h"

#include <cmath>
#include <memory>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::control {
namespace {
TEST(ParkingPathToTrajectory, NotUpdate) {
  const VehicleModel vehicle_model = VEHICLE_MARVELR;
  TrajectoryProto trajectory;
  const auto status = ParkingPathToTrajectory(vehicle_model, &trajectory);
  EXPECT_EQ(true, status.ok());
}

TEST(ParkingPathToTrajectory, ForwardM5) {
  const VehicleModel vehicle_model = VEHICLE_QCRAFTVEHICLE_SUV;
  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(true);
  trajectory.mutable_directional_path()->set_forward(true);
  for (int i = 0; i < 10; ++i) {
    auto* point = trajectory.mutable_directional_path()->add_path();
    point->set_x(i * 1.0);
    point->set_y(0.0);
    point->set_z(0.0);
    point->set_kappa(0.1);
    point->set_lambda(0.0);
    point->set_s(i * 1.0);
    point->set_theta(-M_PI);
  }
  const auto status = ParkingPathToTrajectory(vehicle_model, &trajectory);
  for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().x(), i * 1.0,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().kappa(), 0.1,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().s(), i * 1.0,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().theta(),
                trajectory.directional_path().path().at(i).theta(), 1e-5);
  }
}

TEST(ParkingPathToTrajectory, BackwardM5) {
  const VehicleModel vehicle_model = VEHICLE_QCRAFTVEHICLE_SUV;
  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(true);
  trajectory.set_enable_stationary_steering(false);
  trajectory.mutable_directional_path()->set_forward(false);
  for (int i = 0; i < 10; ++i) {
    auto* point = trajectory.mutable_directional_path()->add_path();
    point->set_x(i * 1.0);
    point->set_y(0.0);
    point->set_z(0.0);
    point->set_kappa(0.1);
    point->set_lambda(0.0);
    point->set_s(i * 1.0);
    point->set_theta(M_PI_2);
  }
  const auto status = ParkingPathToTrajectory(vehicle_model, &trajectory);
  for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().x(), i * 1.0,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().kappa(), -0.1,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().s(), i * 1.0,
                1e-5);
  }
  EXPECT_NEAR(trajectory.trajectory_point().at(0).path_point().theta(), -M_PI_2,
              1e-5);
}

TEST(ParkingPathToTrajectory, BackwardSF5) {
  const VehicleModel vehicle_model = VEHICLE_SERES_SF5_SUV;
  TrajectoryProto trajectory;
  trajectory.set_low_speed_freespace(true);
  trajectory.set_enable_stationary_steering(false);
  trajectory.mutable_directional_path()->set_forward(false);
  for (int i = 0; i < 10; ++i) {
    auto* point = trajectory.mutable_directional_path()->add_path();
    point->set_x(i * 1.0);
    point->set_y(0.0);
    point->set_z(0.0);
    point->set_kappa(0.1);
    point->set_lambda(0.0);
    point->set_s(i * 1.0);
    point->set_theta(M_PI_2);
  }
  const auto status = ParkingPathToTrajectory(vehicle_model, &trajectory);
  for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().x(), i * 1.0,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().kappa(), -0.1,
                1e-5);
    EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().s(), i * 1.0,
                1e-5);
  }
  EXPECT_NEAR(trajectory.trajectory_point().at(0).path_point().theta(), -M_PI_2,
              1e-5);
}

}  // namespace
}  // namespace qcraft::control
