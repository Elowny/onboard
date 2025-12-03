#include "onboard/control/trajectory_interface.h"

#include <cmath>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/control/control_flags.h"
#include "onboard/global/clock.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::control {
namespace {

constexpr double kScenarioStartTime = 100;  // s.
constexpr double kPlannerInterval = 0.1;    // s.
constexpr double kTrajPointInterval = 0.1;  // s.

ApolloTrajectoryPointProto GenerateTrajectoryPoint(const Vec2d& xy, double s,
                                                   double theta, double v,
                                                   double relative_time) {
  ApolloTrajectoryPointProto apollo_traj_point;
  apollo_traj_point.mutable_path_point()->set_x(xy.x());
  apollo_traj_point.mutable_path_point()->set_y(xy.y());
  apollo_traj_point.mutable_path_point()->set_s(s);
  apollo_traj_point.mutable_path_point()->set_theta(theta);
  apollo_traj_point.set_v(v);
  apollo_traj_point.set_relative_time(relative_time);

  return apollo_traj_point;
}

TrajectoryProto GenerateTrajectory(double trajectory_start_timestamp,
                                   const Vec2d& start_point, double theta,
                                   double v) {
  TrajectoryProto trajectory_proto;
  trajectory_proto.set_trajectory_start_timestamp(trajectory_start_timestamp);

  constexpr int kNumOfTrajPoint = 30;
  constexpr int kNumOfPastPoint = 15;

  *trajectory_proto.add_trajectory_point() = GenerateTrajectoryPoint(
      start_point, /* s = */ 0.0, theta, v, /* relative_time = */ 0.0);

  for (int i = 1; i < kNumOfTrajPoint; ++i) {
    const double s = v * kTrajPointInterval * i;
    Vec2d traj_point_xy = start_point + Vec2d::UnitFromAngle(theta) * s;
    const double relative_time = i * kTrajPointInterval;

    *trajectory_proto.add_trajectory_point() =
        GenerateTrajectoryPoint(traj_point_xy, s, theta, v, relative_time);
  }

  for (int i = -kNumOfPastPoint; i < 0; ++i) {
    const double s_past = v * i * kTrajPointInterval;
    const Vec2d past_point_xy =
        start_point - Vec2d::UnitFromAngle(theta) * s_past;
    const double relative_time = i * kTrajPointInterval;

    *trajectory_proto.add_past_points() =
        GenerateTrajectoryPoint(past_point_xy, s_past, theta, v, relative_time);
  }

  trajectory_proto.set_gear(Chassis::GEAR_DRIVE);
  trajectory_proto.set_low_speed_freespace(false);
  trajectory_proto.set_aeb_triggered(false);

  return trajectory_proto;
}

TEST(TrajectoryInterfaceTest, TestReceiveStationaryTrajectory) {
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  ControllerDebugProto debug_proto;

  const Vec2d k_start_point{100, 200};
  constexpr double kTheta = M_PI;
  constexpr double kSpeed = 0;

  for (int i = 0; i < 10; ++i) {
    const TrajectoryProto trajectory_proto =
        GenerateTrajectory(kScenarioStartTime + i * kPlannerInterval,
                           k_start_point, kTheta, kSpeed);
    for (int j = 0; j < 20; ++j) {
      const auto status = trajectory_interface.Update(
          /*is_emergency_to_stop*/ false, trajectory_proto, &debug_proto);
      EXPECT_TRUE(status.ok());
      for (const auto& p : trajectory_interface.GetAllTrajPoints()) {
        const Vec2d xy{p.path_point().x(), p.path_point().y()};
        EXPECT_EQ(xy.DistanceTo(k_start_point), 0.0);
      }
      EXPECT_TRUE(trajectory_interface.GetIsStationaryTrajectory());
      EXPECT_FALSE(trajectory_interface.GetIsLowSpeedFreespace());
      EXPECT_FALSE(trajectory_interface.GetEnableStationarySteering());
      EXPECT_FALSE(trajectory_interface.aeb_triggered());
    }
  }

  EXPECT_NEAR(trajectory_interface.GetMinAccelFromTrajectory(), 0.0, 1e-5);
}

TEST(TrajectoryInterfaceTest, EmergencyToStopTest) {
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  ControllerDebugProto debug_proto;

  Vec2d start_point{100, 200};
  constexpr double kTheta = M_PI;
  constexpr double kSpeed = 10.0;

  const double prev_start_time = ToUnixDoubleSeconds(Clock::Now()) - 0.4;
  TrajectoryProto trajectory_proto =
      GenerateTrajectory(prev_start_time, start_point, kTheta, kSpeed);

  EXPECT_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, trajectory_proto, &debug_proto));
  EXPECT_NEAR(trajectory_interface.GetPlannerStartTime(), prev_start_time,
              1e-12);

  constexpr double kInterval = 0.4;
  start_point += kSpeed * kInterval * Vec2d(std::cos(kTheta), std::sin(kTheta));
  trajectory_proto = GenerateTrajectory(ToUnixDoubleSeconds(Clock::Now()),
                                        start_point, kTheta, kSpeed);
  EXPECT_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ true, trajectory_proto, &debug_proto));
  EXPECT_NEAR(trajectory_interface.GetMinAccelFromTrajectory(),
              FLAGS_control_estop_soft_brake_acceleration, 1e-12);
}

}  // namespace
}  // namespace qcraft::control
