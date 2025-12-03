#include "onboard/control/controllers/controller_common.h"

#include <algorithm>
#include <cmath>

#include "absl/status/status.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"

namespace qcraft {
namespace control {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kTrajPointInterval = 0.1;  // s.

PathPoint BuildPathPoint(const Vec2d& xy, double theta, double s) {
  PathPoint path_point;
  path_point.set_x(xy.x());
  path_point.set_y(xy.y());
  path_point.set_theta(theta);
  path_point.set_s(s);

  return path_point;
}

ApolloTrajectoryPointProto GenerateTrajectoryPoint(const Vec2d& xy, double s,
                                                   double theta, double v,
                                                   double relative_time) {
  ApolloTrajectoryPointProto apollo_traj_point;
  apollo_traj_point.mutable_path_point()->CopyFrom(
      BuildPathPoint(xy, theta, s));
  apollo_traj_point.set_v(v);
  apollo_traj_point.set_relative_time(relative_time);

  return apollo_traj_point;
}

TEST(ControllerCommonTest, LatControlErrorTest) {
  PathPoint ref_point =
      BuildPathPoint(Vec2d(0.0, 0.0), /*theta*/ M_PI_4, /*s*/ 0.0);

  Vec2d xy(1.0, 1.0);
  for (int i = 0; i < 10; ++i) {
    xy += Vec2d(0.2, 0.2);
    EXPECT_NEAR(
        CalculateLatControlError(xy, /*yaw*/ 0.0, ref_point).lateral_error, 0.0,
        kEpsilon);
  }

  xy = Vec2d(1.0, 0.0);
  for (int i = 0; i < 10; ++i) {
    xy += Vec2d(0.2, 0.2);
    EXPECT_NEAR(
        CalculateLatControlError(xy, /*yaw*/ 0.0, ref_point).lateral_error,
        -0.5 * sqrt(2), kEpsilon);
  }
}

TEST(ControllerCommonTest, LonControlErrorTest) {
  ApolloTrajectoryPointProto ref_point =
      GenerateTrajectoryPoint(Vec2d(0.0, 0.0), /*s*/ 0.0, /*theta*/ M_PI_4,
                              /*v*/ 2.0, /*relative_time*/ 1.0);
  Vec2d xy(0.0, 0.0);
  for (int i = 0; i < 10; ++i) {
    xy += Vec2d(-0.2, 0.2);
    EXPECT_NEAR(CalculateLonControlError(xy, /*av_speed*/ 0.0,
                                         /*acc_error*/ 0.0, ref_point)
                    .station_error,
                0.0, kEpsilon);
  }

  xy = Vec2d(1.0, 0.0);
  for (int i = 0; i < 10; ++i) {
    xy += Vec2d(-0.2, 0.2);
    EXPECT_NEAR(CalculateLonControlError(xy, /*av_speed*/ 0.0,
                                         /*acc_error*/ 0.0, ref_point)
                    .station_error,
                0.5 * sqrt(2), kEpsilon);
  }
}

VehicleStateProto BuildVehicleStateProto(const Vec2d& xy, double time_step) {
  VehicleStateProto vehicle_state;
  vehicle_state.set_x(xy.x());
  vehicle_state.set_y(xy.y());
  vehicle_state.set_timestamp(time_step);
  return vehicle_state;
}

TrajectoryProto GenerateTrajectory(double trajectory_start_timestamp,
                                   const Vec2d& start_point, double theta,
                                   double v) {
  TrajectoryProto trajectory_proto;
  trajectory_proto.set_trajectory_start_timestamp(trajectory_start_timestamp);

  constexpr int kNumOfTrajPoint = 10;
  constexpr int kNumOfPastPoint = 5;

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

TEST(ControllerCommonTest, ControlErrorTest) {
  TrajectoryProto trajectory =
      GenerateTrajectory(0.0, Vec2d(0.0, 0.0), /*theta*/ 0.0, /*v*/ 1.0);
  constexpr double kExpectLonErr = 1.0;  // m.
  constexpr double kExpectLatErr = 0.5;  // m.

  VehicleStateProto vehicle_state =
      BuildVehicleStateProto(Vec2d(kExpectLonErr, kExpectLatErr), 0.0);
  ControllerDebugProto controller_debug_proto;
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  EXPECT_OK(
      trajectory_interface.Update(false, trajectory, &controller_debug_proto));
  ControlCacheManager control_cache;

  ControlError control_error = CalculateControlError(
      /*enable_yaw_consider_slip*/ false, vehicle_state, trajectory_interface,
      trajectory_interface, 0.0, 0.0, 0.0);
  EXPECT_EQ(control_error.lateral_error(), kExpectLatErr);
  EXPECT_EQ(control_error.station_error(), kExpectLonErr);
}

TEST(ControllerCommonTest, ComputeStepLengthFromTControlTest) {
  std::vector<double> speed_vec;
  speed_vec.reserve(kSControlHorizon);
  for (int i = 0; i < kSControlHorizon; ++i) {
    speed_vec.push_back(1.0 * i);
  }
  constexpr double kMpcPeriod = 0.1;
  const auto t_control_s_vec =
      ComputeStepLengthFromTControl(speed_vec, kMpcPeriod);

  std::vector<double> s_vec_gt;
  s_vec_gt.push_back(0.5 * kMpcPeriod);
  for (int i = 1; i < kSControlHorizon - 1; ++i) {
    s_vec_gt.push_back((1.0 * i + 0.5) * kMpcPeriod + s_vec_gt[i - 1]);
  }
  s_vec_gt.push_back(9.0 * kMpcPeriod + s_vec_gt[kSControlHorizon - 2]);

  EXPECT_EQ(t_control_s_vec.size(), kSControlHorizon);
  EXPECT_EQ(t_control_s_vec.size(), s_vec_gt.size());
  for (int i = 0; i < kSControlHorizon; ++i) {
    EXPECT_EQ(t_control_s_vec[i], s_vec_gt[i]) << "i: " << i;
  }
}

TEST(ControllerCommonTest, IsFullstop) {
  bool is_fullstop = IsFullStop(/*trajectory_accumulate_s*/ 1.21,
                                /*av_speed*/ 0.0,
                                /*is_freesapce*/ false);
  EXPECT_EQ(is_fullstop, false);
  is_fullstop = IsFullStop(/*trajectory_accumulate_s*/ 1.0,
                           /*av_speed*/ 0.0,
                           /*is_freesapce*/ false);
  EXPECT_EQ(is_fullstop, true);
  is_fullstop = IsFullStop(/*trajectory_accumulate_s*/ 0.2,
                           /*av_speed*/ 0.0,
                           /*is_freesapce*/ true);
  EXPECT_EQ(is_fullstop, false);
  is_fullstop = IsFullStop(/*trajectory_accumulate_s*/ 0.05,
                           /*av_speed*/ 0.1,
                           /*is_freesapce*/ true);
  EXPECT_EQ(is_fullstop, true);
}

TEST(ControllerCommonTest, IsStandstill) {
  const bool is_standstill = IsStandstill(/*av_speed*/ 0.1);
  EXPECT_EQ(is_standstill, false);
}

}  // namespace
}  // namespace control
}  // namespace qcraft
