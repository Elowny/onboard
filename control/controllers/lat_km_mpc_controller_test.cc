#include "onboard/control/controllers/lat_km_mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/control/steering_protection.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {
namespace {

struct ControllerParams {
  ControllerConf controller_conf;
  VehicleGeometryParamsProto vehicle_geo_params;
  VehicleDriveParamsProto vehicle_drive_params;
};

ControllerParams BuildControllerParams() {
  ControllerParams controller_params;
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  controller_params.vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  controller_params.vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  controller_params.controller_conf =
      run_params.vehicle_params().controller_conf();

  return controller_params;
}

PathPoint BuildPathPoint(const Vec2d& xy, double s, double theta,
                         double kappa) {
  PathPoint path_point;
  path_point.set_x(xy.x());
  path_point.set_y(xy.y());
  path_point.set_theta(theta);
  path_point.set_s(s);
  path_point.set_kappa(kappa);

  return path_point;
}

ApolloTrajectoryPointProto BuildrajectoryPoint(const PathPoint& path_point,
                                               double v, double relative_time) {
  ApolloTrajectoryPointProto traj_point;
  traj_point.mutable_path_point()->CopyFrom(path_point);
  traj_point.set_v(v);
  traj_point.set_relative_time(relative_time);
  return traj_point;
}

TrajectoryProto BuildTrajectoryProto(const Vec2d& xy, double theta, double v,
                                     double kappa) {
  // Build a constant v and kappa trajectory.
  TrajectoryProto trajectory;
  trajectory.set_trajectory_start_timestamp(0.0);

  *trajectory.add_trajectory_point() = BuildrajectoryPoint(
      BuildPathPoint(xy, /*s*/ 0.0, theta, kappa), v, /*relative_time*/ 0.0);

  Vec2d xy_tmp = xy;
  double theta_tmp = theta;
  constexpr double kDt = 0.1;  // s.
  for (int i = 1; i < /*trajectory_point_size*/ 30; ++i) {
    const double s = v * kDt * i;
    const double delta_theta = v * kappa * kDt;
    xy_tmp += v * kDt * Vec2d::UnitFromAngle(theta_tmp + 0.5 * delta_theta);
    theta_tmp += delta_theta;

    *trajectory.add_trajectory_point() =
        BuildrajectoryPoint(BuildPathPoint(xy_tmp, s, theta_tmp, kappa), v,
                            /*relative_time*/ i * kDt);
  }

  xy_tmp = xy;
  theta_tmp = theta;
  for (int i = 0; i < /*past_point_size*/ 5; ++i) {
    const double s = -v * kDt * i;
    const double delta_theta = -v * kappa * kDt;
    xy_tmp -= v * kDt * Vec2d::UnitFromAngle(theta_tmp + 0.5 * delta_theta);
    theta_tmp -= delta_theta;

    *trajectory.add_past_points() =
        BuildrajectoryPoint(BuildPathPoint(xy_tmp, s, theta_tmp, kappa), v,
                            /*relative_time*/ -i * kDt);
  }

  trajectory.set_gear(Chassis_GearPosition_GEAR_DRIVE);

  return trajectory;
}

VehicleStateProto BuildVehicleState(const Vec2d& xy, double theta, double v,
                                    double kappa) {
  VehicleStateProto vehicle_state;
  vehicle_state.set_x(xy.x());
  vehicle_state.set_y(xy.y());
  vehicle_state.set_yaw(theta);
  vehicle_state.set_linear_velocity(v);
  vehicle_state.set_kappa(kappa);

  vehicle_state.set_timestamp(0.0);
  vehicle_state.set_gear(Chassis_GearPosition_GEAR_DRIVE);
  vehicle_state.set_is_auto_mode(true);

  return vehicle_state;
}

LonControllerOutputProto BuildLonControllerOutputProto() {
  LonControllerOutputProto lon_controller_output;
  lon_controller_output.set_is_standstill(false);
  for (int i = 0; i < 10; ++i) {
    lon_controller_output.add_t_control_acc_vec(0.0);
  }

  return lon_controller_output;
}

TEST(LatKmMpcControllerTest, StraightLaneTest) {
  ControlCommand cmd;
  ControllerDebugProto debug_proto;

  // Vehicle init pose setting;
  const Vec2d av_xy(100, 100);
  const double av_v = 10.0;
  const double av_kappa = 0.0;
  const double av_theta = 0.5;

  // Init LatKmMpcController.
  ControllerParams controller_params = BuildControllerParams();
  SteeringConverter steering_converter(controller_params.vehicle_geo_params,
                                       controller_params.vehicle_drive_params);

  LatKmMpcController lat_km_mpc_controller(&controller_params.controller_conf,
                                           &steering_converter);

  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  QCHECK_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ false,
      BuildTrajectoryProto(av_xy, av_theta, av_v, av_kappa), &debug_proto));

  VehicleStateProto vehicle_state =
      BuildVehicleState(av_xy, av_theta, av_v, av_kappa);

  SteeringProtection steering_protection(controller_params.vehicle_drive_params,
                                         &steering_converter,
                                         &controller_params.controller_conf);
  const SteeringProtectionResult steering_protection_result =
      steering_protection.CalcKappaAndKappaRateLimit(av_kappa, vehicle_state);

  const VehPose init_lon_pose(&controller_params.vehicle_geo_params,
                              vehicle_state);

  const LonControllerOutputProto lon_controller_output =
      BuildLonControllerOutputProto();

  double max_kappa_rate = std::numeric_limits<double>::quiet_NaN();
  double min_kappa_rate = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 100; ++i) {
    lat_km_mpc_controller.Reset(vehicle_state);
    QCHECK_OK(lat_km_mpc_controller.ComputeControlCommand(
        vehicle_state, trajectory_interface, steering_protection_result,
        init_lon_pose, lon_controller_output, &cmd, &debug_proto));
    if (std::isnan(max_kappa_rate) || std::isnan(min_kappa_rate)) {
      max_kappa_rate = cmd.steer_speed_target();
      min_kappa_rate = cmd.steer_speed_target();
    }
    max_kappa_rate = std::max(max_kappa_rate, cmd.steer_speed_target());
    min_kappa_rate = std::min(min_kappa_rate, cmd.steer_speed_target());
  }

  EXPECT_NEAR(max_kappa_rate, 0.0, 1e-10);
  EXPECT_NEAR(min_kappa_rate, 0.0, 1e-10);
  EXPECT_LT(std::abs(max_kappa_rate - min_kappa_rate), 1e-7);
}

TEST(LatKmMpcControllerTest, FullTurnTest) {
  ControlCommand cmd;
  ControllerDebugProto debug_proto;

  // Vehicle init pose setting;
  const Vec2d av_xy(100, 100);
  const double av_v = 10.0;
  const double av_kappa = 0.2;
  const double av_theta = 0.5;

  // Init LatKmMpcController.
  ControllerParams controller_params = BuildControllerParams();
  SteeringConverter steering_converter(controller_params.vehicle_geo_params,
                                       controller_params.vehicle_drive_params);

  LatKmMpcController lat_km_mpc_controller(&controller_params.controller_conf,
                                           &steering_converter);

  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  ASSERT_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ false,
      BuildTrajectoryProto(av_xy, av_theta, av_v, av_kappa), &debug_proto));

  const VehicleStateProto vehicle_state =
      BuildVehicleState(av_xy, av_theta, av_v, av_kappa);

  const SteeringProtection steering_protection(
      controller_params.vehicle_drive_params, &steering_converter,
      &controller_params.controller_conf);
  const SteeringProtectionResult steering_protection_result =
      steering_protection.CalcKappaAndKappaRateLimit(av_kappa, vehicle_state);

  const LonControllerOutputProto lon_controller_output =
      BuildLonControllerOutputProto();

  double max_kappa_rate = std::numeric_limits<double>::quiet_NaN();
  double min_kappa_rate = std::numeric_limits<double>::quiet_NaN();

  const VehPose init_lon_pose(&controller_params.vehicle_geo_params,
                              vehicle_state);

  for (int i = 0; i < 100; ++i) {
    lat_km_mpc_controller.Reset(vehicle_state);
    ASSERT_OK(lat_km_mpc_controller.ComputeControlCommand(
        vehicle_state, trajectory_interface, steering_protection_result,
        init_lon_pose, lon_controller_output, &cmd, &debug_proto));

    if (std::isnan(max_kappa_rate) || std::isnan(min_kappa_rate)) {
      max_kappa_rate = cmd.steer_speed_target();
      min_kappa_rate = cmd.steer_speed_target();
    }

    max_kappa_rate = std::max(max_kappa_rate, cmd.steer_speed_target());
    min_kappa_rate = std::min(min_kappa_rate, cmd.steer_speed_target());
  }
  EXPECT_NEAR(max_kappa_rate, min_kappa_rate, 1e-7);
}

}  // namespace
}  // namespace control
}  // namespace qcraft
