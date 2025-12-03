#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

// IWYU pragma: no_include "onboard/global/buffered_logger.h"
#include "benchmark/benchmark.h"

#include "onboard/control/benchmark/control_test_config_setup.h"
#include "onboard/control/controllers/lat_km_mpc_controller.h"
#include "onboard/control/controllers/lon_mpc_controller.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/steering_protection.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/lite/check.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::control {
namespace {

void BM_TrajectoryInterfaceUpdate(benchmark::State& state) {  // NOLINT
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  ControllerDebugProto debug_proto;
  const TrajectoryProto trajectory = BuildTrajectoryProtoForTest();

  for (auto _ : state) {
    benchmark::DoNotOptimize(trajectory_interface.Update(
        /*is_emergency_to_stop*/ false, trajectory, &debug_proto));
  }
}

VehicleStateProto BuildVehicleStateProto(
    const AutonomyStateProto_State& autonomy_state, const PoseProto& pose,
    const Chassis& chassis, const SteeringConverter& steering_converter,
    ControllerDebugProto* debug_proto) {
  VehicleStateInterface vehicle_state_interface;
  QCHECK_OK(vehicle_state_interface.Update(
      /*yaw_bias*/ 0.0, autonomy_state, pose, chassis, std::nullopt,
      steering_converter, debug_proto));

  return vehicle_state_interface.Result();
}

LonControllerOutputProto BuildLonControllerOutputProto() {
  LonControllerOutputProto lon_controller_output_proto;
  ControllerDebugProto controller_debug_proto =
      BuildControllerDebugProtoForTest();
  for (int i = 0;
       i < controller_debug_proto.mpc_debug_proto().t_control_mpc_result_size();
       ++i) {
    lon_controller_output_proto.add_t_control_acc_vec(
        controller_debug_proto.mpc_debug_proto().t_control_mpc_result()[i]);
  }
  lon_controller_output_proto.set_is_standstill(false);

  return lon_controller_output_proto;
}

void BM_LatKmMpcControllerComputeControlCommand(
    benchmark::State& state) {  // NOLINT
  ControlCommand cmd;
  ControllerDebugProto debug_proto;

  // Init LatKmMpcController.
  ControllerParams controller_params = BuildControllerParams("Q1001");
  SteeringConverter steering_converter(controller_params.vehicle_geo_params,
                                       controller_params.vehicle_drive_params);
  LatKmMpcController lat_km_mpc_controller(&controller_params.controller_conf,
                                           &steering_converter);

  // Prepare controller input.
  ControlCommand prev_cmd = BuildControlCommandForTest();
  VehicleStateProto vehicle_state = BuildVehicleStateProto(
      BuildAutonomyStateProtoForTest().autonomy_state(), BuildPoseForTest(),
      BuildChassisForTest(), steering_converter, &debug_proto);
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  QCHECK_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, BuildTrajectoryProtoForTest(),
      &debug_proto));

  SteeringProtection steering_protection(controller_params.vehicle_drive_params,
                                         &steering_converter,
                                         &controller_params.controller_conf);
  const SteeringProtectionResult steering_protection_result =
      steering_protection.CalcKappaAndKappaRateLimit(prev_cmd.curvature(),
                                                     vehicle_state);

  // Predict pose after steer delay
  const VehPose predicted_pose_after_delay(
      /*vehicle_geometry_params*/ nullptr, vehicle_state);
  // Adaptive for having steer delay or having no steer delay.
  const LonControllerOutputProto lon_controller_output =
      BuildLonControllerOutputProto();

  double max_kappa_rate = std::numeric_limits<double>::quiet_NaN();
  double min_kappa_rate = std::numeric_limits<double>::quiet_NaN();
  for (auto _ : state) {
    lat_km_mpc_controller.Reset(vehicle_state);
    benchmark::DoNotOptimize(lat_km_mpc_controller.ComputeControlCommand(
        vehicle_state, trajectory_interface, steering_protection_result,
        predicted_pose_after_delay, lon_controller_output, &cmd, &debug_proto));
    if (std::isnan(max_kappa_rate) || std::isnan(min_kappa_rate)) {
      max_kappa_rate = cmd.steer_speed_target();
      min_kappa_rate = cmd.steer_speed_target();
    }
    max_kappa_rate = std::max(max_kappa_rate, cmd.steer_speed_target());
    min_kappa_rate = std::min(min_kappa_rate, cmd.steer_speed_target());
  }
  QCHECK_LT(std::abs(max_kappa_rate - min_kappa_rate), 1e-10);
}

void BM_LonMpcControllerComputeControlCommand(
    benchmark::State& state) {  // NOLINT
  ControlCommand cmd;
  ControllerDebugProto debug_proto;
  LonControllerOutputProto lon_controller_output;

  // Init LonMpcController.
  ControllerParams controller_params = BuildControllerParams("Q1001");
  SteeringConverter steering_converter(controller_params.vehicle_geo_params,
                                       controller_params.vehicle_drive_params);
  LonMpcController lon_mpc_controller(&controller_params.controller_conf,
                                      &controller_params.vehicle_drive_params);

  // Prepare controller input.
  ControlCommand prev_cmd = BuildControlCommandForTest();
  VehicleStateProto vehicle_state = BuildVehicleStateProto(
      BuildAutonomyStateProtoForTest().autonomy_state(), BuildPoseForTest(),
      BuildChassisForTest(), steering_converter, &debug_proto);
  TrajectoryInterface trajectory_interface(VEHICLE_MARVELR);
  QCHECK_OK(trajectory_interface.Update(
      /*is_emergency_to_stop*/ false, BuildTrajectoryProtoForTest(),
      &debug_proto));
  // Predict pose after steer delay
  const VehPose predicted_pose_after_delay(
      /*vehicle_geometry_params*/ nullptr, vehicle_state);

  for (auto _ : state) {
    lon_mpc_controller.Reset(vehicle_state);
    benchmark::DoNotOptimize(lon_mpc_controller.ComputeControlCommand(
        trajectory_interface, predicted_pose_after_delay, &cmd, &debug_proto,
        &lon_controller_output));
  }
}

BENCHMARK(BM_TrajectoryInterfaceUpdate)->Unit(benchmark::kMicrosecond)->Arg(5);
BENCHMARK(BM_LatKmMpcControllerComputeControlCommand)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(5);
BENCHMARK(BM_LonMpcControllerComputeControlCommand)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(5);

}  // namespace
}  // namespace qcraft::control

BENCHMARK_MAIN();
