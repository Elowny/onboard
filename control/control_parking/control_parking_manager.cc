#include "onboard/control/control_parking/control_parking_manager.h"

#include <algorithm>
#include <utility>

#include "onboard/control/control_flags.h"
#include "onboard/control/controllers/controller_common.h"
#include "onboard/proto/chassis.pb.h"

namespace qcraft {
namespace control {
// Compute speed cmd of marvelr.
double ComputeMarvelrSpeedCmd(Chassis::GearPosition gear_fb, bool is_full_stop,
                              double speed_planner, double station_error) {
  if (is_full_stop) return 0.0;
  double speed_cmd =
      speed_planner + FLAGS_control_apa_speed_gain * (-station_error);
  constexpr double kSpeedCutoff = 0.2;  // Prevent slipping |speed|<kSpeedCutoff
  if (gear_fb == Chassis::GEAR_REVERSE) {
    speed_cmd = std::min(-kSpeedCutoff, speed_cmd);
  } else {
    speed_cmd = std::max(kSpeedCutoff, speed_cmd);
  }
  constexpr double kMaxSpeed = 1.66;  // 6km/h
  return std::clamp(speed_cmd, -kMaxSpeed, kMaxSpeed);
}

// Compute speed and distance cmd of M5
std::pair<double, double> ComputeM5SpeedDistanceCmd(bool is_full_stop,
                                                    double stop_s) {
  if (is_full_stop) return {0.0, 0.0};

  // If not fullstop, need to maintain min speed and distance cmd.
  const double apa_distance_cmd = stop_s + FLAGS_control_apa_min_distance;

  // If real distance to stop > 4m, by 1m/s speed drive.
  constexpr double kHighDriveSpeed = 1.0;  // m/s
  constexpr double kHighDistance = 4.0;    // m
  if (stop_s > kHighDistance) return {kHighDriveSpeed, apa_distance_cmd};

  // If 1m < real distance to stop < 4m, by 0.5m/s speed drive.
  constexpr double kLowDriveSpeed = 0.5;  // m/s
  constexpr double kLowDistance = 1.0;    // m
  if (stop_s > kLowDistance) return {kLowDriveSpeed, apa_distance_cmd};

  // If 0m < real distance to stop < 1m, by plf speed drive.
  const double speed_cmd =
      stop_s * kLowDriveSpeed / std::max(kLowDistance, 0.1);
  return {std::min(speed_cmd, kLowDriveSpeed), apa_distance_cmd};
}

ParkingManager::ParkingManager(const VehicleModel& vehicle_model)
    : vehicle_model_(vehicle_model) {}

ParkingState ParkingManager::ParkingProcess(const ParkingManagerInput& input,
                                            ControlCommand* command,
                                            ControllerDebugProto* debug) {
  const bool is_freesapce =
      input.trajectory_interface->GetIsLowSpeedFreespace();
  command->mutable_custom_command()->set_low_speed_freespace(is_freesapce);
  if (!is_freesapce)
    return ParkingState{/*reset_lon_controller*/ false,
                        /*reset_lat_controller*/ false};
  // Entry freespace mode.
  const double av_speed = input.vehicle_state->linear_velocity();

  // 1. If or not fullstop.
  const double stop_s = input.trajectory_interface->GetParkingStopS();
  const bool is_fullstop = IsFullStop(stop_s, av_speed,
                                      /*is_freesapce*/ true);
  debug->mutable_parking_debug()->set_is_fullstop(is_fullstop);
  debug->mutable_parking_debug()->set_stop_s(stop_s);

  // 2. If or not standstill (simple).
  const bool is_standstill = IsStandstill(av_speed);
  debug->mutable_parking_debug()->set_is_standstill(is_standstill);

  // 3. If or not stationary_steering, M5 vehicle: one point.
  const bool is_stationary_steering =
      input.trajectory_interface->GetEnableStationarySteering() &&
      is_standstill && is_fullstop && input.vehicle_state->is_auto_mode();
  debug->mutable_parking_debug()->set_is_stationary_steering(
      is_stationary_steering);

  // 3.1. If stationary_steering, compute cmd and debug.
  const StationarySteerControlInput stationary_steer_input = {
      .is_stationary_steer = is_stationary_steering,
      .kappa_cmd = command->curvature(),
      .kappa_trajectory =
          input.trajectory_interface->GetStationarySteerRefKappa(),
      .vehicle_state = input.vehicle_state,
      .steering_protection_result = input.steering_protection_result,
      .steering_converter = input.steering_converter};
  const double kappa_cmd_stationary =
      stationary_steer_control_.ComputestationarySteerCmd(
          stationary_steer_input);
  debug->mutable_parking_debug()->set_stationary_kappa_target(
      stationary_steer_input.kappa_trajectory);

  // 3.2. If is_stationary_steering, reset lat controller.
  bool reset_lat_controller = false;
  if (is_stationary_steering) {
    command->set_curvature(kappa_cmd_stationary);
    reset_lat_controller = true;
  }
  debug->mutable_parking_debug()->set_is_reset_lat_controller(
      reset_lat_controller);

  // 4. If M5 parking and is onboard (not reset when sim), reset lon controller.
  bool reset_lon_controller = false;
  if (input.is_onboard && (vehicle_model_ == VEHICLE_QCRAFTVEHICLE_SUV ||
                           vehicle_model_ == VEHICLE_SERES_SF5_SUV))
    reset_lon_controller = true;
  debug->mutable_parking_debug()->set_is_reset_lon_controller(
      reset_lat_controller);

  // 5. Compute parking speed and distance cmd.
  switch (vehicle_model_) {
    case VEHICLE_MARVELR:
    case VEHICLE_MARVELR_NEW: {
      const double speed_cmd = ComputeMarvelrSpeedCmd(
          input.vehicle_state->gear(), is_fullstop,
          command->debug().simple_mpc_debug().speed_reference(),
          command->debug().control_error().station_error());
      command->set_speed(speed_cmd);
      break;
    }
    case VEHICLE_SERES_SF5_SUV:
    case VEHICLE_QCRAFTVEHICLE_SUV: {
      const auto speed_distance_cmd =
          ComputeM5SpeedDistanceCmd(is_fullstop, stop_s);
      command->set_speed(speed_distance_cmd.first);
      command->set_parking_distance(speed_distance_cmd.second);
      command->set_acceleration(0.0);

      // Compute virtual acc for simulation
      if (!input.is_onboard) {
        const double speed_error =
            input.trajectory_interface->GetIsParkingForward()
                ? (command->speed() - av_speed)
                : (-command->speed() - av_speed);
        const double distance_error =
            command->debug().control_error().station_error();

        constexpr double kParkingSimGain = 2.0;
        const double virtual_acc =
            kParkingSimGain * speed_error + kParkingSimGain * (-distance_error);
        constexpr double kParkingMaxAcc = 1.5;  // m/s2
        command->set_acceleration(
            std::clamp(virtual_acc, -kParkingMaxAcc, kParkingMaxAcc));
        command->mutable_debug()->mutable_control_error()->set_speed_error(
            speed_error);
        command->mutable_debug()->mutable_control_error()->set_station_error(
            distance_error);
      }
      break;
    }
    default:
      break;
  }

  return ParkingState{reset_lon_controller, reset_lat_controller};
}

}  // namespace control
}  // namespace qcraft
