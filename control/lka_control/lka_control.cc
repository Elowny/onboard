#include "onboard/control/lka_control/lka_control.h"

#include <cmath>
#include <optional>
#include <ostream>
#include <vector>

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/controllers/controller_common.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"  // for VehPose
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"  // for Vec3dProto
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/autonomy_state.pb.h"  // for AutonomyStateProto
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace control {

LKAController::LKAController(
    const ControllerConf* control_conf,
    const VehicleGeometryParamsProto* vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleModel& vehicle_model)
    : geo_param_(QCHECK_NOTNULL(vehicle_geometry_params)) {
  // Steering_converter init
  steering_converter_ptr_ = std::make_unique<const SteeringConverter>(
      *vehicle_geometry_params, vehicle_drive_params);

  // Trajectory interface init
  trajectory_interface_ptr_ =
      std::make_unique<TrajectoryInterface>(vehicle_model);

  // Steering protection init
  steering_protection_ptr_ = std::make_unique<SteeringProtection>(
      vehicle_drive_params, steering_converter_ptr_.get(), control_conf);

  // Lat mpc controller init
  lat_km_mpc_controller_ptr_ = std::make_unique<LatKmMpcController>(
      control_conf, steering_converter_ptr_.get());
  QLOG(INFO) << " Lat mpc controller Init() success.";

  // Torque controller init
  torque_controller_ptr_ =
      std::make_unique<TorqueController>(control_conf->steer_torque_conf());
  QLOG(INFO) << " Torque controller Init() success.";

  // Steer delay and predict pose.
  steer_delay_steps_ =
      RoundToInt(FLAGS_lss_control_predict_time * kControlFrequency);

  QLOG(INFO) << " LKA Controller Init() success.";
}

absl::Status LKAController::ComputeControlCommand(
    const TrajectoryProto& trajectory, const PoseProto& pose,
    const Chassis& chassis, ControlCommand* control_command,
    ControllerDebugProto* controller_debug_proto) {
  if (chassis.gear_location() != Chassis::GEAR_DRIVE) {
    Reset();
    return absl::InternalError("chassis gear is not D !");
  }

  // Mpc input: 1. Vehicle state interface.
  constexpr double kYawBias = 0.0;
  AutonomyStateProto autonomy_state;
  VehicleStateInterface vehicle_state_interface;
  const auto status_state = vehicle_state_interface.Update(
      kYawBias, autonomy_state.autonomy_state(), pose, chassis, std::nullopt,
      *steering_converter_ptr_, controller_debug_proto);
  if (!status_state.ok()) {
    Reset();
    return absl::InternalError("Update vehicle state failed !");
  }
  vehicle_state_ = vehicle_state_interface.Result();
  vehicle_state_.set_is_auto_mode(true);

  // Mpc input: 2. Lon controller output.
  const auto lon_control_output =
      CreateLonControllerOutput(vehicle_state_.linear_acceleration());

  // Mpc input: 3.Predict Pose
  const VehPose predicted_pose_after_delay = PredictControlInitPoseByKM(
      VehPose(geo_param_, vehicle_state_), *steering_converter_ptr_,
      std::vector<double>(steer_delay_steps_,
                          /*kappa_target*/ previous_kappa_cmd_),
      std::vector<double>(steer_delay_steps_,
                          /*acc_target*/ vehicle_state_.linear_acceleration()));

  // Mpc input: 4.ControlConstraint, simple update.
  if (is_first_run_) {
    is_first_run_ = false;
    previous_kappa_cmd_ = steering_converter_ptr_->SteerPctToKappa(
        vehicle_state_.chassis_steering_percentage());
    lat_km_mpc_controller_ptr_->Reset(vehicle_state_);
  }
  const auto steering_protection_result =
      steering_protection_ptr_->CalcKappaAndKappaRateLimit(previous_kappa_cmd_,
                                                           vehicle_state_);

  // Mpc input: 5.Trajectory interface
  const auto status_trajectory = trajectory_interface_ptr_->Update(
      autonomy_state.autonomy_state() == AutonomyStateProto::EMERGENCY_TO_STOP,
      trajectory, controller_debug_proto);
  if (!status_trajectory.ok()) {
    Reset();
    return absl::InternalError("Update trajectory interface failed !");
  }

  // Mpc controller compute.
  const auto status_mpc = lat_km_mpc_controller_ptr_->ComputeControlCommand(
      vehicle_state_, *trajectory_interface_ptr_, steering_protection_result,
      predicted_pose_after_delay, lon_control_output, control_command,
      controller_debug_proto);
  if (!status_mpc.ok()) {
    Reset();
    return absl::InternalError("Mpc controller compute failed !");
  }
  const double steering_target_pct =
      steering_converter_ptr_->KappaToSteerPct(control_command->curvature());
  control_command->set_steering_target(steering_target_pct);
  control_command->set_steer_angle_bias(0.0);

  // Torque controller compute.
  const TorqueControllerInput torque_input = {
      .is_auto_steer = true,
      .is_lka = true,
      .steer_angle_target_past = 0.0,
      .vehicle_state = &vehicle_state_,
      .control_cmd = control_command,
      .steering_converter = steering_converter_ptr_.get()};
  const double torque_cmd = torque_controller_ptr_->ComputeSteerTorqueTarget(
      torque_input, controller_debug_proto);
  control_command->set_torque_target(torque_cmd);
  control_command->set_steer_mode(SteerMode::TORQUE_MODE);

  // Compute control error.
  const auto closest_path_point =
      trajectory_interface_ptr_
          ->QueryNearestTrajPointByXY({vehicle_state_.x(), vehicle_state_.y()})
          .path_point();
  const LatControlError control_error =
      CalculateLatControlError(Vec2d(vehicle_state_.x(), vehicle_state_.y()),
                               vehicle_state_.yaw(), closest_path_point);

  constexpr double kContLatAccThreshold = 2.5;  // m/s^2
  if (!FLAGS_control_error_kickout_slack_mode &&
      std::fabs(vehicle_state_.pose().accel_body().y()) >
          kContLatAccThreshold) {
    return absl::InternalError("Lateral acc is too large!");
  }

  // Fill other control_command.
  control_command->set_gear_location(trajectory.gear());
  auto mpc_debug = control_command->mutable_debug()->mutable_simple_mpc_debug();
  mpc_debug->mutable_steering_protection_result()->CopyFrom(
      steering_protection_result);
  mpc_debug->set_steer_percentage_feedforward(
      steering_converter_ptr_->KappaToSteerPct(
          control_command->debug().simple_mpc_debug().kappa_feedforward()));
  mpc_debug->set_lateral_error(control_error.lateral_error);
  mpc_debug->set_heading_error(control_error.heading_error);
  control_command->mutable_debug()->mutable_control_error()->set_lateral_error(
      control_error.lateral_error);
  control_command->mutable_debug()->mutable_control_error()->set_heading_error(
      control_error.heading_error);
  control_command->mutable_custom_command()->set_low_speed_freespace(
      trajectory.low_speed_freespace());

  // Fill control_debug
  controller_debug_proto->set_active_lat_controller("lka_control");

  // Record lka control state;
  previous_kappa_cmd_ = control_command->curvature();

  return absl::OkStatus();
}

void LKAController::Reset() {
  lat_km_mpc_controller_ptr_->Reset(vehicle_state_);
  torque_controller_ptr_->Reset();
  previous_kappa_cmd_ = 0.0;
  is_first_run_ = true;
}

LonControllerOutputProto LKAController::CreateLonControllerOutput(
    double acc_pose) {
  LonControllerOutputProto lon_control_output;
  lon_control_output.set_is_standstill(false);
  for (int i = 0; i < kTControlHorizon; ++i) {
    lon_control_output.add_t_control_acc_vec(acc_pose);
  }
  return lon_control_output;
}

}  // namespace control
}  // namespace qcraft
