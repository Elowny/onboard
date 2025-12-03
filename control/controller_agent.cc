#include "onboard/control/controller_agent.h"

#include <cmath>
#include <memory>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/control_monitoring.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/math/util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace control {

namespace {

// Control error thresholds for kickout;
constexpr double kErrThresholdLateralAbs = 0.6;         // m.
constexpr double kErrThresholdHeadingAbs = M_PI / 6.0;  // rad.
constexpr double kErrThresholdStationForward = 5.0;     // m.
constexpr double kErrThresholdStationBackward = -5.0;   // m.
constexpr double kErrThresholdSpeedAbs = 3.0;           // m/s.

// Demo mode thresholds:
constexpr double kSlackErrThresholdLateralAbs = 2.0;         // m.
constexpr double kSlackErrThresholdHeadingAbs = M_PI / 6.0;  // rad
constexpr double kSlackErrThresholdStationForward = 5.0;     // m.
constexpr double kSlackErrThresholdStationBackward = -8.0;   // m.
constexpr double kSlackErrThresholdSpeedAbs = 10.0;          // m/s.

absl::Status ReportLatErrorIssue(const ControlError& control_error) {
#define CHECK_CONTROL_ERROR(msg)                                  \
  if (!control_error.has_##msg()) {                               \
    return absl::FailedPreconditionError(                         \
        absl::StrCat("Check control error field failed:", #msg)); \
  }
  CHECK_CONTROL_ERROR(lateral_error);
  CHECK_CONTROL_ERROR(heading_error);

#undef CHECK_CONTROL_ERROR

  const double error_threshold_lateral_abs =
      FLAGS_control_error_kickout_slack_mode ? kSlackErrThresholdLateralAbs
                                             : kErrThresholdLateralAbs;
  const double error_threshold_heading_abs =
      FLAGS_control_error_kickout_slack_mode ? kSlackErrThresholdHeadingAbs
                                             : kErrThresholdHeadingAbs;
  if (std::fabs(control_error.lateral_error()) >
      FLAGS_control_max_error_warning_factor * error_threshold_lateral_abs) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_WARNING, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_LATERAL_ERROR_TOO_LARGE,
        "lateral error too large warning",
        absl::StrCat("Control lateral error too large warning. Lateral error: ",
                     control_error.lateral_error(), " is not within threshold ",
                     FLAGS_control_max_error_warning_factor *
                         error_threshold_lateral_abs));
  }

  if (std::fabs(control_error.lateral_error()) > error_threshold_lateral_abs) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_LATERAL_ERROR_TOO_LARGE,
        "lateral error too large",
        absl::StrFormat("Lateral error [%f]'s abs value is larger than %f.",
                        control_error.lateral_error(),
                        error_threshold_lateral_abs));
  }
  if (std::fabs(control_error.heading_error()) > error_threshold_heading_abs) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_HEADING_ERROR_TOO_LARGE,
        "heading error too large",
        absl::StrFormat("Heading error [%f]'s abs value is larger than %f.",
                        control_error.heading_error(),
                        error_threshold_heading_abs));
  }
  return absl::OkStatus();
}

absl::Status ReportLonErrorIssue(const ControlError& control_error) {
#define CHECK_CONTROL_ERROR(msg)                                  \
  if (!control_error.has_##msg()) {                               \
    return absl::FailedPreconditionError(                         \
        absl::StrCat("Check control error field failed:", #msg)); \
  }
  CHECK_CONTROL_ERROR(speed_error);
  CHECK_CONTROL_ERROR(station_error);

#undef CHECK_CONTROL_ERROR

  const double error_threshold_station_forward =
      FLAGS_control_error_kickout_slack_mode ? kSlackErrThresholdStationForward
                                             : kErrThresholdStationForward;
  const double error_threshold_station_backward =
      FLAGS_control_error_kickout_slack_mode ? kSlackErrThresholdStationBackward
                                             : kErrThresholdStationBackward;
  const double error_threshold_speed_abs =
      FLAGS_control_error_kickout_slack_mode ? kSlackErrThresholdSpeedAbs
                                             : kErrThresholdSpeedAbs;
  if (control_error.station_error() < error_threshold_station_backward) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_STATION_BACKWARD_ERROR_TOO_LARGE,
        "station distance fallback too much",
        absl::StrFormat("Station error [%f] is less than %f",
                        control_error.station_error(),
                        error_threshold_station_backward));
  }
  if (control_error.station_error() > error_threshold_station_forward) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_STATION_FORWARD_ERROR_TOO_LARGE,
        "station distance advanced too much",
        absl::StrFormat("Station error [%f] is larger than range %f",
                        control_error.station_error(),
                        error_threshold_station_forward));
  }
  if (std::fabs(control_error.speed_error()) > error_threshold_speed_abs) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_SPEED_ERROR_TOO_LARGE,
        "speed error too large",
        absl::StrFormat("Speed error [%f]'s abs value is larger than %f.",
                        control_error.speed_error(),
                        error_threshold_speed_abs));
  }
  return absl::OkStatus();
}

void DebugLatControllerPose(const VehPose& lat_controller_pose,
                            ControllerDebugProto* controller_debug_proto) {
  const double yaw_diff = NormalizeAngle(lat_controller_pose.moving_direction -
                                         lat_controller_pose.heading);
  controller_debug_proto->mutable_vehicle_state_debug()
      ->mutable_slip_debug()
      ->set_yaw_diff_wrt_slip(yaw_diff);
}

}  // namespace

ControllerInitPose WrapControllerInitPose(
    int lon_delay_steps, int lat_delay_steps,
    const SteeringConverter& steering_converter,
    const VehicleStateProto& vehicle_state,
    const ControlCacheManager& control_cache,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    ControllerDebugProto* controller_debug_proto) {
  ControllerInitPose controller_init_pose;
  const VehPose current_pose(control_cache.QueryAccTarget(0),
                             &vehicle_geometry_params, vehicle_state);

  // TODO(zhichao): call prediction function once to find lat and lon predicted
  // poses.
  controller_init_pose.lon_pose = PredictControlInitPoseByKM(
      current_pose, steering_converter,
      control_cache.QueryKappaCmdVector(lon_delay_steps),
      control_cache.QueryAccTargetVector(lon_delay_steps));
  controller_init_pose.lon_pose.ToProto(
      controller_debug_proto->mutable_predicted_veh_lon_pose_proto());

  controller_init_pose.lat_pose_km = PredictControlInitPoseByKM(
      current_pose, steering_converter,
      control_cache.QueryKappaCmdVector(lat_delay_steps),
      control_cache.QueryAccTargetVector(lat_delay_steps));
  controller_init_pose.lat_pose_km.ToProto(
      controller_debug_proto->mutable_predicted_veh_pose_proto());

  controller_init_pose.lat_pose_dm = PredictControlInitPoseByDM(
      current_pose, steering_converter,
      control_cache.QuerySteerSpeedTargetVector(lat_delay_steps),
      control_cache.QueryAccTargetVector(lat_delay_steps));
  controller_init_pose.lat_pose_dm.ToProto(
      controller_debug_proto->mutable_predicted_veh_lat_dm_pose_proto());

  if (vehicle_state.is_auto_steer() &&
      vehicle_state.linear_velocity() > kDmSpeedThreshold) {
    QEventLatVehPosDiff(controller_init_pose.lat_pose_km,
                        controller_init_pose.lat_pose_dm);
  }

  return controller_init_pose;
}

ControllerAgent::ControllerAgent(
    const VehicleGeometryParamsProto* vehicle_geometry_params,
    const VehicleDriveParamsProto* vehicle_drive_params,
    const ControllerConf* control_conf,
    const SteeringConverter* steering_converter) {
  QCHECK_NOTNULL(vehicle_drive_params);
  QCHECK_NOTNULL(control_conf);
  QCHECK_NOTNULL(steering_converter);

  // TODO(shijun): Need to redesign register wrt config active controllers
  // compatibility. Refer to design doc:
  // https://qcraft.feishu.cn/docs/doccnJZ88ZAlThx6X50wJrYVCSg
  if (FLAGS_force_use_lon_tob_mpc_controller ||
      (control_conf->has_active_lon_controller() &&
       control_conf->active_lon_controller() ==
           LonControllerType::LON_TOB_PK_MPC)) {
    lon_tob_mpc_controller_ = std::make_optional<LonTobMpcController>(
        control_conf, vehicle_drive_params);
  } else {
    lon_mpc_controller_ = std::make_optional<LonMpcController>(
        control_conf, vehicle_drive_params);
  }

  if (FLAGS_force_use_lat_dm_mpc_controller) {
    lat_dm_mpc_controller_ = std::make_optional<LatDmMpcController>(
        vehicle_geometry_params, control_conf, steering_converter);
  } else {
    lat_km_mpc_controller_ = std::make_optional<LatKmMpcController>(
        control_conf, steering_converter);
  }
  control_conf_ = control_conf;
}

absl::Status ControllerAgent::ComputeControlCommand(
    const VehicleStateProto& vehicle_state,
    const TrajectoryInterface& trajectory_interface,
    const ControlConstraint& control_constraint,
    const ControllerInitPose& init_pose, ControlCommand* cmd,
    ControllerDebugProto* controller_debug_proto,
    LonControllerOutputProto* lon_controller_output) {
  QCHECK_NOTNULL(cmd);
  QCHECK_NOTNULL(controller_debug_proto);
  QCHECK_NOTNULL(lon_controller_output);

  if (lon_tob_mpc_controller_.has_value()) {
    RETURN_IF_ERROR(lon_tob_mpc_controller_->ComputeControlCommand(
        trajectory_interface, init_pose.lon_pose, cmd, controller_debug_proto,
        lon_controller_output));
  } else if (lon_mpc_controller_.has_value()) {
    RETURN_IF_ERROR(lon_mpc_controller_->ComputeControlCommand(
        trajectory_interface, init_pose.lon_pose, cmd, controller_debug_proto,
        lon_controller_output));
  }

  const VehPose& lat_controller_pose =
      control_conf_->enable_dynamic_prediction_pose() &&
              vehicle_state.linear_velocity() > kDmSpeedThreshold
          ? init_pose.lat_pose_dm
          : init_pose.lat_pose_km;
  DebugLatControllerPose(lat_controller_pose, controller_debug_proto);

  if (FLAGS_force_use_lat_dm_mpc_controller) {
    RETURN_IF_ERROR(lat_dm_mpc_controller_->ComputeControlCommand(
        vehicle_state, trajectory_interface,
        control_constraint.steering_protection_result, lat_controller_pose,
        *lon_controller_output, cmd, controller_debug_proto));
  } else {
    RETURN_IF_ERROR(lat_km_mpc_controller_->ComputeControlCommand(
        vehicle_state, trajectory_interface,
        control_constraint.steering_protection_result, lat_controller_pose,
        *lon_controller_output, cmd, controller_debug_proto));
  }

  const auto& control_error = cmd->mutable_debug()->control_error();
  // Don't report longitudinal kickout when AEB planner triggers.
  // Also don't report lateral/longitudinal error too big kickout when AV is in
  // EMERGENCY_TO_STOP, because in this state, av may be not under control.
  const auto autonomy_state = vehicle_state.autonomy_state();
  if ((autonomy_state == AutonomyStateProto::AUTO_DRIVE ||
       autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY) &&
      !trajectory_interface.aeb_triggered()) {
    RETURN_IF_ERROR(ReportLonErrorIssue(control_error));
  }
  if (autonomy_state == AutonomyStateProto::AUTO_DRIVE ||
      autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY) {
    RETURN_IF_ERROR(ReportLatErrorIssue(control_error));
  }

  return absl::OkStatus();
}

void ControllerAgent::MayBeReset(const VehicleStateProto& vehicle_state,
                                 const ParkingState& parking_state) {
  // Reset lon controller when lon controller is not under control.
  if (!vehicle_state.is_auto_speed() || parking_state.reset_lon_controller) {
    if (lon_tob_mpc_controller_.has_value()) {
      lon_tob_mpc_controller_->Reset(vehicle_state);
    } else if (lon_mpc_controller_.has_value()) {
      lon_mpc_controller_->Reset(vehicle_state);
    }
  }

  // Reset lat controller when lat controller is not under control.
  if (!vehicle_state.is_auto_steer() || parking_state.reset_lat_controller) {
    if (FLAGS_force_use_lat_dm_mpc_controller) {
      lat_dm_mpc_controller_->Reset(vehicle_state);
    } else {
      lat_km_mpc_controller_->Reset(vehicle_state);
    }
  }
}

}  // namespace control
}  // namespace qcraft
