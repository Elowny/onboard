#include "onboard/control/vehicle_control_module.h"

#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/planner/freespace/proto/freespace_planner.pb.h"

// IWYU pragma: no_include <ostream>
// IWYU pragma: no_include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"

#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/control/control_monitoring.h"
#include "onboard/control/control_validation.h"
#include "onboard/control/controllers/controller_common.h"
#include "onboard/control/controllers/torque_controller.h"
#include "onboard/control/param/control_param_integrator.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_protection.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/global/run_context.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/q_issue.pb.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

DEFINE_bool(control_openloop_enable, false,
            "is or not enable openloop control");

namespace qcraft::control {

namespace {

// Continuous lateral control error thresholds for kickout;
constexpr double kContLatErrThreshold = 0.5;  // m.
// Continuous lateral acc  thresholds for kickout;
constexpr double kContLatAccThreshold = 4.0;  // m.

absl::Status IsAllowToEngage(double av_speed, double max_steer_angle,
                             double steering_pct_cmd, double steering_pct_fb) {
  const double steering_cmd_rad = 0.01 * max_steer_angle * steering_pct_cmd;
  const double steering_canbus_rad = 0.01 * max_steer_angle * steering_pct_fb;

  if (std::fabs(steering_cmd_rad - steering_canbus_rad) >
          FLAGS_max_steering_angle_diff_threshold &&
      std::fabs(av_speed) > FLAGS_engage_protection_min_speed) {
    return absl::OutOfRangeError(absl::StrCat(
        "The steering angle diff between control command and "
        "canbus status is out of range: \n control command is:\t",
        r2d(steering_cmd_rad), " degree, while canbus steering is at \t",
        r2d(steering_canbus_rad), " degree, at speed:\t", av_speed, " m/s"));
  }

  return absl::OkStatus();
}

}  // namespace

VehicleControlModule::VehicleControlModule(LiteClientBase* lite_client)
    : LiteModule(lite_client) {}

VehicleControlModule::~VehicleControlModule() {
  if (IsOnboardMode()) {
    dynamic_param_.set_steering_angle_bias(
        parameter_identificator_->SteerBiasOutput());
    const auto status = SaveDynamicParamProto(dynamic_param_);
    if (!status.ok()) {
      QEVENT("shijun", "Fail_save_dynamic_param", [&](QEvent* qevent) {
        qevent->AddField("error message", status.ToString());
      });
    }
  }
}

void VehicleControlModule::OnInit() {
  QLOG(INFO) << "Control init, starting ...";

  RunParamsProtoV2 run_params = GetRunParams();

  if (IsOnboardMode()) {
    QCHECK(run_params.vehicle_params().has_vehicle_geometry_params() &&
           run_params.vehicle_params().has_vehicle_drive_params() &&
           run_params.vehicle_params().has_controller_conf());
  } else {
    // Legacy.
    if (!run_params.vehicle_params().has_vehicle_geometry_params() ||
        !run_params.vehicle_params().has_vehicle_drive_params() ||
        !run_params.vehicle_params().has_controller_conf()) {
      // Note(mike): this logic is needed to provide backward compatibility to
      // old runs which don't have the new params in the log. Here, we use a new
      // param manager to avoid interfering with other places using the global
      // param manager.
      CreateParamManagerFromCarId("Q0001")->GetRunParams(&run_params);
    }
  }

  QLOG(INFO) << "Load vehicle-based controller configuration from: "
             << run_params.vehicle_params().car_id();

  vehicle_geometry_params_ =
      run_params.vehicle_params().vehicle_geometry_params();
  vehicle_drive_params_ = run_params.vehicle_params().vehicle_drive_params();
  control_conf_ = run_params.vehicle_params().controller_conf();
  QCHECK_OK(IntegrateControlParam(
      run_params.vehicle_params().vehicle_params().model(), &control_conf_));
  const auto vehicle_type =
      run_params.vehicle_params().vehicle_info().vehicle_interface();
  const auto vehicle_model =
      run_params.vehicle_params().vehicle_params().model();

  steering_converter_ = std::make_unique<const SteeringConverter>(
      vehicle_geometry_params_, vehicle_drive_params_, vehicle_type);
  trajectory_interface_ = std::make_unique<TrajectoryInterface>(vehicle_model);
  if (control_conf_.active_controllers_size() == 0) {
    control_conf_.add_active_controllers(
        ControllerConf::TOB_TSPKMPC_CONTROLLER);
  }

  double init_steer_angle_bias = 0.0;
  if (IsOnboardMode()) {
    const auto status = LoadDynamicParamProto(&dynamic_param_);
    // Default init steer angle bias in dynamic param is 0.0 which is invalid.
    // then set init steer angle bias from configure files to forbid the car
    // driving not straight.
    init_steer_angle_bias = dynamic_param_.steering_angle_bias() == 0.0
                                ? vehicle_drive_params_.steering_angle_bias()
                                : dynamic_param_.steering_angle_bias();
    if (!status.ok()) {
      QLOG(ERROR) << "Fail load dynamic param: " << status.ToString();
      QEVENT("shijun", "Fail_load_dynamic_param", [&](QEvent* qevent) {
        qevent->AddField("error message", status.ToString());
      });
      init_steer_angle_bias = vehicle_drive_params_.steering_angle_bias();
    }
  }

  parameter_identificator_.emplace(control_conf_, steering_converter_.get(),
                                   init_steer_angle_bias);
  // set controller
  controller_agent_ = std::make_unique<ControllerAgent>(
      &vehicle_geometry_params_, &vehicle_drive_params_, &control_conf_,
      steering_converter_.get());

  double steer_delay_time = 0.0;
  if (control_conf_.has_steer_delay_time()) {
    steer_delay_time = control_conf_.steer_delay_time();
  } else if (control_conf_.has_steering_delay_step()) {
    steer_delay_time = control_conf_.steering_delay_step() *
                       control_conf_.ts_pkmpc_controller_conf().ts();
  }
  lon_wire_control_checker_ = std::make_unique<LonWireControlChecker>();
  lat_wire_control_checker_ = std::make_unique<LatWireControlChecker>(
      steer_delay_time, vehicle_geometry_params_.wheel_base(),
      vehicle_drive_params_.steer_ratio(),
      vehicle_drive_params_.max_steer_angle());

  steer_calibration_.emplace(control_conf_, steering_converter_.get());
  lon_post_process_manager_ =
      std::make_unique<LonPostProcess>(&control_conf_, &vehicle_drive_params_);

  if (FLAGS_control_openloop_enable) {
    openloop_controller_ = std::make_unique<OpenloopControl>(
        vehicle_geometry_params_.wheel_base(),
        vehicle_drive_params_.steer_ratio(),
        vehicle_drive_params_.max_steer_angle());
  }

  if (vehicle_drive_params_.steer_interface() == SteerInterface::STEER_TORQUE) {
    torque_controller_ =
        std::make_unique<TorqueController>(control_conf_.steer_torque_conf());
  }

  if (control_conf_.mrac_conf().enable_mrac()) {
    mrac_control_ = std::make_unique<MracControl>(
        control_conf_.mrac_conf(), control_conf_.steer_delay_time());
  }

  const bool is_available_idle =
      vehicle_drive_params_.calibration_table_v2().has_idle_v_a_plf();
  acc_closed_loop_ =
      std::make_unique<ClosedLoopAcc>(control_conf_, is_available_idle,
                                      vehicle_drive_params_.throttle_deadzone(),
                                      vehicle_drive_params_.brake_deadzone());
  parking_manager_ = std::make_unique<ParkingManager>(vehicle_model);
}

void VehicleControlModule::OnSubscribeChannels() {
  Subscribe(&VehicleControlModule::OnAutonomyState, this, 20);
  Subscribe(&VehicleControlModule::OnTrajectory, this, 20);
  Subscribe(&VehicleControlModule::OnLocalizationDebug, this, 20);

  if (IsOnboardMode() || !FLAGS_control_replay_pose) {
    Subscribe(&VehicleControlModule::OnPoseProto, this, "pose_proto", 50);
  } else {
    Subscribe(&VehicleControlModule::OnPoseProto, this, "sensor_pose", 50);
  }

  if (IsOnboardMode() || !FLAGS_control_replay_chassis) {
    Subscribe(&VehicleControlModule::OnChassis, this, "chassis", 25);
  } else {
    Subscribe(&VehicleControlModule::OnChassis, this, "shadow_chassis", 25);
  }
}

void VehicleControlModule::OnSetUpTimers() {
  if (OnTestBenchForDriverTest()) return;

  AddTimerOrDie("vehicle_control_main_loop", &VehicleControlModule::Proc, this,
                absl::Milliseconds(990), absl::Seconds(kControlInterval),
                /*one_shot=*/false);
}

void VehicleControlModule::OnAutonomyState(
    std::shared_ptr<const AutonomyStateProto> autonomy_state) {
  local_view_.autonomy_state = std::move(autonomy_state);
}

void VehicleControlModule::OnChassis(std::shared_ptr<const Chassis> chassis) {
  local_view_.chassis = std::move(chassis);
}

void VehicleControlModule::OnTrajectory(
    std::shared_ptr<const TrajectoryProto> trajectory) {
  local_view_.trajectory = std::move(trajectory);
}

void VehicleControlModule::OnPoseProto(std::shared_ptr<const PoseProto> pose) {
  local_view_.pose = std::move(pose);
}

void VehicleControlModule::OnLocalizationDebug(
    std::shared_ptr<const LocalizationViewerDebugProto> localization_debug) {
  local_view_.localization_debug = std::move(localization_debug);
}

absl::Status VehicleControlModule::ProduceControlCommand(
    ControlCommand* control_command,
    ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("ProduceControlCommand");
  RETURN_IF_ERROR(UpdateInput(local_view_, controller_debug_proto));
  RETURN_IF_ERROR(CheckTimestamp(local_view_, is_input_ready_));
  is_input_ready_ = true;

  if (local_view_.trajectory->trajectory_point().empty() &&
      local_view_.trajectory->directional_path().path().empty()) {
    if (planner_trajectory_ready_) {
      QISSUEX(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
              QIssueSubType::QIST_PLANNER_PROTO_TRAJECTORY_EMPTY,
              "Check control msg: EMPTY(trajectory proto)");
    }
    return absl::InvalidArgumentError(
        "Check control msg: trajectory is not ready.");
  } else {
    planner_trajectory_ready_ = true;
  }

  controller_agent_->MayBeReset(vehicle_state_interface_.Result(),
                                ParkingState{/*reset_lon_controller*/ false,
                                             /*reset_lon_controller*/ false});

  const double previous_kappa_cmd =
      control_cache_mgr_.QueryKappaCmd(/*step*/ 0);

  SteeringProtection steering_protection(
      vehicle_drive_params_, steering_converter_.get(), &control_conf_);
  const SteeringProtectionResult steering_protection_result =
      steering_protection.CalcKappaAndKappaRateLimit(
          previous_kappa_cmd, vehicle_state_interface_.Result());
  control_command->mutable_debug()
      ->mutable_simple_mpc_debug()
      ->mutable_steering_protection_result()
      ->CopyFrom(steering_protection_result);

  ASSIGN_OR_RETURN(const auto gear_cmd,
                   GenerateGearCmd(local_view_.chassis->gear_location(),
                                   local_view_.trajectory->gear(),
                                   local_view_.pose->vel_body().x()));
  control_command->set_gear_location(gear_cmd);

  const double long_delay_time = acc_closed_loop_->GetLongitudinalDelay(
      control_conf_.closed_loop_acc_conf().brake_delay_time(),
      control_conf_.closed_loop_acc_conf().throttle_delay_time());
  controller_debug_proto->set_longitudinal_delay(long_delay_time);
  const int lon_delay_steps = RoundToInt(long_delay_time * kControlFrequency);

  const int lat_delay_steps = RoundToInt(
      parameter_identificator_->SteerDelayTime() * kControlFrequency);

  // Calculate control errors ( Estimate the lateral error due to canbus noise
  // by integrating lateral_error_canbus);
  control_command->mutable_debug()->mutable_control_error()->CopyFrom(
      CalculateControlError(
          control_conf_.enable_yaw_consider_slip(),
          vehicle_state_interface_.Result(), *trajectory_interface_,
          *trajectory_interface_,
          control_cache_mgr_.IntegrateLatErrorCanbus(
              static_cast<int>(/*duration*/ 1.0 * kControlFrequency)),
          control_cache_mgr_.QueryAccTarget(lon_delay_steps),
          steering_converter_->SteerAngleToSteerPct(
              steering_converter_->KappaToSteerAngle(previous_kappa_cmd) -
              parameter_identificator_->SteerBiasOutput())));

  // Calculate lateral prediction errors.
  controller_debug_proto->mutable_predicted_error_proto()->CopyFrom(
      CalculateLateralPredictionError(
          vehicle_state_interface_.Result(), control_cache_mgr_,
          parameter_identificator_->SteerDelayTime()));

  // Wrap control init pose;
  ControllerInitPose controller_init_pose = WrapControllerInitPose(
      lon_delay_steps, lat_delay_steps, *steering_converter_,
      vehicle_state_interface_.Result(), control_cache_mgr_,
      vehicle_geometry_params_, controller_debug_proto);

  LonControllerOutputProto lon_controller_output;
  const bool is_standstill =
      IsStandstill(vehicle_state_interface_.Result().linear_velocity());
  lon_controller_output.set_is_standstill(is_standstill);
  SCOPED_QTRACE("ControllerAgentMainLoop");
  absl::Status status_compute = controller_agent_->ComputeControlCommand(
      vehicle_state_interface_.Result(), *trajectory_interface_,
      {steering_protection_result}, controller_init_pose, control_command,
      controller_debug_proto, &lon_controller_output);

  if (IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state()) &&
      !status_compute.ok()) {
    const auto args_message = std::string(status_compute.message());
    // TODO(zhichao): separate controller error types.
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_CONTROL_COMPUTE_COMMAND_FAILED,
                      "Check control main function: FAILED", args_message);
    return absl::InternalError(args_message);
  }

  if (!IsOnboardMode()) {
    QLOG_IF_NOT_OK(WARNING, Publish(lon_controller_output));
  }

  // longitudinal cmd post-processing.
  const double kappa_rate = steering_converter_->SteerRateToKappaRate(
      control_command->steer_speed_target(),
      steering_converter_->SteerPctToSteerAngle(
          control_command->steering_target()));

  LonPostProcessInput lon_post_process_input(
      IsOnboardMode(), trajectory_interface_->GetIsLowSpeedFreespace(),
      kappa_rate, *control_command, vehicle_state_interface_.Result(),
      *trajectory_interface_);
  controller_debug_proto->mutable_lon_post_process_debug_proto()->set_jerk_pose(
      (vehicle_state_interface_.Result().linear_acceleration() -
       control_cache_mgr_.QueryAccPose(lon_delay_steps)) /
      long_delay_time);
  lon_post_process_manager_->Process(lon_post_process_input, control_command,
                                     controller_debug_proto);

  // lateral cmd post-processing: mrac compute.
  if (control_conf_.mrac_conf().enable_mrac()) {
    const MracInput mrac_input = {
        .is_automode = vehicle_state_interface_.Result().is_auto_steer(),
        .kappa_target = control_command->curvature(),
        .av_kappa = vehicle_state_interface_.Result().kappa(),
        .speed = vehicle_state_interface_.Result().linear_velocity(),
        .kappa_upper = steering_protection_result.kappa_output_upper()[0],
        .kappa_lower = steering_protection_result.kappa_output_lower()[0]};
    const double kappa_cmd_mrac = mrac_control_->Compute(
        mrac_input, controller_debug_proto->mutable_mrac_debug_proto());
    control_command->set_curvature(kappa_cmd_mrac);
  }

  // Freesapce parking porcess
  const auto vehicle_state = vehicle_state_interface_.Result();
  const ParkingManagerInput parking_input = {
      .trajectory_interface = trajectory_interface_.get(),
      .vehicle_state = &vehicle_state,
      .steering_protection_result = &steering_protection_result,
      .steering_converter = steering_converter_.get(),
      .is_onboard = IsOnboardMode()};

  const ParkingState parking_state = parking_manager_->ParkingProcess(
      parking_input, control_command, controller_debug_proto);
  controller_agent_->MayBeReset(vehicle_state_interface_.Result(),
                                parking_state);

  // Steering protection and check.
  const auto mpc_steer_state = steering_protection.SteerResultStatus(
      vehicle_state_interface_.Result(), control_cache_mgr_,
      *controller_debug_proto, steering_protection_result);
  if (!mpc_steer_state.ok()) {
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_CONTROL_STEER_ANGLE_SPEED_TOO_FAST,
                      "Steering too fast", mpc_steer_state.ToString());
  }

  if (control_conf_.bias_estimation_conf().enable_online_bias_estimation()) {
    ParameterIdentificationInput input = {
        .steer_cmd = steering_converter_->KappaToFrontWheelAngle(
            control_command->curvature()),
        .steer_pose = steering_converter_->KappaToFrontWheelAngle(
            local_view_.pose->curvature()),
        .steer_feedback = steering_converter_->SteerPctToFrontWheelAngle(
            local_view_.chassis->steering_percentage()),
        .speed_measurement =
            vehicle_state_interface_.Result().linear_velocity(),
        .lat_acc = local_view_.pose->accel_body().y(),
        .lat_error = control_command->debug().control_error().lateral_error(),
        .heading_err = control_command->debug().control_error().heading_error(),
        .is_auto = vehicle_state_interface_.Result().is_auto_steer(),
        .control_cache_mgr = &control_cache_mgr_};

    parameter_identificator_->EstimateLatBias(
        input,
        control_command->mutable_debug()->mutable_bias_estimation_debug());
  }

  if (control_conf_.bias_estimation_conf().use_low_pass_filter_estimation()) {
    control_command->set_steer_angle_bias(
        control_command->debug().bias_estimation_debug().steer_bias());
  }

  QCOUNTER("heading_bias_deg * 1e6",
           RoundToInt(r2d(parameter_identificator_->HeadingBias()) * 1e6));
  const double cmd_kappa_clamped =
      steering_converter_->ClampKappaByMaxSteerAngle(
          control_command->curvature());
  double steer_target_calibration =
      steering_converter_->KappaToSteerPct(cmd_kappa_clamped);

  if (IsOnboardMode()) {
    constexpr int kPastSteps = 10;  // 0.1 s.
    const int delay_step = static_cast<int>(
        parameter_identificator_->SteerDelayTime() * kControlFrequency);
    steer_target_calibration = steer_calibration_->SteerCalibrationMain(
        control_command->curvature(),
        control_cache_mgr_.QueryKappaCmd(kPastSteps),
        local_view_.pose->curvature(),
        control_cache_mgr_.QueryKappaCmd(delay_step),
        vehicle_state_interface_.Result().linear_velocity(),
        local_view_.pose->roll(),
        controller_debug_proto->mutable_steer_calibration_debug_proto());
  }
  QCOUNTER("steering_percentage", RoundToInt(steer_target_calibration));
  control_command->set_steering_target(steer_target_calibration);
  control_command->set_steering_target_wrt_bias(
      steer_target_calibration - control_command->steer_angle_bias() /
                                     vehicle_drive_params_.max_steer_angle() *
                                     100.0);
  control_command->mutable_debug()
      ->mutable_simple_mpc_debug()
      ->set_steer_percentage_feedforward(steering_converter_->KappaToSteerPct(
          control_command->debug().simple_mpc_debug().kappa_feedforward()));

  if ((vehicle_drive_params_.steer_interface() ==
       SteerInterface::STEER_TORQUE)) {
    auto vehicle_state = vehicle_state_interface_.Result();
    const TorqueControllerInput torque_input = {
        .is_auto_steer = vehicle_state.is_auto_steer(),
        .is_lka = false,
        .steer_angle_target_past = steering_converter_->KappaToSteerPct(
            control_cache_mgr_.QueryKappaCmd(RoundToInt(
                control_conf_.bias_estimation_conf().steer_status_delay_time() *
                kControlFrequency))),
        .vehicle_state = &vehicle_state,
        .control_cmd = control_command,
        .steering_converter = steering_converter_.get()};
    const double torque_cmd = torque_controller_->ComputeSteerTorqueTarget(
        torque_input, controller_debug_proto);
    control_command->set_torque_target(torque_cmd);
    control_command->set_steer_mode(SteerMode::TORQUE_MODE);
  } else {
    control_command->set_torque_target(0.0);
    control_command->set_steer_mode(SteerMode::ANGLE_MODE);
  }

  // If freespace and active torque controller,
  // so set SteerMode::ANGLE_MODE and reset torque controller.
  if (trajectory_interface_->GetIsLowSpeedFreespace() &&
      (vehicle_drive_params_.steer_interface() ==
       SteerInterface::STEER_TORQUE)) {
    control_command->set_torque_target(0.0);
    control_command->set_steer_mode(SteerMode::ANGLE_MODE);
    torque_controller_->Reset();
  }

  LightControl(*local_view_.trajectory, control_command);
  DoorControl(*local_view_.trajectory, control_command);

  control_cache_mgr_.UpdateCacheData(
      *control_command, vehicle_state_interface_.Result(),
      {.delay_time = parameter_identificator_->SteerDelayTime(),
       .control_debug = controller_debug_proto,
       .trajectory_interface = trajectory_interface_.get(),
       .steering_converter = steering_converter_.get(),
       .veh_predicted_pose =
           &controller_debug_proto->predicted_veh_pose_proto()});

  return absl::OkStatus();
}

// TODO(lidong): Change this function to return absl::Status.
void VehicleControlModule::Proc() {
  SCOPED_QTRACE("VehicleControlModule::Proc");
  if (local_view_.pose == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "Pose does not exist.";
    return;
  }
  if (local_view_.chassis == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "Chassis does not exist.";
    return;
  }

  ControlCommand control_command;
  ControllerDebugProto controller_debug_proto;

  // Openloop controller to build command.
  if (FLAGS_control_openloop_enable) {
    if (IS_AUTO_DRIVE(local_view_.autonomy_state->autonomy_state())) {
      openloop_controller_->CreateCommand(&control_command);
      QLOG_IF_NOT_OK(WARNING, Publish(control_command));
      QLOG_IF_NOT_OK(WARNING, Publish(controller_debug_proto));
    } else {
      openloop_controller_->Reset();
    }
    return;
  }

  vis::vantage::ChartsDataProto chart_data;

  if (local_view_.autonomy_state == nullptr) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "AutonomyState does not exist.";
    return;
  }
  parameter_identificator_->Process(vehicle_state_interface_.Result(),
                                    &control_command,
                                    control_cache_mgr_.QueryKappaCmd(0));
  const double steer_bias_output = parameter_identificator_->SteerBiasOutput();
  if (FLAGS_use_dynamic_steer_angle_bias) {
    control_command.set_steer_angle_bias(steer_bias_output);
  } else {
    control_command.set_steer_angle_bias(
        vehicle_drive_params_.steering_angle_bias());
  }

  const auto status =
      ProduceControlCommand(&control_command, &controller_debug_proto);

  QCOUNTER("steer_angle_bias_rad*100", RoundToInt(steer_bias_output * 100.0));
  QCOUNTER(
      "steer_angle_bias_rad_lowpass_filter*1e6",
      RoundToInt(control_command.debug().bias_estimation_debug().steer_bias() *
                 1e6));
  // TODO(shijun): Tease return logic on a macro level.
  if (!status.ok()) {
    QLOG_EVERY_N(ERROR, 200)
        << "Failed to produce control command:" << status.message();
    return;
  }

  // Check large continous lateral error;
  constexpr double kLatErrCheckHorizon = 0.2;  // s.
  const int lat_err_check_step = kLatErrCheckHorizon * kControlFrequency;
  if (!FLAGS_control_error_kickout_slack_mode &&
      control_cache_mgr_.IsInAlwaysAutoMode(lat_err_check_step) &&
      control_cache_mgr_.QueryMinAbsLatError(lat_err_check_step) >
          kContLatErrThreshold) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_LATERAL_ERROR_TOO_LARGE,
        "continous lateral error too large",
        absl::StrFormat(
            "Min lateral error [%f]'s abs value in last %f seconds "
            "is larger than %f.",
            control_cache_mgr_.QueryMinAbsLatError(lat_err_check_step),
            kLatErrCheckHorizon, kContLatErrThreshold));
  }

  // Check large continous lateral acceleration;
  constexpr double kLatAccCheckHorizon = 0.3;  // s.
  const int lat_acc_check_step = kLatAccCheckHorizon * kControlFrequency;
  if (!FLAGS_control_error_kickout_slack_mode &&
      control_cache_mgr_.IsInAlwaysSteerMode(lat_acc_check_step) &&
      control_cache_mgr_.QueryMinAbsLatACC(lat_acc_check_step) >
          kContLatAccThreshold) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CONTROL_LATERAL_ACC_TOO_LARGE,
        "continous lateral acc too large",
        absl::StrFormat(
            "Min lateral acc [%f]'s abs value in last %f seconds "
            "is larger than %f.",
            control_cache_mgr_.QueryMinAbsLatError(lat_acc_check_step),
            kLatAccCheckHorizon, kContLatAccThreshold));
  }

  // Wire control checker
  if (lon_wire_control_checker_->IsAbnormal(
          vehicle_state_interface_.Result(), control_command,
          controller_debug_proto.mutable_wire_control_check_proto())) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CHASSIS_LON_OUT_OF_CONTROL,
        "CANBUS check fails: longitudinal wire control is abnormal.",
        absl::StrFormat(
            "speed is [%f] m/s, acceleration is [%f] m/s2.",
            vehicle_state_interface_.Result().linear_velocity(),
            vehicle_state_interface_.Result().linear_acceleration()));
    return;
  }

  if (lat_wire_control_checker_->IsAbnormal(
          vehicle_state_interface_.Result(), control_command, IsOnboardMode(),
          controller_debug_proto.mutable_wire_control_check_proto())) {
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_CHASSIS_LAT_OUT_OF_CONTROL,
        "CANBUS check fails: lateral wire control is abnormal.",
        absl::StrFormat(
            "speed is [%f] m/s, steer error is [%f] rad, kappa error is [%f].",
            vehicle_state_interface_.Result().linear_velocity(),
            controller_debug_proto.mutable_wire_control_check_proto()
                ->steer_error(),
            controller_debug_proto.mutable_wire_control_check_proto()
                ->kappa_error()));
    return;
  }

  // Engage protection, check steering cmd and steering fb status when system is
  // in READY_TO_AUTO_DRIVE state to forbid collision accident.
  if (vehicle_state_interface_.Result().autonomy_state() ==
      AutonomyStateProto::READY_TO_AUTO_DRIVE) {
    const auto allow_engage_status =
        IsAllowToEngage(local_view_.pose->vel_body().x(),
                        vehicle_drive_params_.max_steer_angle(),
                        control_command.steering_target(),
                        local_view_.chassis->steering_percentage());
    if (!allow_engage_status.ok() && IsOnboardMode()) {
      const auto args_message = std::string(allow_engage_status.message());
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL,
                        QIssueType::QIT_PREVENT_ENGAGE,
                        QIssueSubType::QIST_CONTROL_STEER_FEEDBACK_BIG_DIFF,
                        "CANBUS status is not ready to engage: steering cmd "
                        "and feedback have a big diff",
                        args_message);
      return;
    }
  }

  const bool is_output_valid = ValidateControlOutput(
      vehicle_state_interface_.Result(), *steering_converter_, control_conf_,
      control_command, &controller_debug_proto);
  if (!is_output_valid) {
    const auto args_message = std::string(
        controller_debug_proto.validation_result_proto().DebugString());
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_CONTROL_VALIDATE_OUTPUT_FAILED,
                      "Control output validation fails: ", args_message);
  }

  const double prev_kappa_cmd = control_cache_mgr_.QueryKappaCmd(/*step*/ 1);
  WrapSteerConstraintChartData(prev_kappa_cmd, control_conf_, control_command,
                               controller_debug_proto, &chart_data);

  // Collect control qevents:
  QCounterControlError(vehicle_state_interface_.Result(),
                       control_command.debug().control_error());
  const auto& controller_info = control_command.debug().simple_mpc_debug();
  constexpr double kHardBrakeThreshold = -2.0;  // m/s^2.
  if (vehicle_state_interface_.Result().is_auto_speed() &&
      control_command.acceleration() < kHardBrakeThreshold) {
    QEventHardBrake(control_command.acceleration(),
                    trajectory_interface_->GetMinAccelFromTrajectory(),
                    controller_info.acceleration_reference(),
                    control_cache_mgr_);
  }

  if (local_view_.chassis->has_auxiliary_button() &&
      local_view_.chassis->auxiliary_button()) {
    QEventDiscomfortable(control_cache_mgr_, *steering_converter_,
                         *local_view_.pose);
  }

  QEventTrackingError(controller_info.steer_percentage_feedforward(),
                      controller_info.speed_reference(), control_cache_mgr_);
  QEventControlCache(vehicle_state_interface_.Result().linear_velocity(),
                     control_cache_mgr_);

  QLOG_IF_NOT_OK(WARNING, Publish(control_command));
  QLOG_IF_NOT_OK(WARNING, Publish(controller_debug_proto));
  QLOG_IF_NOT_OK(WARNING, Publish(chart_data));
}

absl::Status VehicleControlModule::UpdateInput(
    const LocalView& local_view, ControllerDebugProto* controller_debug_proto) {
  SCOPED_QTRACE("UpdateInput");
  if (local_view.pose == nullptr) {
    return absl::FailedPreconditionError("Pose does not exist.");
  }
  if (local_view.chassis == nullptr) {
    return absl::FailedPreconditionError("Chassis does not status");
  }
  if (local_view.autonomy_state == nullptr) {
    return absl::FailedPreconditionError("AutonomyState does not exist.");
  }
  if (local_view.trajectory == nullptr) {
    return absl::FailedPreconditionError("Trajectory does not exist.");
  }

  const double yaw_bias =
      control_conf_.has_bias_estimation_conf() &&
              control_conf_.bias_estimation_conf().enable_compensate_yaw_bias()
          ? parameter_identificator_->HeadingBias()
          : 0.0;
  auto localization_debug =
      local_view.localization_debug == nullptr
          ? std::nullopt
          : std::make_optional(*local_view.localization_debug);

  RETURN_IF_ERROR(vehicle_state_interface_.Update(
      yaw_bias, local_view.autonomy_state->autonomy_state(), *local_view.pose,
      *local_view.chassis, localization_debug, *steering_converter_,
      controller_debug_proto));

  RETURN_IF_ERROR(trajectory_interface_->Update(
      vehicle_state_interface_.Result().autonomy_state() ==
          AutonomyStateProto::EMERGENCY_TO_STOP,
      *local_view.trajectory, controller_debug_proto))
      << "Trajectory has empty message.";

  QCounterPose(*local_view.pose);

  return absl::OkStatus();
}

absl::Status VehicleControlModule::CheckTimestamp(const LocalView& local_view,
                                                  bool is_input_ready) {
  const double current_timestamp = ToUnixDoubleSeconds(Clock::Now());

  // Positioning signal time delay (QEvent or kickout).
  const double pose_time_diff =
      current_timestamp - local_view.pose->timestamp();
  const double max_time_diff =
      control_conf_.max_pose_miss_num() * control_conf_.positioning_period();
  constexpr double kQEventPoseDelayThreshold = 0.15;  // s.
  if (pose_time_diff > kQEventPoseDelayThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "pose_delay_too_much",
                           /*seconds=*/5.0, [&](QEvent* qevent) {
                             qevent->AddField("pose_time_diff", pose_time_diff);
                           });
  }
  if (pose_time_diff > max_time_diff) {
    const auto args_message = absl::StrFormat(
        "Pose msg timeout, pose_time: %f, now: %f, diff: %f, threshold: %f",
        local_view.pose->timestamp(), current_timestamp, pose_time_diff,
        max_time_diff);
    if (is_input_ready) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_POSITION_PROTO_POSE_TIMEOUT,
                        "Check timestamp: FAILED", args_message);
    }
    return absl::DataLossError(args_message);
  }

  // Canbus signal time delay (QEvent or kickout).
  const double chassis_time_diff =
      current_timestamp - local_view.chassis->header().timestamp() * 1e-6;
  const double max_chassis_time_diff =
      control_conf_.max_chassis_miss_num() * control_conf_.chassis_period();
  constexpr double kQEventChassisDelayThreshold = 0.15;  // s.
  if (chassis_time_diff > kQEventChassisDelayThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "canbus_delay_too_much",
                           /*seconds=*/5.0, [&](QEvent* qevent) {
                             qevent->AddField("chassis_time_diff",
                                              chassis_time_diff);
                           });
  }
  if (chassis_time_diff > max_chassis_time_diff) {
    const auto args_message = absl::StrFormat(
        "Chassis msg timeout, time_diff: %f, max_chassis_time_diff: %f",
        chassis_time_diff, max_chassis_time_diff);
    if (is_input_ready) {
      QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_CHASSIS_PROTO_CHASSIS_TIMEOUT,
                        "Check timestamp: FAILED", args_message);
    }
    return absl::DataLossError(args_message);
  }

  // Planner signal time delay (QEvent).
  const double planner_time_delay =
      current_timestamp - local_view.trajectory->header().timestamp() * 1e-6;
  constexpr double kQEventPlannerDelayThreshold = 0.5;  // s.
  if (planner_time_delay > kQEventPlannerDelayThreshold) {
    QEVENT_EVERY_N_SECONDS("zhichao", "planner_delay_too_much",
                           /*seconds=*/5.0, [&](QEvent* qevent) {
                             qevent->AddField("planner_time_diff",
                                              planner_time_delay);
                           });
  }
  // Kickout if planner delays too much.
  constexpr double kQIssuePlannerDelayThreshold = 3.0;  // s.
  if (planner_time_delay > kQIssuePlannerDelayThreshold) {
    const auto args_message = absl::StrFormat(
        "Trajectory msg timeout, time_diff: %f, max_planner_time_diff: %f",
        planner_time_delay, kQIssuePlannerDelayThreshold);
    if (is_input_ready) {
      // In L2 mode, when switch from lcc to acc, if odc fail, planner will
      // not publish trajectory, so don't let control kickout which will cause
      // continuous odc fail.
      const auto q_issue_severity = IsRunModeL4() ? QIssueSeverity::QIS_FATAL
                                                  : QIssueSeverity::QIS_WARNING;
      QISSUEX_WITH_ARGS(q_issue_severity, QIssueType::QIT_BUSINESS,
                        QIssueSubType::QIST_PLANNER_PROTO_TRAJECTORY_TIMEOUT,
                        "Control check planner's timestamp failed.",
                        args_message);
    }
    return absl::DataLossError(args_message);
  }

  return absl::OkStatus();
}
}  // namespace qcraft::control
