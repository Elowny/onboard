#include "onboard/control/calibration/control_calibration_module.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"

#include "onboard/global/car_common.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/utils/status_macros.h"

DEFINE_string(savedir, "/hosthome/calibration", "save dir");
DEFINE_string(method, "IDLE", "IDLE, SLIDE, FORCE");
DEFINE_double(throttle_deadzone, 0.0, "throttle_deadzone");
DEFINE_double(brake_deadzone, 0.0, "brake_deadzone");
DEFINE_bool(is_save_sample, true, "is_save_sample");

namespace qcraft {
namespace control {

namespace {

// Control calibration module main loop running frequency.
constexpr absl::Duration kControlCalibrationInterval = absl::Milliseconds(20);

absl::Status CheckInput(const ControlCalibrationInput& input) {
  if (input.chassis == nullptr) {
    return absl::FailedPreconditionError("The input chassis is nullptr.");
  }
  if (input.pose == nullptr) {
    return absl::FailedPreconditionError("The input pose is nullptr.");
  }
  return absl::OkStatus();
}

}  // namespace

void ControlCalibrationModule::OnInit() {
  // Load vehicle params.
  RunParamsProtoV2 run_params;
  param_manager().GetRunParams(&run_params);
  control_calibration_input_.vehicle_params = &run_params.vehicle_params();
  const std::string car_id = run_params.vehicle_params().car_id();

  std::optional<std::string> directory_name =
      FLAGS_savedir.empty() ? std::nullopt
                            : std::make_optional<std::string>(FLAGS_savedir);

  calibration_tools_ = std::make_unique<calibration::CalibrationTools>(
      directory_name, car_id, FLAGS_method, FLAGS_throttle_deadzone,
      FLAGS_brake_deadzone, FLAGS_is_save_sample);
}

void ControlCalibrationModule::OnSubscribeChannels() {
  Subscribe(&ControlCalibrationModule::OnChassis, this, 25);

  if (IsOnboardMode()) {
    Subscribe(&ControlCalibrationModule::OnPoseProto, this, "pose_proto", 50);
  } else {
    Subscribe(&ControlCalibrationModule::OnPoseProto, this, "sensor_pose", 50);
  }
}

void ControlCalibrationModule::OnChassis(
    std::shared_ptr<const Chassis> chassis) {
  control_calibration_input_.chassis = std::move(chassis);
}
void ControlCalibrationModule::OnPoseProto(
    std::shared_ptr<const PoseProto> pose) {
  control_calibration_input_.pose = std::move(pose);
}

ControlCalibrationModule::~ControlCalibrationModule() {
  calibration_tools_->SaveData();
}

void ControlCalibrationModule::OnSetUpTimers() {
  AddTimerOrDie("control_calibration_main_loop",
                std::bind(&ControlCalibrationModule::MainLoop, this),
                kControlCalibrationInterval,
                /*one_shot=*/false);
}

// Calibration main loop.
void ControlCalibrationModule::MainLoop() {
  vis::vantage::ChartsDataProto chart_data;
  QCHECK_OK(Process(control_calibration_input_, &chart_data));

  QLOG_IF_NOT_OK(WARNING, Publish(chart_data));
}

absl::Status ControlCalibrationModule::Process(
    const ControlCalibrationInput& input,
    vis::vantage::ChartsDataProto* chart_data) {
  RETURN_IF_ERROR(CheckInput(input));

  if (!calibration_tools_->Process(
          input.pose->speed(), input.pose->accel_body().x(),
          input.pose->pitch(), input.chassis->throttle_percentage(),
          input.chassis->brake_percentage(), input.chassis->gear_location(),
          chart_data->add_charts())) {
    return absl::InternalError("[calibration]: process failed!");
  }

  return absl::OkStatus();
}

}  // namespace control
}  // namespace qcraft
