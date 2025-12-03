#include "onboard/control/longitudinal_postprocess/speed_mode_manager_v2.h"

namespace qcraft {
namespace control {

void SpeedModeManager::UpdateSpeedMode(
    bool is_auto_drive, bool is_full_stop, double acc_calibration,
    double acc_idle, ControllerDebugProto* controller_debug_proto) {
  const auto hysteresis_zone = CalHysteresisZone(acc_idle);
  if (acc_calibration > hysteresis_zone.upper_limit) {
    speed_mode_ = SpeedMode::ACC_MODE;
  } else if (acc_calibration < hysteresis_zone.lower_limit) {
    speed_mode_ = SpeedMode::DEC_MODE;
  } else {
    speed_mode_ = SpeedMode::IDLE_MODE;
  }

  // Override speed mode when not in auto drive and is full stop.
  if (is_full_stop) {
    speed_mode_ = SpeedMode::DEC_MODE;
  }
  if (!is_auto_drive) {
    speed_mode_ = SpeedMode::IDLE_MODE;
  }

  FillDebugProto(acc_calibration, acc_idle, hysteresis_zone,
                 controller_debug_proto);
}

SpeedModeManager::HysteresisZone SpeedModeManager::CalHysteresisZone(
    double acc_idle) const {
  HysteresisZone hysteresis_zone;
  hysteresis_zone.upper_limit = acc_idle + hysteresis_threshold_;
  hysteresis_zone.lower_limit = acc_idle - hysteresis_threshold_;
  return hysteresis_zone;
}

void SpeedModeManager::FillDebugProto(
    double acc_calibration, double acc_idle,
    const HysteresisZone& hysteresis_zone,
    ControllerDebugProto* controller_debug_proto) const {
  auto* debug = controller_debug_proto->mutable_speed_mode_debug_proto();

  debug->set_input_acc(acc_calibration);
  debug->set_curr_speed_mode(speed_mode_);
  debug->set_acc_bound(acc_idle);
  debug->set_hysteresis_upper_limit(hysteresis_zone.upper_limit);
  debug->set_hysteresis_lower_limit(hysteresis_zone.lower_limit);
}

}  // namespace control
}  // namespace qcraft
