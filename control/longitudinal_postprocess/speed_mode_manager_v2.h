#ifndef ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_SPEED_MODE_MANAGER_V2_
#define ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_SPEED_MODE_MANAGER_V2_

#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

// TODO(yangyu): replace google doc below with feishu doc link.
// https://docs.google.com/document/d/1J9ZSnC1SAWxIQ2cXL4RwHffdI_8mR2Cqzz4SgFsWXj0/edit#
class SpeedModeManager {
 public:
  explicit SpeedModeManager(const ControllerConf* control_conf)
      : hysteresis_threshold_(control_conf->hysteresis_zone()) {}

  void UpdateSpeedMode(bool is_auto_drive, bool is_full_stop,
                       double acc_calibration, double acc_idle,
                       ControllerDebugProto* controller_debug_proto);

  SpeedMode speed_mode() const { return speed_mode_; }

 private:
  struct HysteresisZone {
    double upper_limit = 0.0;
    double lower_limit = 0.0;
  };

  HysteresisZone CalHysteresisZone(double acc_idle) const;

  void FillDebugProto(double acc_calibration, double acc_idle,
                      const HysteresisZone& hysteresis_zone,
                      ControllerDebugProto* controller_debug_proto) const;

 private:
  SpeedMode speed_mode_ = SpeedMode::DISABLE;
  double hysteresis_threshold_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_SPEED_MODE_MANAGER_V2_
