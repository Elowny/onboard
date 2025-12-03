#ifndef ONBOARD_CONTROL_CONTROLLERS_PID_CONTROLLER_H_
#define ONBOARD_CONTROL_CONTROLLERS_PID_CONTROLLER_H_

#include "onboard/control/control_defs.h"

namespace qcraft {
namespace control {

struct PIDConfig {
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;
  double period = kControlInterval;
  double min_value = 0.0;       // out low limit
  double max_value = 0.0;       // out up limit
  double integral_limit = 0.0;  // anti windup integral
};

struct PIDOut {
  double result = 0.0;
  double offset_P = 0.0;
  double offset_I = 0.0;
  double offset_D = 0.0;
};

class PIDControl {
 public:
  PIDControl();
  void SetConfig(const PIDConfig& config);
  void Compute(double error);
  PIDOut GetPIDOut();
  void ResetIntegral();

 private:
  double error_last_;
  PIDConfig config_;
  PIDOut pid_out_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_PID_CONTROLLER_H_
