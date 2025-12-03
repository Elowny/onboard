#ifndef ONBOARD_CONTROL_CALIBRATION_TOOLS_INTERFACE_H_
#define ONBOARD_CONTROL_CALIBRATION_TOOLS_INTERFACE_H_

#include <string>

#include "gflags/gflags.h"

#include "onboard/control/calibration/calibration_tools/proto/control_calibration.pb.h"

namespace qcraft {
namespace control {
namespace calibration {

// For data sampling.
constexpr double kSpeedInterval = 0.1;     // do nothing, variation < threshold.
constexpr double kApertureDeadZone = 2.0;  // not throttle/brake, in deadzone.
constexpr double kAccInterval = 0.05;  // do nothing, acc variation < threshold.

constexpr double kIdleSpeedMin = 0.2;   // process from 0.2m/s
constexpr double kSlideSpeedMin = 0.2;  // process from 0.2m/s
constexpr double kForceSpeedMin = 3.0;  // process in 3.0 to 15.0m/s
constexpr double kForceSpeedMax = 15.0;

// For data polyfit.
constexpr int kIdlePolyfitOrder = 2;  // polyfit number
constexpr int kSlidePolyfitOrder = 2;
constexpr int kForcePolyfitOrder = 2;

// For data saving.
constexpr double kIdleSpeedInterval = 0.3;
constexpr double kSlideSpeedInterval = 2.5;
constexpr double kForceAccInterval = 0.25;  // or 0.5

class CalibrationInterface {
 public:
  CalibrationInterface() = default;
  virtual ~CalibrationInterface() = default;
  virtual bool Init(std::optional<std::string> filename,
                    double throttle_deadzone, double brake_deadzone) = 0;
  virtual bool Process(double speed, double acceleration, double throttle,
                       double brake) = 0;
  virtual void Reset() = 0;
  virtual CalibrationProto GetSampleProto() = 0;
  virtual CalibrationProto GetOutputProto() = 0;
};

}  // namespace calibration
}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CALIBRATION_TOOLS_INTERFACE_H_
