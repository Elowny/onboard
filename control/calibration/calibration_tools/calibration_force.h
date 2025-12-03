#ifndef ONBOARD_CONTROL_CALIBRATION_TOOLS_FORCE_H_
#define ONBOARD_CONTROL_CALIBRATION_TOOLS_FORCE_H_

#include <memory>
#include <string>

#include "onboard/control/calibration/calibration_tools/calibration_interface.h"
#include "onboard/control/calibration/calibration_tools/calibration_utils.h"
#include "onboard/control/calibration/calibration_tools/proto/control_calibration.pb.h"

namespace qcraft {
namespace control {
namespace calibration {

class CalibrationForce : public CalibrationInterface {
 public:
  bool Init(std::optional<std::string> filename, double throttle_deadzone,
            double brake_deadzone) override;
  bool Process(double speed, double acceleration, double throttle,
               double brake) override;
  void Reset() override;
  CalibrationProto GetSampleProto() override;
  CalibrationProto GetOutputProto() override;

 private:
  bool LoadIdleProtoFile(const std::string& filename);
  bool LoadSlideProtoFile(const std::string& filename);
  void UpdateThrottleProto();
  void UpdateBrakeProto();

  CalibrationProto force_proto_output_;
  CalibrationProto force_proto_sample_;

  double acc_last_ = 0.0;
  double throttle_deadzone_ = 0.0;
  double brake_deadzone_ = 0.0;

  std::unique_ptr<PolyFit> poly_fit_;
};

}  // namespace calibration
}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CALIBRATION_TOOLS_FORCE_H_
