#include "onboard/control/calibration/calibration_tools/calibration_idle.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include "glog/logging.h"

#include "onboard/math/proto/piecewise_linear_function.pb.h"

namespace qcraft {
namespace control {
namespace calibration {

bool CalibrationIdle::Init(std::optional<std::string> /*filename*/,
                           double throttle_deadzone, double brake_deadzone) {
  throttle_deadzone_ = throttle_deadzone;
  brake_deadzone_ = brake_deadzone;

  Reset();

  poly_fit_ = std::make_unique<PolyFit>(kIdlePolyfitOrder);

  LOG(INFO) << "[Calibration] Idle method Init is OK !";
  return true;
}

void CalibrationIdle::Reset() {
  idle_proto_output_.Clear();
  idle_proto_sample_.Clear();

  first_run_ = true;
  flag_record_ = true;
  speed_last_ = 0.0;
}

bool CalibrationIdle::Process(double speed, double acceleration,
                              double throttle, double brake) {
  if (first_run_) {
    first_run_ = false;
    if (std::fabs(speed) > kIdleSpeedMin) {
      flag_record_ = false;
    }
  }

  if (!flag_record_) {
    LOG(ERROR) << "[Calibration] Idle method must stop vehicle!";
    return false;
  }

  if (std::fabs(speed - speed_last_) >= kSpeedInterval &&
      std::fabs(speed) >= kIdleSpeedMin &&
      std::fabs(throttle - throttle_deadzone_) < kApertureDeadZone &&
      std::fabs(brake - brake_deadzone_) < kApertureDeadZone) {
    const auto idle_v_a_tmp = idle_proto_sample_.mutable_idle_v_a_plf();
    idle_v_a_tmp->add_x(speed);
    idle_v_a_tmp->add_y(acceleration);
    speed_last_ = speed;
  }
  idle_proto_output_.Clear();
  UpdateIdleProto();
  return true;
}

void CalibrationIdle::UpdateIdleProto() {
  const auto idle_v_a_tmp = idle_proto_sample_.idle_v_a_plf();
  if (idle_proto_sample_.has_idle_v_a_plf()) {
    std::vector<double> x, y;
    for (int i = 0; i < idle_v_a_tmp.x_size(); i++) {
      x.emplace_back(idle_v_a_tmp.x().Get(i));
      y.emplace_back(idle_v_a_tmp.y().Get(i));
    }
    if (!poly_fit_->ComputerCoff(x, y)) {
      return;
    }
    const auto min_loc = std::min_element(std::begin(x), std::end(x));
    const auto max_loc = std::max_element(std::begin(x), std::end(x));

    double idle_speed = *max_loc;
    if (*max_loc < 0.0) {
      idle_speed = *min_loc;
    }

    double x_speed = 0.0;
    const auto idle_v_a = idle_proto_output_.mutable_idle_v_a_plf();
    while (x_speed <= idle_speed) {
      idle_v_a->add_x(PrecisionTruncation(x_speed, 2));
      idle_v_a->add_y(PrecisionTruncation(poly_fit_->GetPolyVal(x_speed), 3));
      x_speed += kIdleSpeedInterval;
    }
  }
}

CalibrationProto CalibrationIdle::GetSampleProto() {
  return idle_proto_sample_;
}
CalibrationProto CalibrationIdle::GetOutputProto() {
  return idle_proto_output_;
}

}  // namespace calibration
}  // namespace control
}  // namespace qcraft
