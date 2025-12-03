#include "onboard/control/calibration/calibration_tools/calibration_slide.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include "glog/logging.h"

#include "onboard/math/proto/piecewise_linear_function.pb.h"

namespace qcraft {
namespace control {
namespace calibration {

bool CalibrationSlide::Init(std::optional<std::string> /*filename*/,
                            double throttle_deadzone, double brake_deadzone) {
  throttle_deadzone_ = throttle_deadzone;
  brake_deadzone_ = brake_deadzone;

  Reset();

  poly_fit_ = std::make_unique<PolyFit>(kSlidePolyfitOrder);

  LOG(INFO) << "[Calibration] Slide method Init is OK !";
  return true;
}

void CalibrationSlide::Reset() {
  slide_proto_output_.Clear();
  slide_proto_sample_.Clear();
  speed_last_ = 0.0;
  polyfit_num_ = 0;
}

bool CalibrationSlide::Process(double speed, double acceleration,
                               double throttle, double brake) {
  if (acceleration < 0.0 && std::fabs(speed - speed_last_) >= kSpeedInterval &&
      std::fabs(speed) >= kSlideSpeedMin &&
      std::fabs(throttle - throttle_deadzone_) < kApertureDeadZone &&
      std::fabs(brake - brake_deadzone_) < kApertureDeadZone) {
    const auto idle_v_a = slide_proto_sample_.mutable_idle_v_a_plf();
    idle_v_a->add_x(speed);
    idle_v_a->add_y(acceleration);
    speed_last_ = speed;
  }
  slide_proto_output_.Clear();
  UpdateSlideProto();
  return true;
}

void CalibrationSlide::UpdateSlideProto() {
  const auto idle_v_a_tmp = slide_proto_sample_.idle_v_a_plf();
  if (slide_proto_sample_.has_idle_v_a_plf()) {
    std::vector<double> x, y;
    for (int i = 0; i < idle_v_a_tmp.x_size(); ++i) {
      x.emplace_back(idle_v_a_tmp.x().Get(i));
      y.emplace_back(idle_v_a_tmp.y().Get(i));
    }
    if (!poly_fit_->ComputerCoff(x, y)) {
      return;
    }
    const auto min_loc = std::min_element(std::begin(x), std::end(x));
    const auto max_loc = std::max_element(std::begin(x), std::end(x));

    double x_speed = 0.0;
    if (*min_loc > 0.5) x_speed = 2.5;

    const auto idle_v_a = slide_proto_output_.mutable_idle_v_a_plf();
    while (x_speed <= *max_loc) {
      idle_v_a->add_x(PrecisionTruncation(x_speed, 2));
      idle_v_a->add_y(PrecisionTruncation(poly_fit_->GetPolyVal(x_speed), 3));
      x_speed += kSlideSpeedInterval;
    }
  }
}

CalibrationProto CalibrationSlide::GetSampleProto() {
  return slide_proto_sample_;
}
CalibrationProto CalibrationSlide::GetOutputProto() {
  return slide_proto_output_;
}

}  // namespace calibration
}  // namespace control
}  // namespace qcraft
