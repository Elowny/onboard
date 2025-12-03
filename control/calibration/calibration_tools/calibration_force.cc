#include "onboard/control/calibration/calibration_tools/calibration_force.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace control {
namespace calibration {

bool CalibrationForce::Init(std::optional<std::string> filename,
                            double throttle_deadzone, double brake_deadzone) {
  poly_fit_ = std::make_unique<PolyFit>(kForcePolyfitOrder);

  throttle_deadzone_ = throttle_deadzone;
  brake_deadzone_ = brake_deadzone;

  Reset();

  if (!filename.has_value()) return false;

  if (!LoadIdleProtoFile(filename.value()) ||
      !LoadSlideProtoFile(filename.value())) {
    LOG(ERROR) << "[Calibration] Load slide data is failed !";
    return false;
  }

  LOG(INFO) << "[Calibration] Throttle-Brake Init is OK !";
  return true;
}

void CalibrationForce::Reset() {
  force_proto_output_.Clear();
  force_proto_sample_.Clear();
  acc_last_ = 0.0;
}

bool CalibrationForce::Process(double speed, double acceleration,
                               double throttle, double brake) {
  const auto idle_v_a_plf =
      PiecewiseLinearFunctionFromProto(force_proto_sample_.idle_v_a_plf());
  const auto acc_idle = idle_v_a_plf.Evaluate(speed);
  const auto acc_pure = acceleration - acc_idle;

  const auto a_throttle = force_proto_sample_.mutable_a_throttle_plf();
  const auto a_brake = force_proto_sample_.mutable_a_brake_plf();

  if (std::fabs(acc_pure - acc_last_) >= kAccInterval &&
      std::fabs(speed) >= kForceSpeedMin &&
      std::fabs(speed) <= kForceSpeedMax) {
    if (acc_pure > 0.0 && throttle > (kApertureDeadZone + throttle_deadzone_)) {
      a_throttle->add_x(acc_pure);
      a_throttle->add_y(throttle);
    }
    if (acc_pure < 0.0 && brake > (kApertureDeadZone + brake_deadzone_)) {
      a_brake->add_x(acc_pure);
      a_brake->add_y(brake);
    }
    acc_last_ = acc_pure;
  }
  force_proto_output_.Clear();
  UpdateThrottleProto();
  UpdateBrakeProto();
  return true;
}

bool CalibrationForce::LoadIdleProtoFile(const std::string& filename) {
  // Match with calibration_tools.cc
  const auto file_idle = absl::StrCat(filename, "_idle.pb.txt");
  file_util::TextFileToProto(file_idle, &force_proto_sample_);
  return true;
}

bool CalibrationForce::LoadSlideProtoFile(const std::string& filename) {
  // Match with calibration_tools.cc
  const auto file_slide = absl::StrCat(filename, "_slide.pb.txt");
  CalibrationProto proto;
  file_util::TextFileToProto(file_slide, &proto);
  if (!proto.has_idle_v_a_plf()) return false;

  auto idle_a_v = force_proto_sample_.mutable_idle_v_a_plf();
  for (int i = 0; i < proto.idle_v_a_plf().x_size(); i++) {
    idle_a_v->add_x(proto.idle_v_a_plf().x().Get(i));
    idle_a_v->add_y(proto.idle_v_a_plf().y().Get(i));
  }

  return true;
}

void CalibrationForce::UpdateThrottleProto() {
  const auto a_throttle_plf = force_proto_sample_.a_throttle_plf();
  if (force_proto_sample_.has_a_throttle_plf()) {
    std::vector<double> x, y;
    for (int i = 0; i < a_throttle_plf.x_size(); i++) {
      x.emplace_back(a_throttle_plf.x().Get(i));
      y.emplace_back(a_throttle_plf.y().Get(i));
    }
    if (!poly_fit_->ComputerCoff(x, y)) {
      return;
    }
    const auto min_loc = std::min_element(std::begin(x), std::end(x));
    const auto max_loc = std::max_element(std::begin(x), std::end(x));

    std::vector<double> list = {std::floor(*min_loc), std::ceil(*min_loc)};
    double x_acc = FindNearst(list, *min_loc);

    while (x_acc <= *max_loc) {
      const auto a_throttle = force_proto_output_.mutable_a_throttle_plf();
      a_throttle->add_x(PrecisionTruncation(x_acc, 2));
      const double throttle_polyval = poly_fit_->GetPolyVal(x_acc);
      double throttle = std::clamp(throttle_polyval, throttle_deadzone_, 100.0);
      a_throttle->add_y(PrecisionTruncation(throttle, 3));
      x_acc += kForceAccInterval;
    }
  }
}

void CalibrationForce::UpdateBrakeProto() {
  const auto a_brake_plf = force_proto_sample_.a_brake_plf();
  if (force_proto_sample_.has_a_brake_plf()) {
    std::vector<double> x, y;
    for (int i = 0; i < a_brake_plf.x_size(); i++) {
      x.emplace_back(a_brake_plf.x().Get(i));
      y.emplace_back(a_brake_plf.y().Get(i));
    }
    if (!poly_fit_->ComputerCoff(x, y)) {
      return;
    }
    const auto min_loc = std::min_element(std::begin(x), std::end(x));
    const auto max_loc = std::max_element(std::begin(x), std::end(x));

    std::vector<double> list = {std::floor(*min_loc), std::ceil(*min_loc)};
    double x_acc = FindNearst(list, *min_loc);

    while (x_acc <= *max_loc) {
      const auto a_brake = force_proto_output_.mutable_a_brake_plf();
      a_brake->add_x(PrecisionTruncation(x_acc, 2));
      const double brake_polyval = poly_fit_->GetPolyVal(x_acc);
      double brake = std::clamp(brake_polyval, brake_deadzone_, 100.0);
      a_brake->add_y(PrecisionTruncation(brake, 3));
      x_acc += kForceAccInterval;
    }
  }
}

CalibrationProto CalibrationForce::GetSampleProto() {
  return force_proto_sample_;
}

CalibrationProto CalibrationForce::GetOutputProto() {
  force_proto_output_.mutable_idle_v_a_plf()->CopyFrom(
      force_proto_sample_.idle_v_a_plf());
  return force_proto_output_;
}

}  // namespace calibration
}  // namespace control
}  // namespace qcraft
