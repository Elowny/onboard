#include "onboard/control/param/variable_steer_ratio.h"

#include <algorithm>
#include <functional>
#include <ostream>
#include <vector>

#include "boost/range/algorithm/max_element.hpp"
#include "boost/range/algorithm/min_element.hpp"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {
namespace {

PiecewiseLinearFunction<double, double> CreateDefaultSteerAngleToSteerRatioPlf(
    const VehicleDriveParamsProto& vehicle_drive_params) {
  const double max_steer_angle = vehicle_drive_params.max_steer_angle();
  const double steer_ratio = vehicle_drive_params.steer_ratio();
  QCHECK_GT(max_steer_angle, 0.0);
  QCHECK_GT(steer_ratio, 0.0);
  return PiecewiseLinearFunction<double, double>(
      std::vector<double>{-max_steer_angle, max_steer_angle},
      std::vector<double>{steer_ratio, steer_ratio});
}

}  // namespace

VariableSteerRatio::VariableSteerRatio(
    const VehicleDriveParamsProto& vehicle_drive_params,
    std::optional<VehicleInfoProto::VehicleInterface> vehicle_type) {
  if (vehicle_type.has_value()) {
    switch (*vehicle_type) {
      case VehicleInfoProto::MARVELR:
      case VehicleInfoProto::MARVELR_NEW:
        // Refer data:
        // https://qcraft.feishu.cn/sheets/WLx0shUiqhbu8WtPkmwcF6XknZg
        steer_angle_to_steer_ratio_plf_ =
            PiecewiseLinearFunction<double, double>(
                std::vector<double>{d2r(-400.0), d2r(-100.0), d2r(100.0),
                                    d2r(400.0)},
                std::vector<double>{14.5, 15.4, 15.4, 14.5});
        break;
      case VehicleInfoProto::QCRAFTVEHICLE_SUV:
        // Refer data:
        // https://qcraft.feishu.cn/sheets/Tsc7s2hAmhsQHytGxGxcsZx2n6f
        steer_angle_to_steer_ratio_plf_ =
            PiecewiseLinearFunction<double, double>(
                std::vector<double>{d2r(-400.0), d2r(-120.0), d2r(120.0),
                                    d2r(400.0)},
                std::vector<double>{15.2, 16.8, 16.8, 15.2});
        break;
      default:
        steer_angle_to_steer_ratio_plf_ =
            CreateDefaultSteerAngleToSteerRatioPlf(vehicle_drive_params);
        break;
    }
  } else {
    steer_angle_to_steer_ratio_plf_ =
        CreateDefaultSteerAngleToSteerRatioPlf(vehicle_drive_params);
  }

  const auto steer_angle_vec = steer_angle_to_steer_ratio_plf_.x();
  const auto steer_ratio_vec = steer_angle_to_steer_ratio_plf_.y();
  const int size = steer_angle_vec.size();
  std::vector<double> front_wheel_angle_vec;
  front_wheel_angle_vec.reserve(size);
  for (int i = 0; i < size; ++i) {
    const double front_wheel_angle = steer_angle_vec[i] / steer_ratio_vec[i];
    front_wheel_angle_vec.push_back(front_wheel_angle);
  }
  QCHECK_EQ(front_wheel_angle_vec.size(), steer_angle_vec.size());
  front_wheel_angle_to_steer_angle_plf_ =
      PiecewiseLinearFunction<double, double>(front_wheel_angle_vec,
                                              steer_angle_vec);
  steer_angle_to_front_wheel_angle_plf_ =
      PiecewiseLinearFunction<double, double>(steer_angle_vec,
                                              front_wheel_angle_vec);
  // Check input steer angle vec and front wheel angle vec is in increasing
  // order.
  const bool is_increasing_order =
      std::is_sorted(steer_angle_vec.begin(), steer_angle_vec.end(),
                     std::less_equal<double>()) &&
      std::is_sorted(front_wheel_angle_vec.begin(), front_wheel_angle_vec.end(),
                     std::less_equal<double>());
  QCHECK_EQ(is_increasing_order, true);
  // Check steer ratio is reasonable.
  constexpr double kMaxSteerRatioDiff = 3.0;
  const double steer_ratio_diff = *boost::range::max_element(steer_ratio_vec) -
                                  *boost::range::min_element(steer_ratio_vec);
  QCHECK_LT(steer_ratio_diff, kMaxSteerRatioDiff);
}

double VariableSteerRatio::FrontWheelAngleToSteerAngle(
    double front_wheel_angle) const {
  // Note: need to use extrapolation.
  return front_wheel_angle_to_steer_angle_plf_.EvaluateWithExtrapolation(
      front_wheel_angle);
}

double VariableSteerRatio::SteerAngleToFrontWheelAngle(
    double steer_angle) const {
  // Note: need to use extrapolation.
  return steer_angle_to_front_wheel_angle_plf_.EvaluateWithExtrapolation(
      steer_angle);
}

double VariableSteerRatio::steer_ratio(double steer_angle) const {
  // Note: don't use extrapolation.
  return steer_angle_to_steer_ratio_plf_.Evaluate(steer_angle);
}

}  // namespace control
}  // namespace qcraft
