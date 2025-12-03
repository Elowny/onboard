#ifndef ONBOARD_CONTROL_CONTROLLERS_MODEL_SINGLE_TRACK_DYNAMIC_MODEL_H_
#define ONBOARD_CONTROL_CONTROLLERS_MODEL_SINGLE_TRACK_DYNAMIC_MODEL_H_

#include <vector>

#include "onboard/control/controllers/model/state_space.h"
#include "onboard/control/proto/dynamic_model_conf.pb.h"

namespace qcraft::control {

[[maybe_unused]] constexpr int kDynamicModelStateSize =
    6;  // x, y, delta yaw, omega, delta u, psi.
[[maybe_unused]] constexpr int kDynamicModelInputSize = 1;  // d(psi)/dt;

struct DynamicModelMeasurement {
  double psi = 0.0;    // rad.
  double yaw = 0.0;    // rad.
  double roll = 0.0;   // rad.
  double u_0 = 0.0;    // m/s.
  double omega = 0.0;  // rad/s.
  std::vector<double> v;

  int Horizon() const;
};

TimeVaryingDiscreteStateSpace BuildSingleTrackDynamicModel(
    double ts, double wheel_base, double vehicle_length,
    const DynamicModelMeasurement& measure, const DynamicModelConfProto& conf);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_MODEL_SINGLE_TRACK_DYNAMIC_MODEL_H_
