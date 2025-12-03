#ifndef ONBOARD_CONTROL_CONTROLLERS_MODEL_TOB_TV_KINEMATIC_MODEL_H_
#define ONBOARD_CONTROL_CONTROLLERS_MODEL_TOB_TV_KINEMATIC_MODEL_H_

#include <vector>

#include "onboard/control/controllers/model/state_space.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"

namespace qcraft::control {

[[maybe_unused]] constexpr int kKinematicModelStateSize =
    4;  // x, y, delta_yaw, kappa;
[[maybe_unused]] constexpr int kKinematicModelInputSize = 1;  // steering_speed;

struct KinematicModelInput {
  double yaw = 0.0;
  std::vector<double> speed;
  std::vector<double> ref_yaw;
  PiecewiseLinearFunctionDoubleProto kappa_decay_ratio;
};

TimeVaryingDiscreteStateSpace TobTvKinematicModelUpdate(
    double time_step, const KinematicModelInput& input);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_MODEL_TOB_TV_KINEMATIC_MODEL_H_
