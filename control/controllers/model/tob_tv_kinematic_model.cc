
#include "onboard/control/controllers/model/tob_tv_kinematic_model.h"

#include <cmath>
// IWYU pragma: no_include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/eigen.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"

namespace qcraft::control {

namespace {

constexpr DiscretizationMethod kMethod =
    DiscretizationMethod::kSecondOrderTaylorExpansion;

}  // namespace

TimeVaryingDiscreteStateSpace TobTvKinematicModelUpdate(
    double time_step, const KinematicModelInput& input) {
  QCHECK_EQ(input.speed.size(), input.ref_yaw.size());

  TimeVaryingDiscreteStateSpace tvd_ss;
  tvd_ss.state_space_vector.reserve(input.speed.size());

  const double cos_yaw = std::cos(input.yaw);
  const double sin_yaw = std::sin(input.yaw);

  for (unsigned int i = 0; i < input.speed.size(); ++i) {
    StateSpace continuous_state_space;
    continuous_state_space.A = Eigen::MatrixXd::Zero(kKinematicModelStateSize,
                                                     kKinematicModelStateSize);
    continuous_state_space.A(0, 2) = -input.speed[i] * sin_yaw;
    continuous_state_space.A(1, 2) = input.speed[i] * cos_yaw;
    if (input.kappa_decay_ratio.x().empty()) {
      continuous_state_space.A(2, 3) = input.speed[i];
    } else {
      const auto kappa_decay_ratio_plf =
          PiecewiseLinearFunctionFromProto(input.kappa_decay_ratio);
      continuous_state_space.A(2, 3) =
          input.speed[i] * kappa_decay_ratio_plf(input.speed[i]);
    }

    continuous_state_space.B = Eigen::MatrixXd::Zero(kKinematicModelStateSize,
                                                     kKinematicModelInputSize);
    continuous_state_space.B(3, 0) = 1.0;

    const double cos_delta_yaw_ref =
        fast_math::Cos(NormalizeAngle(input.ref_yaw[i] - input.ref_yaw[0]));
    continuous_state_space.W = Eigen::MatrixXd::Zero(kKinematicModelStateSize,
                                                     kKinematicModelInputSize);
    continuous_state_space.W(0, 0) =
        input.speed[i] * cos_yaw * cos_delta_yaw_ref;
    continuous_state_space.W(1, 0) =
        input.speed[i] * sin_yaw * cos_delta_yaw_ref;

    tvd_ss.state_space_vector.push_back(
        Discretize(time_step, kMethod, continuous_state_space));
  }

  return tvd_ss;
}

}  // namespace qcraft::control
