#ifndef ONBOARD_CONTROL_CONTROLLERS_MODEL_STATE_SPACE_H_
#define ONBOARD_CONTROL_CONTROLLERS_MODEL_STATE_SPACE_H_

#include <vector>

#include "absl/status/status.h"

#include "onboard/math/eigen.h"

namespace qcraft::control {

struct StateSpace {
  // State space form for both continuous and discrete time forms.
  // x' = A x + B u + W for continuous-time form;
  // x(k+1) = A x(k) + B u(k) + W for discrete-time form;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd W;

  int InputSize() const;
  int StateSize() const;

  absl::Status CheckMatrixSize() const;
};

struct TimeVaryingDiscreteStateSpace {
  // Time varying state space form for both continuous and discrete time forms.
  // x(k+1) = A(k) x(k) + B(k) u(k) + W(k) for discrete-time form;
  std::vector<StateSpace> state_space_vector;

  int InputSize() const;
  int StateSize() const;
  int Steps() const;

  absl::Status CheckSize() const;
};

enum class DiscretizationMethod {
  kEular = 0,
  kBilinear = 1,
  kSecondOrderTaylorExpansion = 2,
  kThirdOrderTaylorExpansion = 3
};

StateSpace Discretize(double time_step, DiscretizationMethod method,
                      const StateSpace& continuous_state_space);

// Tvd: time-varying discrete.
std::vector<Eigen::VectorXd> EvaluateTvdStateSpace(
    const Eigen::VectorXd& init_state,
    const TimeVaryingDiscreteStateSpace& tvd_state_space,
    const std::vector<Eigen::VectorXd>& input_vector);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_MODEL_STATE_SPACE_H_
