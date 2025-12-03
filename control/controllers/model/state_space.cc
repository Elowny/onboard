#include "onboard/control/controllers/model/state_space.h"

#include <ostream>
// IWYU pragma: no_include <algorithm>

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/util.h"

namespace qcraft::control {

namespace {

Eigen::MatrixXd BilinearTransform(double time_step, const Eigen::MatrixXd& a) {
  QCHECK_GT(time_step, 0.0);
  QCHECK_EQ(a.cols(), a.rows());
  QCHECK_GT(a.cols(), 0);

  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(a.cols(), a.cols());

  return (identity + 0.5 * time_step * a) *
         (identity - 0.5 * time_step * a).inverse();
}

}  // namespace

absl::Status StateSpace::CheckMatrixSize() const {
  const int state_size = B.rows();
  const int input_size = B.cols();

  if (state_size <= 0 || input_size <= 0) {
    return absl::InvalidArgumentError("empty input matrix.");
  }

  if (A.cols() != state_size || A.rows() != state_size) {
    return absl::InvalidArgumentError(
        "state matrix is not square or does not match input matrix.");
  }

  if (W.rows() != state_size) {
    return absl::InvalidArgumentError("noise matrix size does not match.");
  }

  return absl::OkStatus();
}

int StateSpace::InputSize() const { return B.cols(); }

int StateSpace::StateSize() const { return B.rows(); }

int TimeVaryingDiscreteStateSpace::InputSize() const {
  if (state_space_vector.empty()) return 0;
  return state_space_vector.begin()->B.cols();
}

int TimeVaryingDiscreteStateSpace::StateSize() const {
  if (state_space_vector.empty()) return 0;
  return state_space_vector.begin()->B.rows();
}

int TimeVaryingDiscreteStateSpace::Steps() const {
  return state_space_vector.size();
}

absl::Status TimeVaryingDiscreteStateSpace::CheckSize() const {
  if (state_space_vector.empty() || InputSize() == 0 || StateSize() == 0) {
    return absl::InvalidArgumentError("empty state space vector.");
  }

  for (int i = 0; i < state_space_vector.size(); ++i) {
    if (state_space_vector[i].B.cols() != InputSize()) {
      return absl::InvalidArgumentError("input size inconsistent.");
    }
    if (state_space_vector[i].B.rows() != StateSize()) {
      return absl::InvalidArgumentError("state size inconsistent.");
    }
    auto matrix_check = state_space_vector[i].CheckMatrixSize();
    if (!matrix_check.ok()) return matrix_check;
  }
  return absl::OkStatus();
}

StateSpace Discretize(double time_step, DiscretizationMethod method,
                      const StateSpace& continuous_state_space) {
  QCHECK_GT(time_step, 0.0);
  QCHECK(continuous_state_space.CheckMatrixSize().ok());

  StateSpace discrete_state_space;
  const int state_size = continuous_state_space.B.rows();

  // Compute Bd and Wd.
  discrete_state_space.B = continuous_state_space.B * time_step;
  discrete_state_space.W = continuous_state_space.W * time_step;

  // Compute Ad.
  const Eigen::MatrixXd identity =
      Eigen::MatrixXd::Identity(state_size, state_size);
  const Eigen::MatrixXd a_t = continuous_state_space.A * time_step;
  const Eigen::MatrixXd b_t = continuous_state_space.B * time_step;
  switch (method) {
    case DiscretizationMethod::kEular:
      discrete_state_space.A = identity + a_t;
      discrete_state_space.B = b_t;
      break;
    case DiscretizationMethod::kBilinear:
      discrete_state_space.A =
          BilinearTransform(time_step, continuous_state_space.A);
      discrete_state_space.B = b_t;
      break;
    case DiscretizationMethod::kSecondOrderTaylorExpansion:
      discrete_state_space.A = identity + a_t + 0.5 * Sqr(a_t);
      discrete_state_space.B = (identity + 0.5 * a_t) * b_t;
      break;
    case DiscretizationMethod::kThirdOrderTaylorExpansion:
      discrete_state_space.A =
          identity + a_t + 0.5 * Sqr(a_t) + 0.1667 * Cube(a_t);
      discrete_state_space.B = (identity + 0.5 * a_t + 0.1667 * Sqr(a_t)) * b_t;
      break;
    default:
      QLOG(FATAL)
          << "No or wrong discretization setting for controller state space.";
  }

  return discrete_state_space;
}

std::vector<Eigen::VectorXd> EvaluateTvdStateSpace(
    const Eigen::VectorXd& init_state,
    const TimeVaryingDiscreteStateSpace& tvd_state_space,
    const std::vector<Eigen::VectorXd>& input_vector) {
  QCHECK_EQ(init_state.rows(), tvd_state_space.StateSize());
  QCHECK_GE(tvd_state_space.Steps(), input_vector.size());
  for (const auto& input : input_vector) {
    QCHECK_EQ(input.rows(), tvd_state_space.InputSize());
  }

  std::vector<Eigen::VectorXd> state_vector;
  state_vector.reserve(input_vector.size());
  Eigen::VectorXd state_update = init_state;
  for (int i = 0; i < input_vector.size(); ++i) {
    const StateSpace& state_space = tvd_state_space.state_space_vector[i];
    state_update = state_space.A * state_update +
                   state_space.B * input_vector[i] + state_space.W;
    state_vector.push_back(state_update);
  }

  return state_vector;
}

}  // namespace qcraft::control
