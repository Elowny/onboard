#include "onboard/control/controllers/model/mpc_cost_constraint.h"

// IWYU pragma: no_include <memory>
// IWYU pragma: no_include <ext/alloc_traits.h>
#include <string>

#include "absl/strings/str_cat.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"

namespace qcraft::control {

namespace {

bool CheckEnableMatrixValueIsGood(const Eigen::MatrixXd& enable_matrix) {
  for (int r = 0; r < enable_matrix.rows(); ++r) {
    for (int c = 0; c < enable_matrix.cols(); ++c) {
      if (r != c) {
        if (enable_matrix(r, c) != 0.0) return false;
      } else {
        if (enable_matrix(r, c) != 0.0 && enable_matrix(r, c) != 1.0) {
          return false;
        }
      }
    }
  }
  return true;
}

bool CheckLimitsIsGood(const Eigen::VectorXd& lower_limit,
                       const Eigen::VectorXd& upper_limit) {
  QCHECK_EQ(lower_limit.size(), upper_limit.size());
  for (int i = 0; i < lower_limit.size(); ++i) {
    if (lower_limit(i) > upper_limit(i)) return false;
  }
  return true;
}

}  // namespace

absl::Status MpcCost::SelfSizeCheck() const {
  if (Q.empty()) return absl::InternalError("No Q matrix.");

  const int q_cols = Q.front().cols();
  const int q_rows = Q.front().rows();
  for (const Eigen::MatrixXd& q : Q) {
    if (q.cols() != q_cols || q.rows() != q_rows) {
      return absl::InternalError("cost matrix vector size is not consistent.");
    }
  }

  if (q_cols != q_rows || R.cols() != R.rows() || N.cols() != N.rows()) {
    return absl::InternalError("cost matrix is not square.");
  }

  if (q_cols != N.cols()) {
    return absl::InternalError("cost matrix does not match terminal matrix.");
  }
  return absl::OkStatus();
}

absl::Status MpcCost::CheckMatrixSize(const StateSpace& state_space) const {
  QCHECK(state_space.CheckMatrixSize().ok());
  if (Q.front().cols() != state_space.StateSize() ||
      R.cols() != state_space.InputSize()) {
    return absl::InvalidArgumentError(
        "Cost function does not match state space.");
  }

  return absl::OkStatus();
}

void MpcConstraint::Init(int state_size, int input_size) {
  input_enable = Eigen::MatrixXd::Zero(input_size, input_size);
  input_lower = Eigen::VectorXd::Zero(input_size);
  input_upper = Eigen::VectorXd::Zero(input_size);
  state_enable = Eigen::MatrixXd::Zero(state_size, state_size);
  state_lower = Eigen::VectorXd::Zero(state_size);
  state_upper = Eigen::VectorXd::Zero(state_size);
}

absl::Status MpcConstraint::SelfCheck() const {
  const int input_size = input_lower.rows();
  const int state_size = state_lower.rows();

  if (input_enable.rows() != input_size || input_enable.cols() != input_size ||
      input_lower.rows() != input_size || input_upper.rows() != input_size ||
      state_enable.rows() != state_size || state_enable.cols() != state_size ||
      state_lower.rows() != state_size || state_upper.rows() != state_size) {
    auto error_msg = absl::StrCat(
        "input_enable size: ", input_enable.rows(), " x ", input_enable.cols(),
        ";\n", "input lower and upper size: ", input_lower.rows(), ", ",
        input_upper.rows(), ";\n", "state_enable size: ", state_enable.rows(),
        " x ", state_enable.cols(), ";\n",
        "state lower and upper size: ", state_lower.rows(), ", ",
        state_upper.rows());

    return absl::InternalError(
        absl::StrCat("MPC constraint matrix size does not match.", error_msg));
  }

  if (!CheckEnableMatrixValueIsGood(input_enable)) {
    return absl::InternalError("Invalid input enable matrix.");
  }

  if (!CheckEnableMatrixValueIsGood(state_enable)) {
    return absl::InternalError("Invalid state enable matrix.");
  }

  if (!CheckLimitsIsGood(input_lower, input_upper)) {
    return absl::InternalError("Invalid input limit setting.");
  }

  if (!CheckLimitsIsGood(state_lower, state_upper)) {
    return absl::InternalError("Invalid state limit setting.");
  }

  return absl::OkStatus();
}

absl::Status MpcConstraint::CheckMatrixSize(
    const StateSpace& state_space) const {
  if (input_enable.rows() != state_space.InputSize()) {
    return absl::InvalidArgumentError("Input matrix size does not match.");
  }

  if (state_enable.rows() != state_space.StateSize()) {
    return absl::InvalidArgumentError("State matrix size does not match.");
  }

  return absl::OkStatus();
}

absl::Status TimeVaryingMpcConstraint::SelfCheck() const {
  for (int i = 0; i < mpc_constraint_vector.size(); ++i) {
    auto check = mpc_constraint_vector[i].SelfCheck();
    if (!check.ok()) return check;
  }

  return absl::OkStatus();
}

absl::Status TimeVaryingMpcConstraint::CheckSize(
    const TimeVaryingDiscreteStateSpace& tvd_ss) const {
  QCHECK(tvd_ss.CheckSize().ok());

  for (int i = 0; i < mpc_constraint_vector.size(); ++i) {
    auto check =
        mpc_constraint_vector[i].CheckMatrixSize(tvd_ss.state_space_vector[i]);
    if (!check.ok()) return check;
  }

  return absl::OkStatus();
}

void MpcReference::Init(int horizon, int state_size, int input_size) {
  input_reference =
      std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(input_size));
  state_reference =
      std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(state_size));
}

absl::Status MpcReference::SelfCheck() const {
  if (input_reference.size() != state_reference.size() ||
      input_reference.empty()) {
    return absl::InternalError("MPC reference horizon does not match.");
  }

  const int input_size = input_reference[0].rows();
  const int state_size = state_reference[0].rows();
  for (int i = 1; i < input_reference.size(); ++i) {
    if (input_reference[i].rows() != input_size ||
        state_reference[i].rows() != state_size)
      return absl::InternalError("MPC reference size is inconsistent.");
  }

  return absl::OkStatus();
}

absl::Status MpcReference::CheckSize(const StateSpace& state_space) const {
  if (input_reference[0].rows() != state_space.InputSize()) {
    return absl::InvalidArgumentError(
        "MPC input reference size does not match the state space.");
  }

  if (state_reference[0].rows() != state_space.StateSize()) {
    return absl::InvalidArgumentError(
        "MPC state reference size does not match the state space.");
  }

  return absl::OkStatus();
}

}  // namespace qcraft::control
