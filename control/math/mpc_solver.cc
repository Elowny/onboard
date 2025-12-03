#include "onboard/control/math/mpc_solver.h"

// IWYU pragma: no_include <ext/alloc_traits.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <ostream>

#include "Eigen/SparseCore"
#include "absl/status/status.h"
#include "osqp/osqp.h"

#include "onboard/global/car_common.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/qp/osqp_solver.h"

namespace qcraft {
namespace control {

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// discrete linear predictive control solver, with control format
// x(i + 1) = A[i] * x(i) + B * u (i) + W[i]

// Convert mpc problem to QP based problem and solve.
// Add state constraints into consideration

// matrix_a: The system dynamic matrix (time-variant or time-invariant);
// matrix_b: The control matrix;
// matrix_w: The disturbance matrix;
// matrix_q: The cost matrix for control state;
// matrix_r: The cost matrix for input;
// matrix_input_constraint_enable: The matrix enables specific input
// constraints; matrix_input_lower: The lower bound control constraint matrix;
// matrix_input_upper: The upper bound control constraint matrix;
// matrix_n: The cost matrix of terminal state scale;
// matrix_state_constraint_enable: The matrix enables specific state
// constraints; matrix_state_lower: The lower bound state constraint matrix;
// matrix_state_upper: The upper bound state constraint matrix;
// matrix_initial_state: The initial state matrix;
// state_reference: The state reference vector with respect to time step;
// input_reference: The input reference vector with respect to time step;
// output return: optimal mpc output sequence.
absl::StatusOr<std::vector<VecXd>> SolveLinearMPC(
    const std::vector<Matrix>& matrix_a, const std::vector<Matrix>& matrix_b,
    const std::vector<VecXd>& matrix_w, const std::vector<Matrix>& matrix_q,
    const Matrix& matrix_r, const Matrix& matrix_n,
    const Matrix& matrix_input_constraint_enable,
    const std::vector<VecXd>& matrix_input_lower,
    const std::vector<VecXd>& matrix_input_upper,
    const Matrix& matrix_state_constraint_enable,
    const std::vector<VecXd>& matrix_state_lower,
    const std::vector<VecXd>& matrix_state_upper,
    const VecXd& matrix_initial_state,
    const std::vector<VecXd>& state_reference,
    const std::vector<VecXd>& input_reference) {
  SCOPED_QTRACE("SolveLinearMPC");
  const size_t horizon = static_cast<size_t>(state_reference.size());
  const size_t state_num = matrix_a[0].rows();
  const size_t input_num = matrix_b[0].cols();

  // Update augment state reference matrix_t
  VecXd matrix_t = VecXd::Zero(state_num * horizon);
  for (size_t j = 0; j < horizon; ++j) {
    matrix_t.block(j * state_num, 0, state_num, 1) = state_reference[j];
  }

  // Update augment input reference matrix_u
  VecXd matrix_u = VecXd::Zero(input_num * horizon);
  for (size_t j = 0; j < horizon; ++j) {
    matrix_u.block(j * input_num, 0, input_num, 1) = input_reference[j];
  }

  // Initialize augment control_output matrix_v which is used to store
  // solved result;
  VecXd matrix_v = VecXd::Zero(input_num * horizon);

  //  Convert formula: x(i + 1) = A[i] * x(i) + B * u (i) + C[i] to below
  //  X[i] = M[i] + K[i] * U[i] + CC[i]
  // Update augment matrix_m
  Matrix matrix_m = Matrix::Zero(state_num * horizon, 1);
  matrix_m.block(0, 0, state_num, 1) = matrix_a[0] * matrix_initial_state;
  for (size_t i = 1; i < horizon; ++i) {
    matrix_m.block(i * state_num, 0, state_num, 1) =
        matrix_a[i] * matrix_m.block((i - 1) * state_num, 0, state_num, 1);
  }

  // Update augment matrix_k
  Matrix matrix_k = Matrix::Zero(state_num * horizon, input_num * horizon);
  matrix_k.block(0, 0, state_num, input_num) = matrix_b[0];
  for (size_t r = 1; r < horizon; ++r) {
    matrix_k.block(r * state_num, 0, state_num, r * input_num) =
        matrix_a[r] *
        matrix_k.block((r - 1) * state_num, 0, state_num, r * input_num);
    matrix_k.block(r * state_num, r * input_num, state_num, input_num) =
        matrix_b[r];
  }

  // Compute matrix_ww
  Matrix matrix_ww = Matrix::Zero(horizon * state_num, 1);
  matrix_ww.block(0, 0, state_num, 1) = matrix_w[0];
  for (size_t i = 1; i < horizon; ++i) {
    matrix_ww.block(i * state_num, 0, state_num, 1) =
        matrix_a[i] * matrix_ww.block((i - 1) * state_num, 0, state_num, 1) +
        matrix_w[i];
  }

  // Update augment  matrix_qq, matrix_rr
  Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows());
  Matrix matrix_rr = Matrix::Zero(matrix_k.cols(), matrix_k.cols());
  for (size_t i = 0; i < horizon; ++i) {
    matrix_qq.block(i * state_num, i * state_num, state_num, state_num) =
        matrix_q[i];
    matrix_rr.block(i * input_num, i * input_num, input_num, input_num) =
        matrix_r;
  }

  // implement a simplistic terminal cost to suppress oscillation
  matrix_qq.block((horizon - 1) * state_num, (horizon - 1) * state_num,
                  state_num, state_num) =
      matrix_n.transpose() *
      matrix_qq.block((horizon - 1) * state_num, (horizon - 1) * state_num,
                      state_num, state_num) *
      matrix_n;

  // Update augment matrix_state_en, matrix_state_ll, matrix_state_uu
  Matrix matrix_state_en =
      Matrix::Zero(state_num * horizon, state_num * horizon);
  VecXd matrix_state_ll = VecXd::Zero(horizon * state_num);
  VecXd matrix_state_uu = VecXd::Zero(horizon * state_num);
  for (size_t i = 0; i < horizon; ++i) {
    matrix_state_en.block(i * state_num, i * state_num, state_num, state_num) =
        matrix_state_constraint_enable;
    matrix_state_ll.block(i * state_num, 0, state_num, 1) =
        matrix_state_lower[i];
    matrix_state_uu.block(i * state_num, 0, state_num, 1) =
        matrix_state_upper[i];
  }

  // Update augment matrix_inequality_state_constraint_ll,
  // matrix_inequality_state_constraint_uu
  Matrix matrix_inequality_state_constraint_ll =
      Matrix::Zero(horizon * state_num, 1);
  Matrix matrix_inequality_state_constraint_uu =
      Matrix::Zero(horizon * state_num, 1);
  matrix_inequality_state_constraint_ll = matrix_state_en * matrix_k;
  matrix_inequality_state_constraint_uu = -matrix_state_en * matrix_k;

  // Update augment matrix_inequality_state_boundary_ll,
  // matrix_inequality_state_boundary_uu
  VecXd matrix_inequality_state_boundary_ll = VecXd::Zero(horizon * state_num);
  VecXd matrix_inequality_state_boundary_uu = VecXd::Zero(horizon * state_num);
  matrix_inequality_state_boundary_ll =
      matrix_state_ll - matrix_state_en * (matrix_m + matrix_ww);
  matrix_inequality_state_boundary_uu =
      -matrix_state_uu + matrix_state_en * (matrix_m + matrix_ww);

  // Update augment input_ll, matrix_input_uu,
  // matrix_inequality_input_constraint_ll,
  // matrix_inequality_input_constraint_uu
  VecXd matrix_input_ll = VecXd::Zero(horizon * input_num);
  VecXd matrix_input_uu = VecXd::Zero(horizon * input_num);

  Matrix matrix_inequality_input_constraint_ll =
      Matrix::Zero(matrix_input_ll.size(), matrix_input_ll.size());
  Matrix matrix_inequality_input_constraint_uu =
      Matrix::Zero(matrix_input_uu.size(), matrix_input_uu.size());

  for (size_t i = 0; i < horizon; ++i) {
    matrix_input_ll.block(i * input_num, 0, input_num, 1) =
        matrix_input_lower[i];
    matrix_input_uu.block(i * input_num, 0, input_num, 1) =
        matrix_input_upper[i];
    matrix_inequality_input_constraint_ll.block(i * input_num, i * input_num,
                                                input_num, input_num) =
        matrix_input_constraint_enable;
    matrix_inequality_input_constraint_uu.block(i * input_num, i * input_num,
                                                input_num, input_num) =
        -matrix_input_constraint_enable;
  }

  // Update augment matrix_inequality_input_boundary_ll,
  // matrix_inequality_input_boundary_uu
  VecXd matrix_inequality_input_boundary_ll = matrix_input_ll;
  VecXd matrix_inequality_input_boundary_uu = -matrix_input_uu;

  // Update matrix_m1, matrix_m2, convert MPC problem to QP problem
  // m1 = k^T * Q * k + R,   m2 = k^T * Q * (m + cc - t) - R * u
  // t: state reference matrix  u: input reference matrix
  const Matrix matrix_m1 = static_cast<Matrix>(
      matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr);
  const VecXd matrix_m2 = static_cast<Matrix>(
      matrix_k.transpose() * matrix_qq * (matrix_m + matrix_ww - matrix_t) -
      matrix_rr * matrix_u);

  // Format in qp_solver

  //    min_x  : q(x) = 0.5 * x^T * m1 * x  + x^T m2
  //    with respect to:  n1 * x = n2 (equality constraint)
  //                      n3 * x >= n4 (inequality constraint)
  //    where, n1: matrix_equality_constraint
  //           n2: matrix_equality_boundary
  //           n3: matrix_inequality_constraint
  //           n4: matrix_inequality_boundary

  // Process equality constraints (inactive).
  Matrix matrix_equality_constraint = Matrix::Zero(
      matrix_input_ll.size() + matrix_input_uu.size(), matrix_input_ll.size());
  VecXd matrix_equality_boundary =
      VecXd::Zero(matrix_input_ll.size() + matrix_input_uu.size());

  // Process inequality constrtaints
  Matrix matrix_inequality_constraint;
  VecXd matrix_inequality_boundary;

  if (matrix_state_constraint_enable.isZero()) {
    matrix_inequality_constraint =
        Matrix::Zero(matrix_inequality_input_constraint_ll.rows() +
                         matrix_inequality_input_constraint_uu.rows(),
                     matrix_inequality_input_constraint_ll.cols());
    matrix_inequality_constraint << matrix_inequality_input_constraint_ll,
        matrix_inequality_input_constraint_uu;

    matrix_inequality_boundary =
        VecXd::Zero(matrix_inequality_input_boundary_ll.size() +
                    matrix_inequality_input_boundary_uu.size());
    matrix_inequality_boundary << matrix_inequality_input_boundary_ll,
        matrix_inequality_input_boundary_uu;
  } else {
    matrix_inequality_constraint =
        Matrix::Zero(matrix_inequality_input_constraint_ll.rows() +
                         matrix_inequality_input_constraint_uu.rows() +
                         matrix_inequality_state_constraint_ll.rows() +
                         matrix_inequality_state_constraint_uu.rows(),
                     matrix_inequality_input_constraint_ll.cols());
    matrix_inequality_constraint << matrix_inequality_input_constraint_ll,
        matrix_inequality_input_constraint_uu,
        matrix_inequality_state_constraint_ll,
        matrix_inequality_state_constraint_uu;

    matrix_inequality_boundary =
        VecXd::Zero(matrix_inequality_input_boundary_ll.rows() +
                    matrix_inequality_input_boundary_uu.rows() +
                    matrix_inequality_state_boundary_ll.rows() +
                    matrix_inequality_state_boundary_uu.rows());
    matrix_inequality_boundary << matrix_inequality_input_boundary_ll,
        matrix_inequality_input_boundary_uu,
        matrix_inequality_state_boundary_ll,
        matrix_inequality_state_boundary_uu;
  }

  // Osqp solve.
  SCOPED_QTRACE("OspqSolver");
  auto qp_solver = std::make_unique<qcraft::OsqpSolver>(
      matrix_m1.sparseView(), matrix_m2,
      matrix_equality_constraint.sparseView(), matrix_equality_boundary,
      matrix_inequality_constraint.sparseView(), matrix_inequality_boundary);

  // https://osqp.org/docs/interfaces/C.html#_CPPv412OSQPSettings
  auto settings = std::make_unique<OSQPSettings>();
  osqp_set_default_settings(settings.get());

  settings->alpha = 1.0;
  settings->verbose = false;
  // Set "adaptive_rho" FALSE could make OSQP deterministic, but will slow down
  // the calculation.
  settings->adaptive_rho = IsOnboardMode();

  const auto status = qp_solver->Solve(*settings);

  if (!status.ok()) {
    return status;
  }

  matrix_v = qp_solver->x();

  std::vector<VecXd> mpc_output =
      std::vector<VecXd>(horizon, VecXd::Zero(input_num));
  for (size_t i = 0; i < horizon; ++i) {
    mpc_output[i] = matrix_v.block(i * input_num, 0, input_num, 1);
  }
  return mpc_output;
}

absl::StatusOr<std::vector<VecXd>> SolveLinearMPCofTimeInvariantSystem(
    const VecXd& init_state, const StateSpace& discrete_state_space,
    const MpcCost& mpc_cost, const MpcConstraint& mpc_constraint,
    const MpcReference& mpc_reference) {
  SCOPED_QTRACE("SolveLinearMPCofTimeInvariantSystem");
  QCHECK(discrete_state_space.CheckMatrixSize().ok());

  const int horizon = mpc_reference.input_reference.size();
  TimeVaryingDiscreteStateSpace tvd_state_space;
  tvd_state_space.state_space_vector =
      std::vector<StateSpace>(horizon, discrete_state_space);

  return SolveLinearMPCofTimeVaryingSystem(
      init_state, tvd_state_space, mpc_cost, mpc_constraint, mpc_reference);
}

absl::StatusOr<std::vector<VecXd>> SolveLinearMPCofTimeVaryingSystem(
    const VecXd& init_state,
    const TimeVaryingDiscreteStateSpace& tvd_state_space,
    const MpcCost& mpc_cost, const MpcConstraint& mpc_constraint,
    const MpcReference& mpc_reference) {
  SCOPED_QTRACE("SolveLinearMPCofTimeVaryingSystem");
  QCHECK_EQ(init_state.rows(), tvd_state_space.StateSize());
  QCHECK(tvd_state_space.CheckSize().ok());
  const StateSpace& ss_for_check = tvd_state_space.state_space_vector.front();
  QCHECK(mpc_cost.SelfSizeCheck().ok() &&
         mpc_cost.CheckMatrixSize(ss_for_check).ok());
  QCHECK(mpc_constraint.SelfCheck().ok() &&
         mpc_constraint.CheckMatrixSize(ss_for_check).ok());
  QCHECK(mpc_reference.SelfCheck().ok() &&
         mpc_reference.CheckSize(ss_for_check).ok());

  const int horizon = tvd_state_space.Steps();

  std::vector<Matrix> matrix_a;
  std::vector<Matrix> matrix_b;
  std::vector<VecXd> matrix_w;
  matrix_a.reserve(horizon);
  matrix_b.reserve(horizon);
  matrix_w.reserve(horizon);

  for (int i = 0; i < tvd_state_space.Steps(); ++i) {
    matrix_a.push_back(tvd_state_space.state_space_vector[i].A);
    matrix_b.push_back(tvd_state_space.state_space_vector[i].B);
    matrix_w.push_back(tvd_state_space.state_space_vector[i].W);
  }

  return SolveLinearMPC(matrix_a, matrix_b, matrix_w, mpc_cost.Q, mpc_cost.R,
                        mpc_cost.N, mpc_constraint.input_enable,
                        std::vector<VecXd>(horizon, mpc_constraint.input_lower),
                        std::vector<VecXd>(horizon, mpc_constraint.input_upper),
                        mpc_constraint.state_enable,
                        std::vector<VecXd>(horizon, mpc_constraint.state_lower),
                        std::vector<VecXd>(horizon, mpc_constraint.state_upper),
                        init_state, mpc_reference.state_reference,
                        mpc_reference.input_reference);
}

}  // namespace control
}  // namespace qcraft
