#ifndef ONBOARD_CONTROL_MATH_MPC_SOLVER_H_
#define ONBOARD_CONTROL_MATH_MPC_SOLVER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/control/controllers/model/mpc_cost_constraint.h"
#include "onboard/control/controllers/model/state_space.h"
#include "onboard/math/eigen.h"

namespace qcraft {
namespace control {

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// TODO(Zhichao): replace the general mpc solver API.
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
    const std::vector<VecXd>& input_reference);

absl::StatusOr<std::vector<VecXd>> SolveLinearMPCofTimeInvariantSystem(
    const VecXd& init_state, const StateSpace& discrete_state_space,
    const MpcCost& mpc_cost, const MpcConstraint& mpc_constraint,
    const MpcReference& mpc_reference);

absl::StatusOr<std::vector<VecXd>> SolveLinearMPCofTimeVaryingSystem(
    const VecXd& init_state,
    const TimeVaryingDiscreteStateSpace& tvd_state_space,
    const MpcCost& mpc_cost, const MpcConstraint& mpc_constraint,
    const MpcReference& mpc_reference);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_MATH_MPC_SOLVER_H_
