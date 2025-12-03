#ifndef ONBOARD_CONTROL_CONTROLLERS_MODEL_MPC_COST_CONSTRAINT_H_
#define ONBOARD_CONTROL_CONTROLLERS_MODEL_MPC_COST_CONSTRAINT_H_

#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "onboard/control/controllers/model/state_space.h"
#include "onboard/math/eigen.h"

namespace qcraft::control {

struct MpcCost {
  // The cost function form is:
  // J = sum_{i=1}^{n-1}(delta_x[i]^T * Q[i] * delta_x[i] )
  //   + delta_x[n]^T * N^T * Q[n] * N * delta_x[n]
  //   + sum_{i=0}^{n-1}(delta_u[i]^T * R * delta_u[i] );
  MpcCost() = default;
  MpcCost(int horizon, const Eigen::MatrixXd& q_matrix,
          Eigen::MatrixXd r_matrix, Eigen::MatrixXd n_matrix)
      : Q(std::vector<Eigen::MatrixXd>(horizon, q_matrix)),
        R(std::move(r_matrix)),
        N(std::move(n_matrix)) {}

  std::vector<Eigen::MatrixXd> Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd N;

  absl::Status SelfSizeCheck() const;
  absl::Status CheckMatrixSize(const StateSpace& state_space) const;
};

struct MpcConstraint {
  // The enable matrices: square matrix
  // disable: set it as zero matrix;
  // enable: set it as identity matrix;
  Eigen::MatrixXd input_enable;
  Eigen::VectorXd input_lower;
  Eigen::VectorXd input_upper;
  Eigen::MatrixXd state_enable;
  Eigen::VectorXd state_lower;
  Eigen::VectorXd state_upper;

  void Init(int state_size, int input_size);
  absl::Status SelfCheck() const;
  absl::Status CheckMatrixSize(const StateSpace& state_space) const;
};

struct TimeVaryingMpcConstraint {
  std::vector<MpcConstraint> mpc_constraint_vector;

  absl::Status SelfCheck() const;
  absl::Status CheckSize(const TimeVaryingDiscreteStateSpace& tvd_ss) const;
};

struct MpcReference {
  std::vector<Eigen::VectorXd> input_reference;
  std::vector<Eigen::VectorXd> state_reference;

  void Init(int horizon, int state_size, int input_size);
  absl::Status SelfCheck() const;
  absl::Status CheckSize(const StateSpace& state_space) const;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLERS_MODEL_MPC_COST_CONSTRAINT_H_
