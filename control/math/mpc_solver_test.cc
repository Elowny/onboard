#include "onboard/control/math/mpc_solver.h"

#include "gtest/gtest.h"
// IWYU pragma: no_include  <ext/alloc_traits.h>

namespace qcraft {
namespace control {
namespace {

constexpr int kStateSize = 4;
constexpr int kInputSizeA = 2;
constexpr int kInputSizeB = 1;
constexpr int kHorizon = 2;
constexpr double kEpsilon = 1e-2;

StateSpace BuildStateSpaceA() {
  StateSpace dss;
  dss.A = Eigen::MatrixXd(kStateSize, kStateSize);
  dss.A << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  dss.B = Eigen::MatrixXd(kStateSize, kInputSizeA);
  dss.B << 0, 1, 0, 0, 1, 0, 0, 1;
  dss.W = Eigen::VectorXd(kStateSize);
  dss.W << 0, 0, 0, 0.1;

  return dss;
}

StateSpace BuildStateSpaceB() {
  StateSpace dss = BuildStateSpaceA();
  dss.B = Eigen::MatrixXd(kStateSize, kInputSizeB);
  dss.B << 0, 0, 1, 0;

  return dss;
}

MpcCost BuildMpcCostA() {
  Eigen::MatrixXd matrix_q = Eigen::MatrixXd(kStateSize, kStateSize);
  matrix_q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::MatrixXd matrix_r = Eigen::MatrixXd(kInputSizeA, kInputSizeA);
  matrix_r << 1, 0, 0, 1;
  Eigen::MatrixXd matrix_n = Eigen::MatrixXd::Identity(kStateSize, kStateSize);

  MpcCost mpc_cost(kHorizon, matrix_q, matrix_r, matrix_n);

  return mpc_cost;
}

MpcCost BuildMpcCostB() {
  MpcCost mpc_cost = BuildMpcCostA();
  mpc_cost.R = Eigen::MatrixXd::Zero(kInputSizeB, kInputSizeB);
  mpc_cost.R << 1;

  return mpc_cost;
}

MpcConstraint BuildMpcConstraintA() {
  MpcConstraint mpc_constraint;
  mpc_constraint.input_enable =
      Eigen::MatrixXd::Identity(kInputSizeA, kInputSizeA);
  mpc_constraint.input_lower = Eigen::VectorXd(kInputSizeA);
  mpc_constraint.input_lower << -10, -10;
  mpc_constraint.input_upper = Eigen::VectorXd(kInputSizeA);
  mpc_constraint.input_upper << 10, 10;

  mpc_constraint.state_enable = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  mpc_constraint.state_lower = Eigen::VectorXd::Zero(kStateSize);
  mpc_constraint.state_upper = Eigen::VectorXd::Zero(kStateSize);

  return mpc_constraint;
}

MpcConstraint BuildMpcConstraintB() {
  MpcConstraint mpc_constraint = BuildMpcConstraintA();
  mpc_constraint.input_enable =
      Eigen::MatrixXd::Identity(kInputSizeB, kInputSizeB);
  mpc_constraint.input_lower = Eigen::VectorXd(kInputSizeB);
  mpc_constraint.input_lower << -5;
  mpc_constraint.input_upper = Eigen::VectorXd(kInputSizeB);
  mpc_constraint.input_upper << 5;

  return mpc_constraint;
}

MpcReference BuildMpcReferenceA() {
  MpcReference ref;
  Eigen::VectorXd input_ref(kInputSizeA);
  input_ref << 0, 0;
  ref.input_reference = std::vector<Eigen::VectorXd>(kHorizon, input_ref);
  Eigen::VectorXd state_ref(kStateSize);
  state_ref << 200, 200, 0, 0;
  ref.state_reference = std::vector<Eigen::VectorXd>(kHorizon, state_ref);

  return ref;
}

MpcReference BuildMpcReferenceB() {
  MpcReference ref;
  Eigen::VectorXd input_ref(kInputSizeB);
  input_ref << 0;
  ref.input_reference = std::vector<Eigen::VectorXd>(kHorizon, input_ref);
  Eigen::VectorXd state_ref(kStateSize);
  state_ref << 0, 0, 0, 0;
  ref.state_reference = std::vector<Eigen::VectorXd>(kHorizon, state_ref);

  return ref;
}

TEST(MPCTest, SolveLinearMPCofTimeInvariantSystemA) {
  const StateSpace dss = BuildStateSpaceA();
  const MpcCost cost = BuildMpcCostA();
  const MpcConstraint constraint = BuildMpcConstraintA();
  const MpcReference ref = BuildMpcReferenceA();

  Eigen::VectorXd initial_state(kStateSize);
  initial_state << 0, 0, 0, 0;

  const auto mpc_result = SolveLinearMPCofTimeInvariantSystem(
      initial_state, dss, cost, constraint, ref);
  EXPECT_NEAR(constraint.input_upper(0), mpc_result.value()[0](0), kEpsilon);
}

TEST(MPCTest, SolveLinearMPCofTimeInvariantSystemB) {
  const StateSpace dss = BuildStateSpaceB();
  const MpcCost cost = BuildMpcCostB();
  const MpcConstraint constraint = BuildMpcConstraintB();
  const MpcReference ref = BuildMpcReferenceB();

  Eigen::VectorXd initial_state(kStateSize);
  initial_state << 100, 100, 0, 0;

  const auto mpc_result = SolveLinearMPCofTimeInvariantSystem(
      initial_state, dss, cost, constraint, ref);
  EXPECT_NEAR(constraint.input_lower(0), mpc_result.value()[0](0), kEpsilon);
}

}  // namespace
}  // namespace control
}  // namespace qcraft
