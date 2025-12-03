#include "onboard/control/controllers/model/mpc_cost_constraint.h"

// IWYU pragma: no_include <memory>
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "gtest/gtest.h"

#include "onboard/control/controllers/model/state_space.h"
#include "onboard/math/eigen.h"

namespace qcraft::control {
namespace {

constexpr int kHorizon = 10;
constexpr int kInputSize = 3;
constexpr int kStateSize = 5;

StateSpace BuildStateSpace() {
  StateSpace ss;
  ss.A = Eigen::MatrixXd(kStateSize, kStateSize);
  ss.B = Eigen::MatrixXd(kStateSize, kInputSize);
  ss.W = Eigen::VectorXd(kStateSize);

  return ss;
}

TimeVaryingMpcConstraint BuildTimeVaryingMpcConstraint() {
  TimeVaryingMpcConstraint tv_mpc_constraint;

  MpcConstraint mpc_constraint;
  mpc_constraint.input_enable = Eigen::MatrixXd::Zero(kInputSize, kInputSize);
  mpc_constraint.input_lower = Eigen::VectorXd::Zero(kInputSize);
  mpc_constraint.input_upper = Eigen::VectorXd::Zero(kInputSize);
  mpc_constraint.state_enable = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  mpc_constraint.state_lower = Eigen::VectorXd::Zero(kStateSize);
  mpc_constraint.state_upper = Eigen::VectorXd::Zero(kStateSize);

  tv_mpc_constraint.mpc_constraint_vector =
      std::vector<MpcConstraint>(kHorizon, mpc_constraint);
  return tv_mpc_constraint;
}

TEST(MpcCostTest, SelfSizeCheckTest) {
  MpcCost mpc_cost;
  Eigen::MatrixXd q = Eigen::MatrixXd(5, 5);
  Eigen::MatrixXd r = Eigen::MatrixXd(3, 3);
  Eigen::MatrixXd n = Eigen::MatrixXd(5, 5);

  mpc_cost = MpcCost(kHorizon, q, r, n);
  EXPECT_TRUE(mpc_cost.SelfSizeCheck().ok());

  q = Eigen::MatrixXd(5, 4);
  r = Eigen::MatrixXd(3, 3);
  n = Eigen::MatrixXd(5, 5);
  mpc_cost = MpcCost(kHorizon, q, r, n);
  EXPECT_FALSE(mpc_cost.SelfSizeCheck().ok());

  q = Eigen::MatrixXd(5, 5);
  r = Eigen::MatrixXd(3, 3);
  n = Eigen::MatrixXd(4, 4);
  mpc_cost = MpcCost(kHorizon, q, r, n);
  EXPECT_FALSE(mpc_cost.SelfSizeCheck().ok());

  q = Eigen::MatrixXd(5, 5);
  r = Eigen::MatrixXd(3, 2);
  n = Eigen::MatrixXd(4, 4);
  mpc_cost = MpcCost(kHorizon, q, r, n);
  EXPECT_FALSE(mpc_cost.SelfSizeCheck().ok());
}

TEST(MpcCostTest, CheckMatrixSizeTest) {
  StateSpace ss = BuildStateSpace();

  MpcCost mpc_cost;
  Eigen::MatrixXd q = Eigen::MatrixXd(kStateSize, kStateSize);
  Eigen::MatrixXd r = Eigen::MatrixXd(kInputSize, kInputSize);
  Eigen::MatrixXd n = Eigen::MatrixXd(kStateSize, kStateSize);
  mpc_cost = MpcCost(kHorizon, q, r, n);
  EXPECT_TRUE(mpc_cost.CheckMatrixSize(ss).ok());

  q = Eigen::MatrixXd(kStateSize + 1, kStateSize + 1);
  r = Eigen::MatrixXd(kInputSize, kInputSize);
  n = Eigen::MatrixXd(kStateSize + 1, kStateSize + 1);
  mpc_cost = MpcCost(kHorizon, q, r, n);
  EXPECT_FALSE(mpc_cost.CheckMatrixSize(ss).ok());
}

TEST(MpcConstraint, SelfCheckTest) {
  MpcConstraint mpc_constraint;
  mpc_constraint.Init(kStateSize, kInputSize);
  EXPECT_TRUE(mpc_constraint.SelfCheck().ok());

  mpc_constraint.input_enable =
      Eigen::MatrixXd::Zero(kInputSize + 1, kInputSize);
  EXPECT_FALSE(mpc_constraint.SelfCheck().ok());

  mpc_constraint.input_enable =
      Eigen::MatrixXd::Identity(kInputSize, kInputSize);
  mpc_constraint.state_enable =
      Eigen::MatrixXd::Identity(kStateSize, kStateSize);
  EXPECT_TRUE(mpc_constraint.SelfCheck().ok());

  mpc_constraint.input_enable =
      2.0 * Eigen::MatrixXd::Identity(kInputSize, kInputSize);
  EXPECT_FALSE(mpc_constraint.SelfCheck().ok());

  mpc_constraint.input_enable =
      Eigen::MatrixXd::Identity(kInputSize, kInputSize);
  mpc_constraint.state_enable =
      Eigen::MatrixXd::Identity(kStateSize, kStateSize);
  mpc_constraint.state_enable(0, 0) = 2.0;
  EXPECT_FALSE(mpc_constraint.SelfCheck().ok());

  mpc_constraint.input_lower(0.0) = 1.0;
  mpc_constraint.input_upper(0.0) = 0.0;
  EXPECT_FALSE(mpc_constraint.SelfCheck().ok());
  mpc_constraint.state_upper(0.0) = 0.0;
  EXPECT_FALSE(mpc_constraint.SelfCheck().ok());
}

TEST(MpcConstraintTest, CheckMatrixSizeTest) {
  StateSpace ss = BuildStateSpace();

  MpcConstraint mpc_constraint;
  mpc_constraint.input_enable = Eigen::MatrixXd::Zero(kInputSize, kInputSize);
  mpc_constraint.input_lower = Eigen::VectorXd::Zero(kInputSize);
  mpc_constraint.input_upper = Eigen::VectorXd::Zero(kInputSize);
  mpc_constraint.state_enable = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  mpc_constraint.state_lower = Eigen::VectorXd::Zero(kStateSize);
  mpc_constraint.state_upper = Eigen::VectorXd::Zero(kStateSize);

  EXPECT_TRUE(mpc_constraint.CheckMatrixSize(ss).ok());

  const int input_size_test = kInputSize - 1;
  mpc_constraint.input_enable =
      Eigen::MatrixXd::Zero(input_size_test, input_size_test);
  mpc_constraint.input_lower = Eigen::VectorXd::Zero(input_size_test);
  mpc_constraint.input_upper = Eigen::VectorXd::Zero(input_size_test);
  mpc_constraint.state_enable = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  mpc_constraint.state_lower = Eigen::VectorXd::Zero(kStateSize);
  mpc_constraint.state_upper = Eigen::VectorXd::Zero(kStateSize);
  EXPECT_FALSE(mpc_constraint.CheckMatrixSize(ss).ok());
}

TEST(TimeVaryingMpcConstraintTest, SelfCheckTest) {
  TimeVaryingMpcConstraint tv_mpc_constraint = BuildTimeVaryingMpcConstraint();
  EXPECT_TRUE(tv_mpc_constraint.SelfCheck().ok());

  tv_mpc_constraint.mpc_constraint_vector.begin()->input_enable =
      Eigen::MatrixXd::Zero(kInputSize + 1, kInputSize);
  EXPECT_FALSE(tv_mpc_constraint.SelfCheck().ok());
}

TEST(TimeVaryingMpcConstraintTest, CheckSizeTest) {
  TimeVaryingMpcConstraint tv_mpc_constraint = BuildTimeVaryingMpcConstraint();
  TimeVaryingDiscreteStateSpace tvd_ss;
  tvd_ss.state_space_vector =
      std::vector<StateSpace>(kHorizon, BuildStateSpace());

  EXPECT_TRUE(tv_mpc_constraint.CheckSize(tvd_ss).ok());
}

TEST(MpcReferenceTest, SelfCheckTest) {
  MpcReference mpc_constraint;
  mpc_constraint.Init(kHorizon, kStateSize, kInputSize);
  EXPECT_TRUE(mpc_constraint.SelfCheck().ok());

  mpc_constraint.input_reference.front() =
      Eigen::VectorXd::Zero(kInputSize + 1);
  EXPECT_FALSE(mpc_constraint.SelfCheck().ok());
}

TEST(MpcReferenceTest, CheckSizeTest) {
  MpcReference mpc_constraint;
  mpc_constraint.Init(kHorizon, kStateSize, kInputSize);
  StateSpace ss = BuildStateSpace();

  EXPECT_TRUE(mpc_constraint.CheckSize(ss).ok());
}

}  // namespace

}  // namespace qcraft::control
