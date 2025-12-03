#include "onboard/control/controllers/model/state_space.h"

// IWYU pragma: no_include <memory>
#include "gtest/gtest.h"

#include "onboard/math/eigen.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kTs = 0.2;

TEST(StateSpaceTest, CheckSizeTest) {
  StateSpace ss;
  ss.A = Eigen::MatrixXd(2, 2);
  ss.B = Eigen::Vector2d(1.0, 2.0);
  ss.W = Eigen::Vector2d(0.1, 0.1);

  EXPECT_TRUE(ss.CheckMatrixSize().ok());

  // Wrong input matrix;
  ss.A = Eigen::MatrixXd(2, 2);
  ss.B = Eigen::MatrixXd(3, 1);
  ss.W = Eigen::Vector2d(0.1, 0.1);
  EXPECT_FALSE(ss.CheckMatrixSize().ok());
  ss.B = Eigen::MatrixXd(1, 1);
  EXPECT_FALSE(ss.CheckMatrixSize().ok());

  // Wrong state matrix;
  ss.A = Eigen::MatrixXd(3, 2);
  ss.B = Eigen::Vector2d(1.0, 2.0);
  ss.W = Eigen::Vector2d(0.1, 0.1);
  EXPECT_FALSE(ss.CheckMatrixSize().ok());
  ss.A = Eigen::MatrixXd(2, 6);
  EXPECT_FALSE(ss.CheckMatrixSize().ok());

  // Wrong noise matrix;
  ss.A = Eigen::MatrixXd(2, 2);
  ss.B = Eigen::Vector2d(1.0, 2.0);
  ss.W = Eigen::MatrixXd(3, 1);
  EXPECT_FALSE(ss.CheckMatrixSize().ok());
}

TEST(StateSpaceTest, DiscreteStateSpace) {
  StateSpace ss;
  ss.A = Eigen::MatrixXd(2, 2);
  ss.A << -1.5, 2.0, 1.0, 0.0;
  ss.B = Eigen::Vector2d(0.5, 0.0);
  ss.W = Eigen::Vector2d(0.1, 0.2);

  EXPECT_TRUE(ss.CheckMatrixSize().ok());

  // Eular method.
  const auto ss_eular = Discretize(kTs, DiscretizationMethod::kEular, ss);

  EXPECT_EQ(ss_eular.A, Eigen::Matrix2d::Identity() + ss.A * kTs);
  EXPECT_EQ(ss_eular.B, ss.B * kTs);
  EXPECT_EQ(ss_eular.W, ss.W * kTs);

  // Bilinear method.
  const auto ss_bilinear = Discretize(kTs, DiscretizationMethod::kBilinear, ss);

  const Eigen::Matrix2d ss_bilinear_a_expected =
      (Eigen::Matrix2d::Identity() + 0.5 * ss.A * kTs) *
      (Eigen::Matrix2d::Identity() - 0.5 * ss.A * kTs).inverse();

  for (int i = 0; i < ss.A.cols() * ss.A.rows(); ++i) {
    EXPECT_NEAR(ss_bilinear.A(i), ss_bilinear_a_expected(i), kEpsilon);
    EXPECT_NEAR(ss_bilinear.A(i), ss_eular.A(i), 0.1);
  }

  EXPECT_EQ(ss_bilinear.B, ss.B * kTs);
  EXPECT_EQ(ss_bilinear.W, ss.W * kTs);

  // SecondOrderTaylorExpansion method.
  const auto ss_taylor =
      Discretize(kTs, DiscretizationMethod::kSecondOrderTaylorExpansion, ss);
  for (int i = 0; i < ss.A.size(); ++i) {
    EXPECT_NEAR(ss_taylor.A(i), ss_bilinear.A(i), 0.02);
  }
}

TEST(TimeVaryingDiscreteStateSpaceTest, CheckTests) {
  StateSpace ss;
  ss.A = Eigen::MatrixXd(2, 2);
  ss.B = Eigen::Vector2d(1.0, 2.0);
  ss.W = Eigen::Vector2d(0.1, 0.1);

  TimeVaryingDiscreteStateSpace tyd_ss;
  constexpr int kSteps = 10;
  tyd_ss.state_space_vector = std::vector<StateSpace>(/*steps*/ kSteps, ss);

  EXPECT_EQ(tyd_ss.InputSize(), 1);
  EXPECT_EQ(tyd_ss.StateSize(), 2);
  EXPECT_EQ(tyd_ss.Steps(), kSteps);
  EXPECT_TRUE(tyd_ss.CheckSize().ok());

  tyd_ss.state_space_vector[5].B = Eigen::Vector3d(1.0, 2.0, 3.0);
  EXPECT_FALSE(tyd_ss.CheckSize().ok());

  tyd_ss.state_space_vector[5].B = Eigen::MatrixXd(2, 2);
  EXPECT_FALSE(tyd_ss.CheckSize().ok());
}

TEST(EvaluateStateSpaceTest, EvaluateTvdStateSpaceTest) {
  // Build a simple state space: y" = u;
  // x = [y, y']^T;
  // x' = [0, 1; 0, 0] x + [0; 1] u +[0; 0];
  StateSpace continuous_ss;
  continuous_ss.A = Eigen::MatrixXd::Zero(2, 2);
  continuous_ss.A(0, 1) = 1.0;
  continuous_ss.B = Eigen::VectorXd::Zero(2);
  continuous_ss.B(1, 0) = 1.0;
  continuous_ss.W = Eigen::VectorXd::Zero(2);

  constexpr double kTimeStep = 0.1;
  const int step = static_cast<int>(1.0 / kTimeStep);

  StateSpace discrete_ss = Discretize(
      /*time_step*/ kTimeStep, DiscretizationMethod::kEular, continuous_ss);
  TimeVaryingDiscreteStateSpace tvd_state_space;
  tvd_state_space.state_space_vector =
      std::vector<StateSpace>(step, discrete_ss);

  // init state.

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);

  EXPECT_EQ(EvaluateTvdStateSpace(
                state, tvd_state_space,
                std::vector<Eigen::VectorXd>{step, Eigen::VectorXd::Zero(1)})
                .back(),
            Eigen::VectorXd::Zero(2));

  Eigen::VectorXd input = Eigen::VectorXd(1);
  input(0) = 1.0;
  state = Eigen::VectorXd::Zero(2);

  auto result = EvaluateTvdStateSpace(
      state, tvd_state_space, std::vector<Eigen::VectorXd>{step, input});

  EXPECT_NEAR(result.back()(0), 0.45, 1e-6);
}

}  // namespace
}  // namespace qcraft::control
