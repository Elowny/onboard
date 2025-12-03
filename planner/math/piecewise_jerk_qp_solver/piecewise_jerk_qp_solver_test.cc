#include "onboard/planner/math/piecewise_jerk_qp_solver/piecewise_jerk_qp_solver.h"

#include <memory>
#include <ostream>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/lite/logging.h"

namespace qcraft {
namespace {

TEST(PiecewiseJerkQpSolverTest, TestQpSolverWithoutSlack) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;
  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(kElementNum, kDeltaT);
  constexpr double kRefPos = 200.0;

  auto start_time = absl::Now();

  // Add kernel
  constexpr double kXW = 1.0;
  constexpr double kDxW = 0.0001;
  constexpr double kDdxW = 10.0;
  constexpr double kDddxW = 10.0;
  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel</*order=*/0>(i, kRefPos, kXW);
  }
  solver->AddFirstOrderDerivativeKernel(kDxW);
  solver->AddSecondOrderDerivativeKernel(kDdxW);
  solver->AddThirdOrderDerivativeKernel(kDddxW);

  // Add equality constraints
  std::array<double, 3> x_init = {0.0, 0.0, 0.0};
  solver->SetInitConditions(x_init);

  // Add inequality constraints
  for (int i = 0; i < 50; ++i) {
    constexpr double kCoeff = 1.0;
    // s constraint
    constexpr double kXLowerBound = 0.0;
    constexpr double kXUpperBound = 20.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kXLowerBound,
                                            kXUpperBound);
    // v constraint
    constexpr double kDxLowerBound = 0.0;
    constexpr double kDxUpperBound = 4.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/1, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kDxLowerBound,
                                            kDxUpperBound);
    // a constraint
    constexpr double kDdxLowerBound = -2.0;
    constexpr double kDdxUpperBound = 2.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/2, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kDdxLowerBound,
                                            kDdxUpperBound);
  }

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.06) {
      auto res = solver->Evaluate(t);
      QLOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                 << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }
    LOG(INFO) << "osqp_iter: " << solver->osqp_iter();
    LOG(INFO) << "status_val: " << solver->status_val();
    LOG(INFO) << "TestQpSolverWithoutSlack cost time: "
              << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    LOG(INFO) << "TestQpSolverWithoutSlack failed.";
  }
}

TEST(PiecewiseJerkQpSolverTest, TestSSoftConstraint) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;

  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(
          kElementNum, kDeltaT, /*x_lower_soft_constraint_type_num=*/0,
          /*x_upper_soft_constraint_type_num=*/1,
          /*dx_lower_soft_constraint_type_num=*/0,
          /*dx_upper_soft_constraint_type_num=*/0,
          /*ddx_lower_soft_constraint_type_num=*/0,
          /*ddx_upper_soft_constraint_type_num=*/0,
          /*dddx_lower_soft_constraint_type_num=*/0,
          /*dddx_upper_soft_constraint_type_num=*/0);
  constexpr double kRefPos = 30.0;

  auto start_time = absl::Now();

  // Add kernel
  constexpr double kXW = 0.1;
  constexpr double kDxW = 0.0001;
  constexpr double kDdxW = 1.0;
  constexpr double kDddxW = 10.0;
  constexpr double kXUpperSlackW = 2.0;

  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel</*order=*/0>(i, kRefPos, kXW);
  }
  solver->AddFirstOrderDerivativeKernel(kDxW);
  solver->AddSecondOrderDerivativeKernel(kDdxW);
  solver->AddThirdOrderDerivativeKernel(kDddxW);
  solver->AddZeroOrderSlackVarKernel(BoundType::UPPER_BOUND,
                                     /*slack_indices=*/{0}, {kXUpperSlackW});

  // Add equality constraints
  constexpr double kInitV = 6.0;
  std::array<double, 3> x_init = {0.0, kInitV, 0.0};
  solver->SetInitConditions(x_init);

  // Add inequality constraints
  for (int i = 0; i < 50; ++i) {
    constexpr double kCoeff = 1.0;
    // s constraint
    constexpr double kXUpperBound = 20.0;
    solver->AddZeroOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i},
        /*coeff=*/{kCoeff},
        /*slack_term_index=*/0,
        /*bound_type=*/BoundType::UPPER_BOUND, kXUpperBound);

    // v constraint
    solver->AddNthOrderInequalityConstraint(/*order=*/1, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff},
                                            /*lower_bound=*/0.0,
                                            /*upper_bound=*/10.0);

    // a constraint
    constexpr double kDdxLowerBound = -4.0;
    constexpr double kDdxUpperBound = 4.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/2, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kDdxLowerBound,
                                            kDdxUpperBound);
  }

  solver->AddRegularization(0.0);

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.06) {
      auto res = solver->Evaluate(t);
      LOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }
    LOG(INFO) << "osqp_iter: " << solver->osqp_iter();
    LOG(INFO) << "status_val: " << solver->status_val();
    LOG(INFO) << "TestSSoftConstraint cost time: "
              << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    LOG(INFO) << "TestSSoftConstraint failed.";
  }
}

TEST(PiecewiseJerkQpSolverTest, TestSpeedSoftConstraint) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;

  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(
          kElementNum, kDeltaT, /*x_lower_soft_constraint_type_num=*/0,
          /*x_upper_soft_constraint_type_num=*/0,
          /*dx_lower_soft_constraint_type_num=*/0,
          /*dx_upper_soft_constraint_type_num=*/2,
          /*ddx_lower_soft_constraint_type_num=*/0,
          /*ddx_upper_soft_constraint_type_num=*/0,
          /*dddx_lower_soft_constraint_type_num=*/0,
          /*dddx_upper_soft_constraint_type_num=*/0);
  constexpr double kRefPos = 30.0;

  auto start_time = absl::Now();

  // Add kernel
  constexpr double kXW = 0.1;
  constexpr double kDxW = 0.0001;
  constexpr double kDdxW = 1.0;
  constexpr double kDddxW = 10.0;
  constexpr double kDxUpperSlackW = 10.0;

  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel</*order=*/0>(i, kRefPos, kXW);
  }
  solver->AddFirstOrderDerivativeKernel(kDxW);
  solver->AddSecondOrderDerivativeKernel(kDdxW);
  solver->AddThirdOrderDerivativeKernel(kDddxW);
  solver->AddFirstOrderSlackVarKernel(BoundType::UPPER_BOUND,
                                      /*slack_indices=*/{0, 1},
                                      {kDxUpperSlackW, kDxUpperSlackW});

  // Add equality constraints
  constexpr double kInitV = 6.0;
  constexpr double kInfinity = 1e6;

  std::array<double, 3> x_init = {0.0, kInitV, 0.0};
  solver->SetInitConditions(x_init);

  // Add inequality constraints
  for (int i = 0; i < 50; ++i) {
    constexpr double kCoeff = 1.0;
    // s constraint
    constexpr double kXLowerBound = 0.0;
    constexpr double kXUpperBound = 20.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kXLowerBound,
                                            kXUpperBound);

    // v constraint
    solver->AddNthOrderInequalityConstraint(/*order=*/1, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, 0.0, kInfinity);
    // The first type of speed constraint
    constexpr double kDxUpperBoundType1 = 4.0;
    solver->AddFirstOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{kCoeff}, /*slack_term_index=*/0,
        BoundType::UPPER_BOUND, kDxUpperBoundType1);
    // The second type of speed constraint
    constexpr double kDxUpperBoundType2 = 3.0;
    solver->AddFirstOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{kCoeff}, /*slack_term_index=*/1,
        BoundType::UPPER_BOUND, kDxUpperBoundType2);

    // a constraint
    constexpr double kDdxLowerBound = -4.0;
    constexpr double kDdxUpperBound = 4.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/2, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kDdxLowerBound,
                                            kDdxUpperBound);
  }

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.06) {
      auto res = solver->Evaluate(t);
      LOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }
    LOG(INFO) << "osqp_iter: " << solver->osqp_iter();
    LOG(INFO) << "status_val: " << solver->status_val();
    LOG(INFO) << "TestSpeedSoftConstraint cost time: "
              << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    LOG(INFO) << "TestSpeedSoftConstraint failed.";
  }
}

TEST(PiecewiseJerkQpSolverTest, TestAccelSoftConstraint) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;

  auto start_time = absl::Now();

  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(
          kElementNum, kDeltaT, /*x_lower_soft_constraint_type_num=*/0,
          /*x_upper_soft_constraint_type_num=*/0,
          /*dx_lower_soft_constraint_type_num=*/0,
          /*dx_upper_soft_constraint_type_num=*/0,
          /*ddx_lower_soft_constraint_type_num=*/1,
          /*ddx_upper_soft_constraint_type_num=*/0,
          /*dddx_lower_soft_constraint_type_num=*/0,
          /*dddx_upper_soft_constraint_type_num=*/0);

  // Add kernel
  constexpr double kSpeedWeight = 0.1;
  constexpr double kAccelWeight = 1.0;
  constexpr double kJerkWeight = 1.0;
  constexpr double kAccelSlackWeight = 20.0;

  constexpr double kRefSpeed = 10.0;
  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel</*order=*/1>(i, kRefSpeed,
                                                         kSpeedWeight);
  }
  solver->AddSecondOrderDerivativeKernel(kAccelWeight);
  solver->AddThirdOrderDerivativeKernel(kJerkWeight);
  solver->AddSecondOrderSlackVarKernel(BoundType::LOWER_BOUND,
                                       /*slack_indices=*/{0},
                                       {kAccelSlackWeight});

  // Add equality constraints.
  constexpr double kInitV = 8.0;
  solver->AddNthOrderEqualityConstraint(/*order=*/0, {0}, {1.0}, 0.0);
  solver->AddNthOrderEqualityConstraint(/*order=*/1, {0}, {1.0}, kInitV);
  solver->AddNthOrderEqualityConstraint(/*order=*/1, {kElementNum - 1}, {1.0},
                                        0.0);

  // Add inequality constraints.
  for (int i = 1; i < kElementNum; ++i) {
    constexpr double kCoeff = 1.0;
    // Add s constraint.
    constexpr double kSLowerBound = 0.0;
    constexpr double kSUpperBound = 50.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kSLowerBound,
                                            kSUpperBound);

    // Add speed constraint.
    constexpr double kSpeedLowerBound = 0.0;
    constexpr double kSpeedUpperBound = 20.0;
    solver->AddNthOrderInequalityConstraint(
        /*order=*/1, /*index_list=*/{i},
        /*coeff=*/{kCoeff}, kSpeedLowerBound, kSpeedUpperBound);

    // Add accel constraint.
    constexpr double kMaxAccelLowerBound = -4.0;
    constexpr double kAccelLowerBound = -2.0;
    constexpr double kAccelUpperBound = 2.0;

    /*
     *  Add accelration soft constraint.
     *
     */
    solver->AddSecondOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{kCoeff}, /*slack_term_index=*/0,
        BoundType::LOWER_BOUND, kAccelLowerBound);

    /*
     *  Add accelration hard constraint.
     *
     */
    solver->AddNthOrderInequalityConstraint(
        /*order=*/2, /*index_list=*/{i},
        /*coeff=*/{kCoeff}, kMaxAccelLowerBound, kAccelUpperBound);
  }

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.2) {
      auto res = solver->Evaluate(t);
      LOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }
    LOG(INFO) << "osqp_iter: " << solver->osqp_iter();
    LOG(INFO) << "status_val: " << solver->status_val();
    LOG(INFO) << "TestAccelSoftConstraint cost time: "
              << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    LOG(INFO) << "TestAccelSoftConstraint failed.";
  }
}

TEST(PiecewiseJerkQpSolverTest, TestJerkSoftConstraint) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;

  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(
          kElementNum, kDeltaT, /*x_lower_soft_constraint_type_num=*/0,
          /*x_upper_soft_constraint_type_num=*/0,
          /*dx_lower_soft_constraint_type_num=*/0,
          /*dx_upper_soft_constraint_type_num=*/0,
          /*ddx_lower_soft_constraint_type_num=*/0,
          /*ddx_upper_soft_constraint_type_num=*/0,
          /*dddx_lower_soft_constraint_type_num=*/0,
          /*dddx_upper_soft_constraint_type_num=*/1);
  constexpr double kRefPos = 30.0;

  auto start_time = absl::Now();

  // Add kernel
  constexpr double kXW = 0.1;
  constexpr double kDxW = 0.0001;
  constexpr double kDdxW = 1.0;
  constexpr double kDddxW = 10.0;
  constexpr double kDddXUpperSlackW = 200.0;

  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel</*order=*/0>(i, kRefPos, kXW);
  }
  solver->AddFirstOrderDerivativeKernel(kDxW);
  solver->AddSecondOrderDerivativeKernel(kDdxW);
  solver->AddThirdOrderDerivativeKernel(kDddxW);
  solver->AddThirdOrderSlackVarKernel(BoundType::UPPER_BOUND,
                                      /*slack_indices=*/{0},
                                      {kDddXUpperSlackW});

  // Add equality constraints
  constexpr double kInitV = 6.0;
  std::array<double, 3> x_init = {0.0, kInitV, 0.0};
  solver->SetInitConditions(x_init);

  // Add inequality constraints
  for (int i = 0; i < 50; ++i) {
    constexpr double kCoeff = 1.0;
    // s constraint
    constexpr double kSLowerBound = 0.0;
    constexpr double kSUpperBound = 50.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kSLowerBound,
                                            kSUpperBound);

    // v constraint
    solver->AddNthOrderInequalityConstraint(/*order=*/1, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff},
                                            /*lower_bound=*/0.0,
                                            /*upper_bound=*/10.0);

    // a constraint
    constexpr double kDdxLowerBound = -4.0;
    constexpr double kDdxUpperBound = 4.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/2, /*index_list=*/{i},
                                            /*coeff=*/{kCoeff}, kDdxLowerBound,
                                            kDdxUpperBound);

    // jerk constraint
    solver->AddThirdOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{kCoeff}, /*slack_term_index=*/0,
        BoundType::UPPER_BOUND, /*bound=*/0.4);
  }

  solver->AddRegularization(0.0);

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.06) {
      auto res = solver->Evaluate(t);
      LOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }
    LOG(INFO) << "osqp_iter: " << solver->osqp_iter();
    LOG(INFO) << "status_val: " << solver->status_val();
    LOG(INFO) << "TestJerkSoftConstraint cost time: "
              << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    LOG(INFO) << "TestJerkSoftConstraint failed.";
  }
}

}  // namespace
}  // namespace qcraft
