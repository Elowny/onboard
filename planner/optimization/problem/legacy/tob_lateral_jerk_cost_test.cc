#include "onboard/planner/optimization/problem/legacy/tob_lateral_jerk_cost.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Core"

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Tob = ThirdOrderBicycle;
using TobLateralJerkCost = TobLateralJerkCost<Tob>;
using TobCostConvergenceTest = CostConvergenceTest<Tob, kSteps>;
using TobCostEvaluationTest = CostEvaluationTest<Tob, kSteps>;

const std::vector<Tob::StateType> kStates = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> kControls = {
    Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
    Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};

TEST(TobLateralJerkCostTest, TobSumGTest) {
  TobLateralJerkCost cost;
  TobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(TobLateralJerkCostTest, DGDxTest) {
  TobLateralJerkCost cost;
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls, TobCostConvergenceTest::GenerateStateVariationBasis(),
      &cost);
}

TEST(TobLateralJerkCostTest, DGDuTest) {
  TobLateralJerkCost cost;
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DDGDxDxTest) {
  TobLateralJerkCost cost;
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls, TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DDGDuDxTest) {
  TobLateralJerkCost cost;
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(TobLateralJerkCostTest, DDGDuDuTest) {
  TobLateralJerkCost cost;
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
