#include "onboard/planner/optimization/problem/curvature_cost.h"

#include <vector>

#include "Eigen/Core"

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

constexpr double kCurvatureBuffer = 2.0;
const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 4.0, 0.1, 0.05, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(CurvatureCostTest, MfobSumGTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(CurvatureCostTest, MfobEvaluateWithDebugInfoGTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(&cost);
}

TEST(CurvatureCostTest, MfobDGDxTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDGDuTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDDGDxDxTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDDGDuDxTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureCostTest, MfobDDGDuDuTest) {
  CurvatureCost<Mfob> cost(kCurvatureBuffer, kSteps);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
