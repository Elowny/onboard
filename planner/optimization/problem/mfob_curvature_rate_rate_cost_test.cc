#include "onboard/planner/optimization/problem/mfob_curvature_rate_rate_cost.h"

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

constexpr double kCurvatureRateRateBuffer = 2.0;
const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.2, 0.1, 0.05, 0.05)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(4.0, 0.1), Mfob::MakeControl(-4.0, 0.1)};

TEST(MfobCurvatureRateRateCostTest, SumGTest) {
  MfobCurvatureRateRateCost<Mfob> cost(kCurvatureRateRateBuffer);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(MfobCurvatureRateRateCostTest, DGDxTest) {
  MfobCurvatureRateRateCost<Mfob> cost(kCurvatureRateRateBuffer);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(MfobCurvatureRateRateCostTest, DGDuTest) {
  MfobCurvatureRateRateCost<Mfob> cost(kCurvatureRateRateBuffer);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(MfobCurvatureRateRateCostTest, DDGDxDxTest) {
  MfobCurvatureRateRateCost<Mfob> cost(kCurvatureRateRateBuffer);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(MfobCurvatureRateRateCostTest, DDGDuDxTest) {
  MfobCurvatureRateRateCost<Mfob> cost(kCurvatureRateRateBuffer);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(MfobCurvatureRateRateCostTest, DDGDuDuTest) {
  MfobCurvatureRateRateCost<Mfob> cost(kCurvatureRateRateBuffer);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
