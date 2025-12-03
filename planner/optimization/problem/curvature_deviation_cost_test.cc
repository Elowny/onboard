#include "onboard/planner/optimization/problem/curvature_deviation_cost.h"

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

const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 30.0)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

const std::vector<double> kRefPathS = {0.0, 3.0, 8.0, 12.0, 50};
const std::vector<double> kRefKappa = {-0.05, 0.0, 0.05, -0.1, 0.15};
std::vector<double> weights(kSteps, 1.0);

TEST(CurvatureDeviationCostTest, MfobSumGTest) {
  CurvatureDeviationCost<Mfob> cost(kRefPathS, kRefKappa, weights);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(CurvatureDeviationCostTest, MfobDGDxTest) {
  CurvatureDeviationCost<Mfob> cost(kRefPathS, kRefKappa, weights);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDGDuTest) {
  CurvatureDeviationCost<Mfob> cost(kRefPathS, kRefKappa, weights);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDDGDxDxTest) {
  CurvatureDeviationCost<Mfob> cost(kRefPathS, kRefKappa, weights);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDDGDuDxTest) {
  CurvatureDeviationCost<Mfob> cost(kRefPathS, kRefKappa, weights);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(CurvatureDeviationCostTest, MfobDDGDuDuTest) {
  CurvatureDeviationCost<Mfob> cost(kRefPathS, kRefKappa, weights);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
