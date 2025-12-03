#include "onboard/planner/optimization/problem/reference_longitudinal_jerk_deviation_cost.h"

#include <cmath>
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

const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

std::vector<double> ZeroPadding(std::vector<double> raw_array) {
  raw_array.resize(kSteps, 0.0);
  return raw_array;
}
const std::vector<double> kRefJerk = ZeroPadding({-0.5, 0.0, 0.5, 1.0, 1.5});
const std::vector<double> kWeights = ZeroPadding({1.0, 1.0, 1.0, 1.0, 1.0});

TEST(ReferenceLongitudinalJerkDeviationCostTest, MfobSumGTest) {
  ReferenceLongitudinalJerkDeviationCost<Mfob> cost(kRefJerk, kWeights);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(ReferenceLongitudinalJerkDeviationCostTest, MfobDGDxTest) {
  ReferenceLongitudinalJerkDeviationCost<Mfob> cost(kRefJerk, kWeights);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceLongitudinalJerkDeviationCostTest, MfobDGDuTest) {
  ReferenceLongitudinalJerkDeviationCost<Mfob> cost(kRefJerk, kWeights);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceLongitudinalJerkDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceLongitudinalJerkDeviationCost<Mfob> cost(kRefJerk, kWeights);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceLongitudinalJerkDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceLongitudinalJerkDeviationCost<Mfob> cost(kRefJerk, kWeights);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceLongitudinalJerkDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceLongitudinalJerkDeviationCost<Mfob> cost(kRefJerk, kWeights);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
