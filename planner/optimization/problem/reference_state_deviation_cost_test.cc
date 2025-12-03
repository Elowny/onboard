#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"

#include <cmath>

#include "Eigen/Core"

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using Tob = ThirdOrderBicycle;

using TobCostConvergenceTest = CostConvergenceTest<Tob, kSteps>;
using TobCostEvaluationTest = CostEvaluationTest<Tob, kSteps>;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

const std::vector<Tob::StateType> kTobStates = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> kTobControls = {
    Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
    Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
const Tob::StatesType kTobRefXs =
    Tob::StatesType::Zero(kSteps * Tob::kStateSize);

TEST(ReferenceStateDeviationCostTest, TobSumGTest) {
  ReferenceStateDeviationCost<Tob> cost(kTobRefXs);
  TobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(ReferenceStateDeviationCostTest, TobDGDxTest) {
  ReferenceStateDeviationCost<Tob> cost(kTobRefXs);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDGDuTest) {
  ReferenceStateDeviationCost<Tob> cost(kTobRefXs);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDDGDxDxTest) {
  ReferenceStateDeviationCost<Tob> cost(kTobRefXs);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDDGDuDxTest) {
  ReferenceStateDeviationCost<Tob> cost(kTobRefXs);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, TobDDGDuDuTest) {
  ReferenceStateDeviationCost<Tob> cost(kTobRefXs);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

const std::vector<Mfob::StateType> kMfobStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
const Mfob::StatesType kMfobRefXs =
    Mfob::StatesType::Zero(kSteps * Mfob::kStateSize);

TEST(ReferenceStateDeviationCostTest, MfobDGDxTest) {
  ReferenceStateDeviationCost<Mfob> cost(kMfobRefXs);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDGDuTest) {
  ReferenceStateDeviationCost<Mfob> cost(kMfobRefXs);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceStateDeviationCost<Mfob> cost(kMfobRefXs);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceStateDeviationCost<Mfob> cost(kMfobRefXs);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceStateDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceStateDeviationCost<Mfob> cost(kMfobRefXs);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
