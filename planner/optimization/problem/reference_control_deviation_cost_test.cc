#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"

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
const std::vector<Tob::StateType> kTobStates = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> kTobControls = {
    Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
    Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};
const Tob::ControlsType kTobRefUs =
    Tob::ControlsType::Zero(Tob::kControlSize * kSteps);

TEST(ReferenceControlDeviationCostTest, TobSumGTest) {
  ReferenceControlDeviationCost<Tob> cost(kTobRefUs);
  TobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(ReferenceControlDeviationCostTest, TobDGDxTest) {
  ReferenceControlDeviationCost<Tob> cost(kTobRefUs);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDGDuTest) {
  ReferenceControlDeviationCost<Tob> cost(kTobRefUs);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDDGDxDxTest) {
  ReferenceControlDeviationCost<Tob> cost(kTobRefUs);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDDGDuDxTest) {
  ReferenceControlDeviationCost<Tob> cost(kTobRefUs);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, TobDDGDuDuTest) {
  ReferenceControlDeviationCost<Tob> cost(kTobRefUs);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
const std::vector<Mfob::StateType> kMfobStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 0.4)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
const Mfob::ControlsType kMfobRefUs =
    Mfob::ControlsType::Zero(kSteps * Mfob::kControlSize);

TEST(ReferenceControlDeviationCostTest, MfobDGDxTest) {
  ReferenceControlDeviationCost<Mfob> cost(kMfobRefUs);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDGDuTest) {
  ReferenceControlDeviationCost<Mfob> cost(kMfobRefUs);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceControlDeviationCost<Mfob> cost(kMfobRefUs);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceControlDeviationCost<Mfob> cost(kMfobRefUs);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceControlDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceControlDeviationCost<Mfob> cost(kMfobRefUs);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
