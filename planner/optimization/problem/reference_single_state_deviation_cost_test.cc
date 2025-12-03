#include "onboard/planner/optimization/problem/reference_single_state_deviation_cost.h"

#include <cmath>

#include "Eigen/Core"

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;

const std::vector<Mfob::StateType> kMfobStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.5),
    Mfob::MakeState(0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, -9.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 12.2, 0.1, 0.1, 0.05, 12.0)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};
const Mfob::StateType kMfobRefEndState =
    Mfob::MakeState(0.2, 0.1, M_PI * 0.02, 1.3, 0.02, 0.5, 0.02, 4.0);
const std::vector<double> kWeights(kMfobRefEndState.size(), 1.0);
const std::vector<double> kBaseNumbers(kMfobRefEndState.size(), 5.0);
constexpr int kRefIndex = 0;

TEST(ReferenceSingleStateDeviationCostTest, MfobDGDxTest) {
  ReferenceSingleStateDeviationCost<Mfob> cost(kMfobRefEndState, kRefIndex,
                                               kWeights, kBaseNumbers);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceSingleStateDeviationCostTest, MfobDGDuTest) {
  ReferenceSingleStateDeviationCost<Mfob> cost(kMfobRefEndState, kRefIndex,
                                               kWeights, kBaseNumbers);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceSingleStateDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceSingleStateDeviationCost<Mfob> cost(kMfobRefEndState, kRefIndex,
                                               kWeights, kBaseNumbers);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceSingleStateDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceSingleStateDeviationCost<Mfob> cost(kMfobRefEndState, kRefIndex,
                                               kWeights, kBaseNumbers);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceSingleStateDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceSingleStateDeviationCost<Mfob> cost(kMfobRefEndState, kRefIndex,
                                               kWeights, kBaseNumbers);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
