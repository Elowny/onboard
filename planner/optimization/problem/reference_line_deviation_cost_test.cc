#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"

#include <cmath>

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

constexpr double kPathGain = 1.0;
constexpr double kEndStateGain = 1.0;
const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(2.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(6.0, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

const std::vector<Vec2d> kRefPoints = {{0.0, 0.1}, {5.0, 0.0}, {10.0, 0.1}};
const std::vector<double> kRefLs = {0.0, 0.4};

TEST(ReferenceLineDeviationCostTest, MfobSumGTest) {
  ReferenceLineDeviationCost<Mfob> cost(kSteps, kPathGain, kEndStateGain,
                                        kRefLs, kRefPoints,
                                        /*center_line_helper=*/nullptr);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(ReferenceLineDeviationCostTest, MfobDGDxTest) {
  ReferenceLineDeviationCost<Mfob> cost(kSteps, kPathGain, kEndStateGain,
                                        kRefLs, kRefPoints,
                                        /*center_line_helper=*/nullptr);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceLineDeviationCostTest, MfobDGDuTest) {
  ReferenceLineDeviationCost<Mfob> cost(kSteps, kPathGain, kEndStateGain,
                                        kRefLs, kRefPoints,
                                        /*center_line_helper=*/nullptr);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(ReferenceLineDeviationCostTest, MfobDDGDxDxTest) {
  ReferenceLineDeviationCost<Mfob> cost(kSteps, kPathGain, kEndStateGain,
                                        kRefLs, kRefPoints,
                                        /*center_line_helper=*/nullptr);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceLineDeviationCostTest, MfobDDGDuDxTest) {
  ReferenceLineDeviationCost<Mfob> cost(kSteps, kPathGain, kEndStateGain,
                                        kRefLs, kRefPoints,
                                        /*center_line_helper=*/nullptr);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(ReferenceLineDeviationCostTest, MfobDDGDuDuTest) {
  ReferenceLineDeviationCost<Mfob> cost(kSteps, kPathGain, kEndStateGain,
                                        kRefLs, kRefPoints,
                                        /*center_line_helper=*/nullptr);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
