#include "onboard/planner/optimization/problem/end_heading_cost.h"

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

const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(2.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(6.0, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

const std::vector<Vec2d> kRefPoints = {{0.0, 0.1}, {5.0, 0.0}, {10.0, 0.1}};
const std::vector<double> kGains(kRefPoints.size() - 1, 1.0);
const std::vector<double> kRefThetas = {0.01, -0.01};

TEST(EndHeadingCostTest, MfobSumGTest) {
  EndHeadingCost<Mfob> cost(kSteps, kRefThetas, kRefPoints,
                            /*center_line_helper=*/nullptr, kGains);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(EndHeadingCostTest, MfobDGDxTest) {
  EndHeadingCost<Mfob> cost(kSteps, kRefThetas, kRefPoints,
                            /*center_line_helper=*/nullptr, kGains);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDGDuTest) {
  EndHeadingCost<Mfob> cost(kSteps, kRefThetas, kRefPoints,
                            /*center_line_helper=*/nullptr, kGains);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDDGDxDxTest) {
  EndHeadingCost<Mfob> cost(kSteps, kRefThetas, kRefPoints,
                            /*center_line_helper=*/nullptr, kGains);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDDGDuDxTest) {
  EndHeadingCost<Mfob> cost(kSteps, kRefThetas, kRefPoints,
                            /*center_line_helper=*/nullptr, kGains);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(EndHeadingCostTest, MfobDDGDuDuTest) {
  EndHeadingCost<Mfob> cost(kSteps, kRefThetas, kRefPoints,
                            /*center_line_helper=*/nullptr, kGains);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
