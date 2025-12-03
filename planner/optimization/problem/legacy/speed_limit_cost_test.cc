#include "onboard/planner/optimization/problem/legacy/speed_limit_cost.h"

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

const std::vector<Vec2d> kRefPoints = {{0.0, 2.0}, {5.0, 0.0}, {10.0, 0.1}};
const std::vector<double> kSpeedLimits = {10.0, 20.0};

const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(SpeedLimitCostTest, MfobSumGTest) {
  SpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(SpeedLimitCostTest, MfobDGDxTest) {
  SpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDGDuTest) {
  SpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDDGDxDxTest) {
  SpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDDGDuDxTest) {
  SpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SpeedLimitCostTest, MfobDDGDuDuTest) {
  SpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
