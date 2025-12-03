#include "onboard/planner/optimization/problem/segmented_speed_limit_cost.h"

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
const std::vector<Vec2d> kFreeRefPoints = {{0.0, 0.2}, {6.0, 0.0}, {9.0, 0.2}};
const std::vector<double> kFreeSpeedLimits = {8.0, 30.0};
constexpr int kTestFreeIndex = kSteps / 2;

const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(SegmentedSpeedLimitCostTest, MfobSumGTest) {
  SegmentedSpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits,
                                     kFreeRefPoints, kFreeSpeedLimits,
                                     kTestFreeIndex);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDGDxTest) {
  SegmentedSpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits,
                                     kFreeRefPoints, kFreeSpeedLimits,
                                     kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDGDuTest) {
  SegmentedSpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits,
                                     kFreeRefPoints, kFreeSpeedLimits,
                                     kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDDGDxDxTest) {
  SegmentedSpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits,
                                     kFreeRefPoints, kFreeSpeedLimits,
                                     kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDDGDuDxTest) {
  SegmentedSpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits,
                                     kFreeRefPoints, kFreeSpeedLimits,
                                     kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostTest, MfobDDGDuDuTest) {
  SegmentedSpeedLimitCost<Mfob> cost(kSteps, kRefPoints, kSpeedLimits,
                                     kFreeRefPoints, kFreeSpeedLimits,
                                     kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
