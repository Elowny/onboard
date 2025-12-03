#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"

#include <cmath>

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

constexpr double kAccelerationBuffer = 2.0;
constexpr double kDecelerationBuffer = -4.0;
const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 4.0, 0.0, 0.4),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, -6.0, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(LongitudinalAccelerationCostTest, MfobSumGTest) {
  LongitudinalAccelerationCost<Mfob> cost(kAccelerationBuffer,
                                          kDecelerationBuffer);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(LongitudinalAccelerationCostTest, MfobDGDxTest) {
  LongitudinalAccelerationCost<Mfob> cost(kAccelerationBuffer,
                                          kDecelerationBuffer);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(LongitudinalAccelerationCostTest, MfobDGDuTest) {
  LongitudinalAccelerationCost<Mfob> cost(kAccelerationBuffer,
                                          kDecelerationBuffer);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(LongitudinalAccelerationCostTest, MfobDDGDxDxTest) {
  LongitudinalAccelerationCost<Mfob> cost(kAccelerationBuffer,
                                          kDecelerationBuffer);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(LongitudinalAccelerationCostTest, MfobDDGDuDxTest) {
  LongitudinalAccelerationCost<Mfob> cost(kAccelerationBuffer,
                                          kDecelerationBuffer);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(LongitudinalAccelerationCostTest, MfobDDGDuDuTest) {
  LongitudinalAccelerationCost<Mfob> cost(kAccelerationBuffer,
                                          kDecelerationBuffer);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
