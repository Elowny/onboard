#include "onboard/planner/optimization/problem/scatter_object_cost.h"

#include <cmath>
#include <memory>

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

const std::vector<Mfob::StateType> kMfobStates = {
    Mfob::MakeState(7.5, 0.5, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(12.0, 7.5, M_PI / 4.0, 1.0, 0.0, 0.1, 0.0, 0.0)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.1, 0.1)};

const std::vector<Vec2d> kPoints = {Vec2d(10.0, 2.5), Vec2d(7.5, 5.0)};

std::vector<std::vector<std::vector<double>>> buffers = {{{15.0, 15.0}},
                                                         {{15.0, 15.0}}};
std::vector<double> gains = {0.01, 0.005};
std::vector<std::string> sub_names = {"a", "b"};

const std::vector<double> kDistToRac = {3.0};
const std::vector<double> kAngleToAxis = {0.1};
const auto kMfobCost = std::make_unique<ScatterStaticObjectCost<Mfob>>(
    kPoints, kDistToRac, kAngleToAxis, buffers, gains, sub_names,
    /*num_objects=*/50,
    /*using_hessian_approximate=*/false);

TEST(ScatterStaticObjectCostTest, MfobSumGTest) {
  MfobCostEvaluationTest::SumForAllStepsTest(kMfobCost.get());
}

TEST(ScatterStaticObjectCostTest, MfobEvaluateWithDebugInfoGTest) {
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(kMfobCost.get());
}

TEST(ScatterStaticObjectCostTest, MfobDGDxTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(ScatterStaticObjectCostTest, MfobDGDuTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

TEST(ScatterStaticObjectCostTest, MfobDDGDxDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(ScatterStaticObjectCostTest, MfobDDGDuDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(ScatterStaticObjectCostTest, MfobDDGDuDuTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
