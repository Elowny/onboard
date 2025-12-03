#include "onboard/planner/optimization/problem/aggregate_static_object_cost.h"

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
    Mfob::MakeState(15.5, 0.5, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(12.0, 7.5, M_PI / 4.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(7.5, 6.0, -0.1, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(1.5, 3.5, 0.05, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(8.0, 0.5, -0.05, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(3.0, 6.0, 0.15, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(-1.0, 5.0, -0.15, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(8.5, 0.5, -M_PI / 2.0, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.1, 0.1)};

const std::vector<Segment2d> kSegments = {
    Segment2d(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0)),
    Segment2d(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0))};

std::vector<std::vector<std::vector<double>>> buffers = {{{15.0, 15.0}},
                                                         {{15.0, 15.0}}};
std::vector<double> gains = {0.01, 0.005};
std::vector<std::string> sub_names = {"a", "b"};

const std::vector<double> kDistToRac = {3.0};
const std::vector<double> kAngleToAxis = {0.1};
const auto kMfobCost = std::make_unique<AggregateStaticObjectCost<Mfob>>(
    kSegments, kDistToRac, kAngleToAxis, buffers, gains, sub_names,
    /*num_objects=*/50,
    /*using_hessian_approximate=*/false);

TEST(AggregateStaticObjectCostTest, MfobSumGTest) {
  MfobCostEvaluationTest::SumForAllStepsTest(kMfobCost.get());
}

TEST(AggregateStaticObjectCostTest, MfobEvaluateWithDebugInfoGTest) {
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(kMfobCost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDGDxTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDGDuTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDDGDxDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDDGDuDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(AggregateStaticObjectCostTest, MfobDDGDuDuTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
