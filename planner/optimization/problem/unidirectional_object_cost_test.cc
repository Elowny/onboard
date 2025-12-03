#include "onboard/planner/optimization/problem/unidirectional_object_cost.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
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

std::vector<UnidirectionalObjectCost<Mfob>::Object>
GetMfobUnidirectionalObjectCostObjects() {
  std::vector<UnidirectionalObjectCost<Mfob>::Object> mfob_objects;
  mfob_objects.resize(kSteps / 2);
  for (int k = 0; k < kSteps / 2; ++k) {
    const Vec2d direction(1.0, 0.01 * static_cast<double>(k));
    const Vec2d ref(0.01 * static_cast<double>(k),
                    0.01 * static_cast<double>(k));
    mfob_objects[k] =
        UnidirectionalObjectCost<Mfob>::Object{.dir = direction.normalized(),
                                               .ref = {0.0, 0.0},
                                               .lateral_extent = 10.0,
                                               .buffers = {10.0, 5.0},
                                               .gains = {7.5, 2.5},
                                               .enable = true};
  }
  return mfob_objects;
}

const auto kObjects = GetMfobUnidirectionalObjectCostObjects();

const std::vector<double> kDistToRac = {0.0, 2.0};
const std::vector<double> kAngleToAxis = {0.0, 0.1};

const std::vector<Mfob::StateType> kMfobStates = {
    Mfob::MakeState(5.0, 5.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.1, 0.1)};

const std::vector<std::string> kSubNames = {"a", "b"};
const auto kMfobCost = std::make_unique<UnidirectionalObjectCost<Mfob>>(
    kObjects, kDistToRac, kAngleToAxis, kSubNames,
    /*using_hessian_approximate=*/false);

TEST(UnidirectionalObjectCostTest, MfobSumGTest) {
  MfobCostEvaluationTest::SumForAllStepsTest(kMfobCost.get());
}

TEST(UnidirectionalObjectCostTest, MfobDGDxTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(UnidirectionalObjectCostTest, MfobDGDuTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

TEST(UnidirectionalObjectCostTest, MfobDDGDxDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(UnidirectionalObjectCostTest, MfobDDGDuDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(UnidirectionalObjectCostTest, MfobDDGDuDuTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
