#include "onboard/planner/optimization/problem/legacy/tob_s_limiting_object_cost.h"

#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Tob = ThirdOrderBicycle;
using TobSLimitingObjectCost = TobSLimitingObjectCost<Tob>;

class TobSLimitingObjectCostTest
    : public ::testing::TestWithParam<
          typename TobSLimitingObjectCost::Object::Type> {
 public:
  void Initialize(typename TobSLimitingObjectCost::Object::Type type) {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);

    std::vector<typename TobSLimitingObjectCost::Object> objects;
    objects.reserve(kSteps);
    for (int k = 0; k < kSteps; ++k) {
      objects.push_back(TobSLimitingObjectCost::Object{.ref = Vec2d(10.0, 0.0),
                                                       .s_ref = 2.0,
                                                       .gain = 1.0,
                                                       .depth = 100.0,
                                                       .standoff = 4.0,
                                                       .type = type,
                                                       .enable = true});
    }
    auto vehicle_geo = run_params.vehicle_params().vehicle_geometry_params();
    cost_ = std::make_unique<TobSLimitingObjectCost>(kSteps, &vehicle_geo,
                                                     std::move(objects));
  }

 protected:
  std::unique_ptr<TobSLimitingObjectCost> cost_;
};

INSTANTIATE_TEST_SUITE_P(
    TobSLimitingObjectCostTests, TobSLimitingObjectCostTest,
    ::testing::Values(TobSLimitingObjectCost::Object::Type::kBackward,
                      TobSLimitingObjectCost::Object::Type::kForward));

using TobCostConvergenceTest = CostConvergenceTest<Tob, kSteps>;
using TobCostEvaluationTest = CostEvaluationTest<Tob, kSteps>;
const std::vector<Tob::StateType> kTobStates = {
    Tob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> kTobControls = {Tob::MakeControl(0.1, 0.0),
                                                    Tob::MakeControl(0.1, 0.1)};

TEST_P(TobSLimitingObjectCostTest, TobSumGTest) {
  Initialize(GetParam());
  TobCostEvaluationTest::SumForAllStepsTest(cost_.get());
}

TEST_P(TobSLimitingObjectCostTest, DGDxTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobSLimitingObjectCostTest, DGDuTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), cost_.get());
}

TEST_P(TobSLimitingObjectCostTest, DDGDxDxTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobSLimitingObjectCostTest, DDGDuDxTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobSLimitingObjectCostTest, DDGDuDuTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), cost_.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
