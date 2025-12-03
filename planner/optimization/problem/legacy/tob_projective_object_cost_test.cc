#include "onboard/planner/optimization/problem/legacy/tob_projective_object_cost.h"

#include <memory>

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
using TobProjectiveObjectCost = TobProjectiveObjectCost<Tob>;

class TobProjectiveObjectCostTest
    : public ::testing::TestWithParam<
          typename TobProjectiveObjectCost::Object::Type> {
 public:
  void Initialize(typename TobProjectiveObjectCost::Object::Type type) {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);

    std::vector<typename TobProjectiveObjectCost::Object> objects;
    objects.reserve(kSteps);

    std::vector<Segment2d> lines;
    lines.emplace_back(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0));
    lines.emplace_back(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0));

    for (int k = 0; k < kSteps; ++k) {
      objects.push_back(TobProjectiveObjectCost::Object{
          .lines = lines,
          .dir = Vec2d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0)),
          .ref = Vec2d(7.5, 2.5),
          .buffers = {1.0, 1.2},
          .extent = 2.5 * sqrt(2.0),
          .gains = {1.0, 1.0},
          .type = type,
          .enable = true});
    }
    auto vehicle_geomery =
        run_params.vehicle_params().vehicle_geometry_params();
    cost_ = std::make_unique<TobProjectiveObjectCost>(kSteps, &vehicle_geomery,
                                                      std::move(objects));
  }

 protected:
  std::unique_ptr<TobProjectiveObjectCost> cost_;
};

INSTANTIATE_TEST_SUITE_P(
    TobProjectiveObjectCostTests, TobProjectiveObjectCostTest,
    ::testing::Values(TobProjectiveObjectCost::Object::Type::kRac,
                      TobProjectiveObjectCost::Object::Type::kFac));

using TobCostConvergenceTest = CostConvergenceTest<Tob, kSteps>;
using TobCostEvaluationTest = CostEvaluationTest<Tob, kSteps>;
const std::vector<Tob::StateType> kTobStates = {
    Tob::MakeState(11.5, 6.0, 0.1, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(9.5, 6.0, -0.1, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(3.5, 3.5, 0.05, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(9.5, 1.5, -0.05, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(17.0, 6.0, -0.1, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(3.0, 6.0, 0.15, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(-1.0, 6.5, -0.15, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(8.5, 1.5, -M_PI / 2.0, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> kTobControls = {Tob::MakeControl(0.1, 0.1)};

TEST_P(TobProjectiveObjectCostTest, TobSumGTest) {
  Initialize(GetParam());
  TobCostEvaluationTest::SumForAllStepsTest(cost_.get());
}

TEST_P(TobProjectiveObjectCostTest, DGDxTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobProjectiveObjectCostTest, DGDuTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), cost_.get());
}

TEST_P(TobProjectiveObjectCostTest, DDGDxDxTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobProjectiveObjectCostTest, DDGDuDxTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(TobProjectiveObjectCostTest, DDGDuDuTest) {
  Initialize(GetParam());
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), cost_.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
