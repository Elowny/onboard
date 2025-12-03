#include "onboard/planner/optimization/problem/legacy/mfob_emeraude_object_cost.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using MfobEmeraudeObjectCost = MfobEmeraudeObjectCost<Mfob>;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

class MfobEmeraudeObjectCostTest
    : public ::testing::TestWithParam<
          typename MfobEmeraudeObjectCost::Object::Type> {
 public:
  void Initialize(typename MfobEmeraudeObjectCost::Object::Type type) {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);

    std::vector<typename MfobEmeraudeObjectCost::Object> objects;
    objects.reserve(kSteps / 2);

    std::vector<Segment2d> lines;
    lines.emplace_back(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0));
    lines.emplace_back(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0));

    for (int k = 0; k < kSteps / 2; ++k) {
      objects.push_back(
          MfobEmeraudeObjectCost::Object{.lines = lines,
                                         .buffers = {{15.0}, {10.0}},
                                         .gains = {1.0},
                                         .ref_x = Vec2d(7.5, 2.5),
                                         .ref_tangent = Vec2d(1.0, 0.0),
                                         .back_offset = -100.0,
                                         .front_offset = 100.0,
                                         .type = type,
                                         .enable = k % 2 == 0 ? true : false});
    }
    auto vehicle_geo = run_params.vehicle_params().vehicle_geometry_params();
    const std::vector<double> cascade_buffers = {1.0, 0.5};
    const std::vector<std::string> sub_names = {"a", "b"};
    cost_ = std::make_unique<MfobEmeraudeObjectCost>(
        &vehicle_geo, std::move(objects), cascade_buffers, sub_names,
        /*using_hessian_approximate=*/false);
  }

 protected:
  std::unique_ptr<MfobEmeraudeObjectCost> cost_;
};
INSTANTIATE_TEST_SUITE_P(
    MfobEmeraudeObjectCostTests, MfobEmeraudeObjectCostTest,
    ::testing::Values(MfobEmeraudeObjectCost::Object::Type::kRac,
                      MfobEmeraudeObjectCost::Object::Type::kFac));
const std::vector<Mfob::StateType> kTobStates = {
    Mfob::MakeState(15.5, 0.5, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(15.0, 10.0, M_PI / 4.0, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(7.5, 6.0, -0.1, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(1.5, 3.5, 0.05, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(8.0, 0.5, -0.05, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(-3.0, 2.0, -0.15, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(100.0, 100.0, -0.15, 0.2, 0.1, 0.1, 0.0, 0.4),
    Mfob::MakeState(8.5, -5.5, -M_PI / 2.0, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kTobControls = {
    Mfob::MakeControl(0.1, 0.1)};

TEST_P(MfobEmeraudeObjectCostTest, MfobSumGTest) {
  Initialize(GetParam());
  MfobCostEvaluationTest::SumForAllStepsTest(cost_.get());
}

TEST_P(MfobEmeraudeObjectCostTest, MfobEvaluateWithDebugInfoGTest) {
  Initialize(GetParam());
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(cost_.get());
}

TEST_P(MfobEmeraudeObjectCostTest, DGDxTest) {
  Initialize(GetParam());
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(MfobEmeraudeObjectCostTest, DGDuTest) {
  Initialize(GetParam());
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), cost_.get());
}

TEST_P(MfobEmeraudeObjectCostTest, DDGDxDxTest) {
  Initialize(GetParam());
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(MfobEmeraudeObjectCostTest, DDGDuDxTest) {
  Initialize(GetParam());
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), cost_.get());
}

TEST_P(MfobEmeraudeObjectCostTest, DDGDuDuTest) {
  Initialize(GetParam());
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), cost_.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
