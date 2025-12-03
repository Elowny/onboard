#include "onboard/planner/optimization/problem/static_boundary_cost_lite.h"

#include <algorithm>
#include <cmath>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

const std::vector<Vec2d> kPathPoints = {
    {-20.0, 10.0}, {0.0, 15.0}, {20.0, 25.0}};
const std::vector<double> kLMax = {3.0, 4.0, 5.0};
const std::vector<double> kLMin = {-3.0, -4.0, -5.0};
const char kName[] = "StaticBoundaryCostLite";
const double kBuffer = 0.0;

const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(-30.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 5.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 10.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 18.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0),
    Mfob::MakeState(-30.0, 15.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 18.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 20.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 35.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::ControlType> kControls = {Mfob::MakeControl(0.1, 0.1)};

class StaticBoundaryCostLiteTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params_ =
        run_params.vehicle_params().vehicle_geometry_params();
  }

 protected:
  VehicleGeometryParamsProto vehicle_geometry_params_;
};
}  // namespace

TEST_F(StaticBoundaryCostLiteTest, MfobSumGTest) {
  StaticBoundaryCostLite<Mfob> cost(
      kSteps, /*path_frame=*/nullptr, kPathPoints, kLMin, kLMax,
      vehicle_geometry_params_.width() * 0.5 + kBuffer, kName);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST_F(StaticBoundaryCostLiteTest, MfobEvaluateWithDebugInfoGTest) {
  StaticBoundaryCostLite<Mfob> cost(
      kSteps, /*path_frame=*/nullptr, kPathPoints, kLMin, kLMax,
      vehicle_geometry_params_.width() * 0.5 + kBuffer, kName);
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(&cost);
}

TEST_F(StaticBoundaryCostLiteTest, MfobUpdateTest) {
  StaticBoundaryCostLite<Mfob> cost(
      kSteps, /*path_frame=*/nullptr, kPathPoints, kLMin, kLMax,
      vehicle_geometry_params_.width() * 0.5 + kBuffer, kName);
  Mfob::StatesType xs(kSteps * Mfob::kStateSize);
  Mfob::ControlsType us(kSteps * Mfob::kControlSize);
  for (int i = 0; i < kSteps; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, kSteps);
}

TEST_F(StaticBoundaryCostLiteTest, MfobDGDxTest) {
  StaticBoundaryCostLite<Mfob> cost(
      kSteps, /*path_frame=*/nullptr, kPathPoints, kLMin, kLMax,
      vehicle_geometry_params_.width() * 0.5 + kBuffer, kName);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StaticBoundaryCostLiteTest, MfobDDGDxDxTest) {
  StaticBoundaryCostLite<Mfob> cost(
      kSteps, /*path_frame=*/nullptr, kPathPoints, kLMin, kLMax,
      vehicle_geometry_params_.width() * 0.5 + kBuffer, kName);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
