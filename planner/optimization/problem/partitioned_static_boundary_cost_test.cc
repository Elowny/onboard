#include "onboard/planner/optimization/problem/partitioned_static_boundary_cost.h"

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Tob = ThirdOrderBicycle;
using TobCostConvergenceTest = CostConvergenceTest<Tob, kSteps>;
using TobCostEvaluationTest = CostEvaluationTest<Tob, kSteps>;

const std::vector<Tob::StateType> kRacStates = {
    Tob::MakeState(-5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(15.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Tob::StateType> kFacStates = {
    Tob::MakeState(-5.0, -6.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(5.0, -6.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(15.0, -6.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Tob::StateType> kFlcStates = {
    Tob::MakeState(-5.0, -10.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(5.0, -10.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0),
    Tob::MakeState(15.0, -10.0, 0.5 * M_PI, 0.0, 0.0, 0.0, 0.0)};
const std::vector<Tob::ControlType> kControls = {Tob::MakeControl(0.0, 0.0)};

const VehicleGeometryParamsProto kVehicleGeoParams = DefaultVehicleGeometry();
const Segment2d kBoundarySegment(Vec2d(0.0, 0.0), Vec2d(10.0, 0.0));
const std::vector<double> kCascadeBuffers = {10.0, 9.0};
const std::vector<double> kCascadeGains = {1.0, 0.9};
const std::vector<std::string> kSubNames = {"a", "b"};

const double kFrontCornerDistToRac = Hypot(
    kVehicleGeoParams.width() * 0.5, kVehicleGeoParams.front_edge_to_center());
const double kFrontCornerAngleToAxis = std::atan2(
    kVehicleGeoParams.width() * 0.5, kVehicleGeoParams.front_edge_to_center());

TEST(PartitionedStaticBoundaryCostTest, TobSumGTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobEvaluateWithDebugInfoGTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostEvaluationTest::EvaluateWithDebugInfoTest(&cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDGDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kRacStates, kControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDGDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kRacStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDDGDxDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kRacStates, kControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDDGDuDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kRacStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobRacDDGDuDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kRac, /*dist_to_rac=*/0.0,
      /*angle_to_axis=*/0.0, kSubNames, /*using_hessian_approximate=*/false,
      kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFacStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDGDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      kVehicleGeoParams.wheel_base(), /*angle_to_axis=*/0.0, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kFacStates, kControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDGDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      kVehicleGeoParams.wheel_base(), /*angle_to_axis=*/0.0, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kFacStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDDGDxDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      kVehicleGeoParams.wheel_base(), /*angle_to_axis=*/0.0, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFacStates, kControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDDGDuDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      kVehicleGeoParams.wheel_base(), /*angle_to_axis=*/0.0, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFacStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFacDDGDuDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kFac,
      kVehicleGeoParams.wheel_base(), /*angle_to_axis=*/0.0, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFacStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDGDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, kFrontCornerDistToRac,
      kFrontCornerAngleToAxis, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kFlcStates, kControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDGDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, kFrontCornerDistToRac,
      kFrontCornerAngleToAxis, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kFlcStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDDGDxDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, kFrontCornerDistToRac,
      kFrontCornerAngleToAxis, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFlcStates, kControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDDGDuDxTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, kFrontCornerDistToRac,
      kFrontCornerAngleToAxis, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFlcStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(PartitionedStaticBoundaryCostTest, TobFlcDDGDuDuTest) {
  PartitionedStaticBoundaryCost<Tob> cost(
      kSteps, &kVehicleGeoParams, kBoundarySegment,
      PartitionedStaticBoundaryCost<Tob>::Type::kCor, kFrontCornerDistToRac,
      kFrontCornerAngleToAxis, kSubNames,
      /*using_hessian_approximate=*/false, kCascadeBuffers, kCascadeGains);
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kFlcStates, kControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
