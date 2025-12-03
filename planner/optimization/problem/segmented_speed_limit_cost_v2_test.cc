#include "onboard/planner/optimization/problem/segmented_speed_limit_cost_v2.h"

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

using SpeedlimitInfoPoint =
    SegmentedSpeedLimitCostV2<Mfob>::SpeedlimitInfoPoint;
using SpeedLimitInfo = SegmentedSpeedLimitCostV2<Mfob>::SpeedLimitInfo;

const std::vector<Vec2d> kRefPoints = {{0.0, 2.0}, {5.0, 0.0}, {10.0, 0.0}};
const std::vector<double> kStationSpeedLimits = {8.0, 20.0, 20.0};
std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits = {
    {{0.75, 16.0}}, {}};
std::vector<std::vector<SpeedlimitInfoPoint>> additional_free_speed_limits = {
    {{0.75, 16.0}}, {}};
const SpeedLimitInfo kSpeedLimits(kStationSpeedLimits, additional_speed_limits);
const SpeedLimitInfo kFreeSpeedLimits(kStationSpeedLimits,
                                      additional_free_speed_limits);

constexpr int kTestFreeIndex = kSteps / 2;
const std::vector<Mfob::StateType> kStates = {
    Mfob::MakeState(1.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.0, 0.4)};
const std::vector<Mfob::ControlType> kControls = {
    Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
    Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

TEST(SegmentedSpeedLimitCostV2Test, MfobSumGTest) {
  SegmentedSpeedLimitCostV2<Mfob> cost(
      kSteps, kRefPoints, /*center_line_query_helper=*/nullptr, kSpeedLimits,
      kFreeSpeedLimits, kTestFreeIndex);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST(SegmentedSpeedLimitCostV2Test, MfobDGDxTest) {
  SegmentedSpeedLimitCostV2<Mfob> cost(
      kSteps, kRefPoints, /*center_line_query_helper=*/nullptr, kSpeedLimits,
      kFreeSpeedLimits, kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostV2Test, MfobDGDuTest) {
  SegmentedSpeedLimitCostV2<Mfob> cost(
      kSteps, kRefPoints, /*center_line_query_helper=*/nullptr, kSpeedLimits,
      kFreeSpeedLimits, kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostV2Test, MfobDDGDxDxTest) {
  SegmentedSpeedLimitCostV2<Mfob> cost(
      kSteps, kRefPoints, /*center_line_query_helper=*/nullptr, kSpeedLimits,
      kFreeSpeedLimits, kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostV2Test, MfobDDGDuDxTest) {
  SegmentedSpeedLimitCostV2<Mfob> cost(
      kSteps, kRefPoints, /*center_line_query_helper=*/nullptr, kSpeedLimits,
      kFreeSpeedLimits, kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST(SegmentedSpeedLimitCostV2Test, MfobDDGDuDuTest) {
  SegmentedSpeedLimitCostV2<Mfob> cost(
      kSteps, kRefPoints, /*center_line_query_helper=*/nullptr, kSpeedLimits,
      kFreeSpeedLimits, kTestFreeIndex);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kStates, kControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(), &cost);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
