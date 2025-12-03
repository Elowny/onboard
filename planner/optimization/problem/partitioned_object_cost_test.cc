#include "onboard/planner/optimization/problem/partitioned_object_cost.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

std::vector<std::vector<PartitionedObjectCost<Mfob>::Object>> GetMfobObjects() {
  std::vector<std::vector<PartitionedObjectCost<Mfob>::Object>> mfob_objects;
  mfob_objects.resize(kSteps / 2);
  std::vector<Segment2d> lines = {Segment2d(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0)),
                                  Segment2d(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0))};
  for (int k = 0; k < kSteps / 2; ++k) {
    mfob_objects[k].push_back(PartitionedObjectCost<Mfob>::Object{
        .lines = lines,
        .buffers = {15.0, 15.0},
        .gains = {1.0, 0.5},
        .ref_x = {7.5, 2.5},
        .offset = 100.0,
        .ref_tangent = {1.0, 0.0},
        .enable = k % 2 == 0 ? true : false});
  }
  return mfob_objects;
}

std::vector<PartitionedObjectCost<Mfob>::filter> GetMfobObjectsFilterBoxs() {
  std::vector<PartitionedObjectCost<Mfob>::filter> filters;
  filters.reserve(kSteps / 2);
  for (int k = 0; k < kSteps / 2; ++k) {
    filters.push_back(PartitionedObjectCost<Mfob>::filter{
        Vec2d(7.5, 2.5), Vec2d(1.0, 0.0), 100.0});
  }
  return filters;
}

const auto kMfobObjects = GetMfobObjects();
const auto kMfobFilters = GetMfobObjectsFilterBoxs();

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

const std::vector<double> kDistToRac = {3.0};
const std::vector<double> kAngleToAxis = {0.1};

const std::vector<double> kCascadeBuffers = {0.01, 0.005};
const std::vector<std::string> kSubNames = {"a", "b"};
const auto kMfobCost = std::make_unique<PartitionedObjectCost<Mfob>>(
    kMfobObjects, kMfobFilters, kDistToRac, kAngleToAxis, kCascadeBuffers,
    /*av_model_helper=*/nullptr, kSubNames,
    /*using_hessian_approximate=*/false);

TEST(PartitionedObjectCostTest, MfobSumGTest) {
  MfobCostEvaluationTest::SumForAllStepsTest(kMfobCost.get());
}

TEST(PartitionedObjectCostTest, MfobEvaluateWithDebugInfoGTest) {
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(kMfobCost.get());
}

TEST(PartitionedObjectCostTest, MfobDGDxTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(PartitionedObjectCostTest, MfobDGDuTest) {
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

TEST(PartitionedObjectCostTest, MfobDDGDxDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(PartitionedObjectCostTest, MfobDDGDuDxTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), kMfobCost.get());
}

TEST(PartitionedObjectCostTest, MfobDDGDuDuTest) {
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobStates, kMfobControls,
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      MfobCostConvergenceTest::GenerateControlVariationBasis(),
      kMfobCost.get());
}

using Tob = ThirdOrderBicycle;
using TobCostConvergenceTest = CostConvergenceTest<Tob, kSteps>;
using TobCostEvaluationTest = CostEvaluationTest<Tob, kSteps>;

std::vector<std::vector<PartitionedObjectCost<Tob>::Object>> GetTobObjects() {
  std::vector<std::vector<PartitionedObjectCost<Tob>::Object>> tob_objects;
  tob_objects.resize(kSteps / 2);
  std::vector<Segment2d> lines = {Segment2d(Vec2d(10.0, 0.0), Vec2d(10.0, 5.0)),
                                  Segment2d(Vec2d(10.0, 5.0), Vec2d(5.0, 5.0))};
  for (int k = 0; k < kSteps / 2; ++k) {
    tob_objects[k].push_back(PartitionedObjectCost<Tob>::Object{
        .lines = lines,
        .buffers = {15.0, 15.0},
        .gains = {1.0, 0.5},
        .ref_x = {7.5, 2.5},
        .offset = 100.0,
        .ref_tangent = {1.0, 0.0},
        .enable = k % 2 == 0 ? true : false});
  }
  return tob_objects;
}

std::vector<PartitionedObjectCost<Tob>::filter> GetTobObjectsFilterBoxs() {
  std::vector<PartitionedObjectCost<Tob>::filter> filters;
  filters.reserve(kSteps / 2);
  for (int k = 0; k < kSteps / 2; ++k) {
    filters.push_back(PartitionedObjectCost<Tob>::filter{
        Vec2d(7.5, 2.5), Vec2d(1.0, 0.0), 100.0});
  }
  return filters;
}

const auto kTobObjects = GetTobObjects();
const auto kTobFilters = GetTobObjectsFilterBoxs();

const std::vector<Tob::StateType> kTobStates = {
    Tob::MakeState(15.5, 0.5, 0.0, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(12.0, 7.5, M_PI / 4.0, 1.0, 0.0, 0.1, 0.0),
    Tob::MakeState(7.5, 6.0, -0.1, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(1.5, 3.5, 0.05, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(8.0, 0.5, -0.05, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(3.0, 6.0, 0.15, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(-1.0, 5.0, -0.15, 0.2, 0.1, 0.1, 0.4),
    Tob::MakeState(8.5, 0.5, -M_PI / 2.0, 0.2, 0.1, 0.1, 0.4)};
const std::vector<Tob::ControlType> kTobControls = {Tob::MakeControl(0.1, 0.1)};
const auto kTobCost = std::make_unique<PartitionedObjectCost<Tob>>(
    kTobObjects, kTobFilters, kDistToRac, kAngleToAxis, kCascadeBuffers,
    /*av_model_helper=*/nullptr, kSubNames,
    /*using_hessian_approximate=*/false);

TEST(PartitionedObjectCostTest, TobSumGTest) {
  TobCostEvaluationTest::SumForAllStepsTest(kTobCost.get());
}

TEST(PartitionedObjectCostTest, TobEvaluateWithDebugInfoGTest) {
  TobCostEvaluationTest::EvaluateWithDebugInfoTest(kTobCost.get());
}

TEST(PartitionedObjectCostTest, TobDGDxTest) {
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(), kTobCost.get());
}

TEST(PartitionedObjectCostTest, TobDGDuTest) {
  TobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(), kTobCost.get());
}

TEST(PartitionedObjectCostTest, TobDDGDxDxTest) {
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateStateVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), kTobCost.get());
}

TEST(PartitionedObjectCostTest, TobDDGDuDxTest) {
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateStateVariationBasis(), kTobCost.get());
}

TEST(PartitionedObjectCostTest, TobDDGDuDuTest) {
  TobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kTobStates, kTobControls,
      TobCostConvergenceTest::GenerateControlVariationBasis(),
      TobCostConvergenceTest::GenerateControlVariationBasis(), kTobCost.get());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
