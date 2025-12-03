#include "onboard/planner/optimization/problem/mfob_path_boundary_cost.h"

#include <cmath>

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/optimization/problem/cost_convergence_test_util.h"
#include "onboard/planner/optimization/problem/cost_evaluation_test_util.h"
#include "onboard/planner/proto/planner_params.pb.h"

namespace qcraft {
namespace planner {
namespace {

constexpr int kSteps = 100;
using Mfob = MixedFourthOrderBicycle;
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

const std::vector<Vec2d> kPathPoints = {
    {-20.0, 10.0}, {0.0, 15.0}, {20.0, 25.0}, {40.0, 25.0}};
const std::vector<std::vector<double>> kPathBoundaryDists = {
    {40.0, 70.0, 65.0}};
const std::vector<double> kLOffsets = {5.0, -5.0, 5.0};
const std::vector<std::vector<double>> kDistsToClampBuffers = {{0.0, 0.0, 0.0}};
const std::vector<std::vector<double>> kRefGains = {{0.005, 0.5, 0.5}};
const std::vector<double> kDistToRac = {0.0, 1.0, 2.0};
const int kRacIndex = 0;

const std::vector<double> kBuffersMin = {-1000.0};
const std::vector<double> kRearBuffersMax = {1000.0};
const std::vector<double> kFrontBuffersMax = {1000.0};
const std::vector<double> kClampedBufferOffset = {0.1};
const std::vector<double> kCascadeGains = {0.5};
const std::vector<double> kRearGain = {0.5};
const std::vector<double> kFrontGain = {0.5};

const std::vector<std::string> kSubNames = {""};

const std::vector<Mfob::StateType> kMfobRightStates = {
    Mfob::MakeState(-30.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 5.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 11.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 18.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::StateType> kMfobLeftStates = {
    Mfob::MakeState(-30.0, 15.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 18.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(0.0, 50.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::ControlType> kMfobControls = {
    Mfob::MakeControl(0.1, 0.1)};

class MfobPathBoundaryCostTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params_ =
        run_params.vehicle_params().vehicle_geometry_params();

    std::vector<VehicleCircleModelParamsProto::CircleParams> query_circles;
    for (const double dist : kDistToRac) {
      VehicleCircleModelParamsProto::CircleParams circle;
      circle.set_dist_to_rac(dist);
      circle.set_radius(0.5 * vehicle_geometry_params_.width());
      circle.set_type(dist == 0.0
                          ? VehicleCircleModelParamsProto::REAR_AXIS_CENTER
                          : VehicleCircleModelParamsProto::MID_AXIS_CENTER);
      query_circles.push_back(std::move(circle));
    }
    const auto path_or =
        BuildKdTreeFrenetFrame(kPathPoints, /*down_sample_raw_points=*/true);
    QCHECK(path_or.ok());
    path_ = std::make_unique<KdTreeFrenetFrame>(path_or.value());
    station_query_helper_ = std::make_unique<CenterLineQueryHelper<Mfob>>(
        kSteps, std::move(query_circles), path_.get(),
        /*last_real_point_index=*/kPathPoints.size(), "MfobStationQueryHelper");
  }

 protected:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::unique_ptr<CenterLineQueryHelper<Mfob>> station_query_helper_;
  std::unique_ptr<KdTreeFrenetFrame> path_;
};
}  // namespace

// Mfob tests.
TEST_F(MfobPathBoundaryCostTest, SumGTest) {
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/false,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

// Mfob tests.
TEST_F(MfobPathBoundaryCostTest, EvaluateWithDebugInfoGTest) {
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/false,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(&cost);
}

TEST_F(MfobPathBoundaryCostTest, UpdateTest) {
  using Mfob = Mfob;
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/false,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  Mfob::StatesType xs(kSteps * Mfob::kStateSize);
  Mfob::ControlsType us(kSteps * Mfob::kControlSize);
  for (int i = 0; i < kSteps; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, kSteps);
}

TEST_F(MfobPathBoundaryCostTest, UpdateWithHelperTest) {
  using Mfob = Mfob;
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      station_query_helper_.get(), kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/false,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  Mfob::StatesType xs(kSteps * Mfob::kStateSize);
  Mfob::ControlsType us(kSteps * Mfob::kControlSize);
  for (int i = 0; i < kSteps; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  station_query_helper_->Update(xs, us);
  cost.Update(xs, us, kSteps);
}

TEST_F(MfobPathBoundaryCostTest, RightDGDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/false,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobRightStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MfobPathBoundaryCostTest, LeftDGDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/true,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kMfobLeftStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MfobPathBoundaryCostTest, RightDDGDxDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/false,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobRightStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(MfobPathBoundaryCostTest, LeftDDGDxDxTest) {
  MfobPathBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      /*center_line_helper=*/nullptr, kLOffsets, kPathBoundaryDists,
      kDistsToClampBuffers,
      /*left=*/true,
      /*using_hessian_approximate=*/false, kDistToRac, kRacIndex, kRefGains,
      kSubNames,
      /*use_qtfm=*/false, kBuffersMin, kRearBuffersMax, kFrontBuffersMax,
      kClampedBufferOffset, kCascadeGains, kRearGain, kFrontGain);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kMfobLeftStates, kMfobControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
