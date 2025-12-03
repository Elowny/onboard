#include "onboard/planner/optimization/problem/static_boundary_cost.h"

#include <cmath>

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
using MfobCostConvergenceTest = CostConvergenceTest<Mfob, kSteps>;
using MfobCostEvaluationTest = CostEvaluationTest<Mfob, kSteps>;

const std::vector<Vec2d> kPathPoints = {
    {-20.0, 10.0}, {0.0, 15.0}, {20.0, 25.0}};
const std::vector<double> kPathBoundaryDists = {3.0, 4.0, 5.0};
const std::vector<double> kRefGains = {1.0, 1.0, 0.3};
const std::vector<double> kCascadeBuffers = {0.0};
const std::vector<double> kCascadeGains = {0.002};
const std::vector<std::string> kSubNames = {""};
const std::vector<double> kDistToRac = {0.0, 2.0};
const std::vector<StaticBoundaryCost<Mfob>::BreakPoint> kBreakPoints = {
    {true, {0.0, 0.1, 1.0}, {10.0, 1.0 / 0.9}, {3.0, 3.0, 4.0}},
    {false, {}, {}, {}}};

const std::vector<Mfob::StateType> kRightStates = {
    Mfob::MakeState(-30.0, 2.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 5.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 10.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 18.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::StateType> kLeftStates = {
    Mfob::MakeState(-30.0, 15.0, M_PI * 0.25, 1.0, 0.0, 0.0, 0.0, 0.0),
    Mfob::MakeState(-10.0, 18.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    Mfob::MakeState(10.0, 20.0, -M_PI * 0.4, 2.0, 0.0, 0.1, 0.0, 0.0),
    Mfob::MakeState(25.0, 35.0, M_PI * 0.1, 1.0, 0.1, 0.1, 0.0, 0.0)};
const std::vector<Mfob::ControlType> kControls = {Mfob::MakeControl(0.1, 0.1)};

class StaticBoundaryCostTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    CHECK(param_manager != nullptr);
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params_ =
        run_params.vehicle_params().vehicle_geometry_params();
    // Set center line helper.
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
    path_ = std::make_unique<KdTreeFrenetFrame>(
        BuildKdTreeFrenetFrame(kPathPoints, /*down_sample_raw_points=*/true)
            .value());
    station_query_helper_ = std::make_unique<CenterLineQueryHelper<Mfob>>(
        kSteps, std::move(query_circles), path_.get(),
        /*last_real_point_index=*/kPathPoints.size(), "MfobStationQueryHelper");

    // Set corner info.
    constexpr double kFrontCircleDefualtRadius = 0.3;  // m.
    const double half_width = 0.5 * vehicle_geometry_params_.width();
    const double dist_to_front_corner =
        Hypot(half_width - kFrontCircleDefualtRadius,
              vehicle_geometry_params_.front_edge_to_center() -
                  kFrontCircleDefualtRadius);
    const double front_angle =
        std::atan2(half_width - kFrontCircleDefualtRadius,
                   vehicle_geometry_params_.front_edge_to_center() -
                       kFrontCircleDefualtRadius);
    left_corner_.set_dist_to_rac(dist_to_front_corner);
    left_corner_.set_angle_to_axis(front_angle);
    left_corner_.set_radius(kFrontCircleDefualtRadius);
    left_corner_.set_type(VehicleCircleModelParamsProto::FRONT_LEFT_CORNER);
    right_corner_.set_dist_to_rac(dist_to_front_corner);
    right_corner_.set_angle_to_axis(-front_angle);
    right_corner_.set_radius(kFrontCircleDefualtRadius);
    right_corner_.set_type(VehicleCircleModelParamsProto::FRONT_RIGHT_CORNER);
  }

 protected:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::unique_ptr<KdTreeFrenetFrame> path_;
  std::unique_ptr<CenterLineQueryHelper<Mfob>> station_query_helper_;
  VehicleCircleModelParamsProto::CircleParams left_corner_;
  VehicleCircleModelParamsProto::CircleParams right_corner_;
};
}  // namespace

TEST_F(StaticBoundaryCostTest, MfobSumGTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/false, kDistToRac, right_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  MfobCostEvaluationTest::SumForAllStepsTest(&cost);
}

TEST_F(StaticBoundaryCostTest, MfobEvaluateWithDebugInfoGTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/false, kDistToRac, right_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  MfobCostEvaluationTest::EvaluateWithDebugInfoTest(&cost);
}

TEST_F(StaticBoundaryCostTest, MfobUpdateTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/false, kDistToRac, right_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  Mfob::StatesType xs(kSteps * Mfob::kStateSize);
  Mfob::ControlsType us(kSteps * Mfob::kControlSize);
  for (int i = 0; i < kSteps; ++i) {
    Mfob::SetStateAtStep(
        Mfob::MakeState(-25.0 + i * 0.8, 0.0, 0.0, 2.0, 0.0, 0.1, 0.0, 0.0), i,
        &xs);
  }
  cost.Update(xs, us, kSteps);
}

TEST_F(StaticBoundaryCostTest, MfobUpdateWithHelperTest) {
  StaticBoundaryCost<Mfob> cost(
      kSteps, vehicle_geometry_params_, kPathPoints,
      station_query_helper_.get(), kPathBoundaryDists, kBreakPoints, kRefGains,
      /*left=*/false, kDistToRac, right_corner_, /*use_qtfm=*/false, kSubNames,
      kCascadeGains, kCascadeBuffers);
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

TEST_F(StaticBoundaryCostTest, MfobRightDGDxTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/false, kDistToRac, right_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kRightStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StaticBoundaryCostTest, MfobLeftDGDxTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/true, kDistToRac, left_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  MfobCostConvergenceTest::ExpectCostGradientResidualOrder(
      kLeftStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StaticBoundaryCostTest, MfobRightDDGDxDxTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/false, kDistToRac, right_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kRightStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

TEST_F(StaticBoundaryCostTest, MfobLeftDDGDxDxTest) {
  StaticBoundaryCost<Mfob> cost(kSteps, vehicle_geometry_params_, kPathPoints,
                                /*center_line_helper=*/nullptr,
                                kPathBoundaryDists, kBreakPoints, kRefGains,
                                /*left=*/true, kDistToRac, left_corner_,
                                /*use_qtfm=*/false, kSubNames, kCascadeGains,
                                kCascadeBuffers);
  MfobCostConvergenceTest::ExpectCostHessianResidualOrder(
      kLeftStates, kControls,
      MfobCostConvergenceTest::GenerateStateVariationBasis(),
      MfobCostConvergenceTest::GenerateStateVariationBasis(), &cost);
}

}  // namespace planner
}  // namespace qcraft
