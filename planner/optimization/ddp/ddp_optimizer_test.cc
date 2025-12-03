#include "onboard/planner/optimization/ddp/ddp_optimizer.h"

#include <cmath>
#include <complex>  // IWYU pragma: keep
#include <functional>
#include <string>

#include "Eigen/Householder"  // IWYU pragma: keep
#include "Eigen/Jacobi"       // IWYU pragma: keep

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_debug_hook.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_rate_cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/segmented_speed_limit_cost.h"
#include "onboard/planner/optimization/problem/static_boundary_cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/planner/optimization/problem/tob_curvature_rate_cost.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

namespace {

constexpr int kSteps = 100;
constexpr double kDt = 0.1;
using Tob = ThirdOrderBicycle;

MotionConstraintParamsProto motion_constraint_params =
    DefaultPlannerParams().motion_constraint_params();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();
VehicleDriveParamsProto veh_drive_params = DefaultVehicleDriveParams();

void TestTobDdpOptimizer(
    const std::string& name, const std::vector<Vec2d>& ref_points,
    const std::vector<TrajectoryPoint>& init_traj,
    const std::function<void(Tob*)>& problem_mutator = [](Tob*) {},
    const std::function<void(DdpOptimizer<Tob>*)>& dopt_mutator =
        [](DdpOptimizer<Tob>*) {}) {
  Tob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
              kSteps, kDt, /*enable_dynamic_2nd_derivatives=*/false,
              /*enable_post_process=*/false);
  constexpr double kAccelerationBufferRatio = 0.75;
  constexpr double kJerkBufferRatio = 0.75;
  constexpr double kCurvatureBufferRatio = 0.75;
  constexpr double kCurvatureRateBufferRatio = 0.75;
  problem.AddCost(std::make_unique<IntrinsicJerkCost<Tob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio));
  problem.AddCost(std::make_unique<TobCurvatureRateCost<Tob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio));
  problem.AddCost(std::make_unique<LongitudinalAccelerationCost<Tob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio));
  problem.AddCost(std::make_unique<CurvatureCost<Tob>>(
      ComputeRelaxedCenterMaxCurvature(veh_geo_params, veh_drive_params) *
          kCurvatureBufferRatio,
      kSteps));

  constexpr double kPathGain = 1.0;
  constexpr double kEndStateGain = 1.0;
  std::vector<double> ref_ls(ref_points.size() - 1, 0.0);
  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Tob>>(
      kSteps, kPathGain, kEndStateGain, std::move(ref_ls), ref_points,
      /*center_line_helper=*/nullptr));

  std::vector<double> speed_limit(ref_points.size() - 1);
  for (int i = 0; i < speed_limit.size(); ++i) {
    speed_limit[i] = 10.0;
  }

  // Use normal speed limit info instead of free speed limit info.
  problem.AddCost(std::make_unique<SegmentedSpeedLimitCost<Tob>>(
      kSteps, ref_points, speed_limit, ref_points, speed_limit, kSteps));

  problem_mutator(&problem);

  DdpOptimizer<Tob> dopt(&problem, kSteps);
  dopt_mutator(&dopt);

  IterationVisualizerHook<Tob> hook(kSteps, name,
                                    /*canvas_render_indices=*/true);
  dopt.AddHook(&hook);

  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&ref_points](int index) { return ref_points[index]; },
          ref_points.size(), /*z_inc=*/0.0, /*z0=*/0.0),
      vis::Color(0.8, 0.4, 0.8),
      /*render_indices=*/true, "dopt_test/" + name + "/ref_line");

  std::vector<TrajectoryPoint> trajectory;
  const auto output = dopt.Solve(init_traj);
  if (output.ok()) {
    trajectory = *output;
  } else {
    trajectory = init_traj;
  }

  for (int k = 0; k < kSteps; ++k) {
    EXPECT_EQ(trajectory[k].pos(), trajectory[k].pos());
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(DdpOptimizerTest, TobConstSpeedTest) {
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobConstSpeedTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobParallelRefLineTest) {
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 1.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobParallelRefLineTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobFarParallelRefLineTest) {
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 10.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobFarParallelRefLineTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobDivergingRefLineTest) {
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, i * 2.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobDivergingRefLineTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobLeftTurnTest) {
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 25; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }
  for (int i = 25; i < 50; ++i) {
    ref_points[i] = {50, i * 2.0 - 48};
  }

  // Initial guess trajectory: going north-east.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 1.0 * k});
    init_traj[k].set_v(14.1421356);
    init_traj[k].set_theta(0.78539825);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer("TobLeftTurnTest", ref_points, init_traj);
}

TEST(DdpOptimizerTest, TobLeftTurnTestWithConflictingBoundary) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  CHECK(param_manager != nullptr);
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  // Set corner info.
  VehicleCircleModelParamsProto::CircleParams left_corner;
  VehicleCircleModelParamsProto::CircleParams right_corner;
  constexpr double kFrontCircleDefualtRadius = 0.3;  // m.
  const double half_width = 0.5 * vehicle_geometry_params.width();
  const double dist_to_front_corner =
      Hypot(half_width - kFrontCircleDefualtRadius,
            vehicle_geometry_params.front_edge_to_center() -
                kFrontCircleDefualtRadius);
  const double front_angle =
      std::atan2(half_width - kFrontCircleDefualtRadius,
                 vehicle_geometry_params.front_edge_to_center() -
                     kFrontCircleDefualtRadius);
  left_corner.set_dist_to_rac(dist_to_front_corner);
  left_corner.set_angle_to_axis(front_angle);
  left_corner.set_radius(kFrontCircleDefualtRadius);
  left_corner.set_type(VehicleCircleModelParamsProto::FRONT_LEFT_CORNER);
  right_corner.set_dist_to_rac(dist_to_front_corner);
  right_corner.set_angle_to_axis(-front_angle);
  right_corner.set_radius(kFrontCircleDefualtRadius);
  right_corner.set_type(VehicleCircleModelParamsProto::FRONT_RIGHT_CORNER);

  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 25; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }
  for (int i = 25; i < 50; ++i) {
    ref_points[i] = {50, i * 2.0 - 48};
  }

  std::vector<Vec2d> boundary_path_points(10);
  std::vector<double> ref_gains(10, 1.0);
  std::vector<StaticBoundaryCost<Tob>::BreakPoint> break_points(
      9, {false, {}, {}, {}});
  std::vector<double> dists_to_left_boundary(10, 3.0);
  std::vector<double> dists_to_right_boundary(10, 3.0);
  for (int i = 0; i < 10; ++i) {
    boundary_path_points[i] = {i * 10.0 - 20.0, 0.0};
  }
  const std::vector<double> cascade_buffers = {0.0};
  const std::vector<double> cascade_gains = {1.0};
  const std::vector<double> dists_to_rac = {0.0, 2.0};
  const std::vector<std::string> sub_names = {""};

  // Initial guess trajectory: going 5 deg north of east.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  const double theta = 5.0 / 180.0 * M_PI;
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 1.0 * k * std::tan(theta)});
    init_traj[k].set_v(10.0 / std::cos(theta));
    init_traj[k].set_theta(theta);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestTobDdpOptimizer(
      "TobLeftTurnTestWithConflictingBoundary", ref_points, init_traj,
      [&vehicle_geometry_params, &boundary_path_points, &dists_to_left_boundary,
       &dists_to_right_boundary, &break_points, &ref_gains, &cascade_buffers,
       &cascade_gains, &dists_to_rac, &left_corner, &right_corner,
       &sub_names](Tob* ddp) {
        ddp->AddCost(std::make_unique<StaticBoundaryCost<Tob>>(
            kSteps, vehicle_geometry_params, boundary_path_points,
            /*center_line_helper=*/nullptr, dists_to_left_boundary,
            break_points, ref_gains,
            /*left=*/true, dists_to_rac, left_corner, /*use_qtfm=*/false,
            sub_names, cascade_gains, cascade_buffers));
        ddp->AddCost(std::make_unique<StaticBoundaryCost<Tob>>(
            kSteps, vehicle_geometry_params, boundary_path_points,
            /*center_line_helper=*/nullptr, dists_to_right_boundary,
            break_points, ref_gains,
            /*left=*/false, dists_to_rac, right_corner, /*use_qtfm=*/false,
            sub_names, cascade_gains, cascade_buffers));
      });
}

DdpOptimizer<Tob> CreateSimplySolver() {
  Tob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
              kSteps, kDt, /*enable_dynamic_2nd_derivatives=*/false,
              /*enable_post_process=*/false);
  return DdpOptimizer<Tob>(&problem, kSteps);
}

TEST(DdpOptimizerTest, IdentitySemiPositiveTest) {
  DdpOptimizer<Tob> solver = CreateSimplySolver();
  EXPECT_TRUE(
      solver.IsDdJdxdxSemiPositive(DdpOptimizer<Tob>::DDJDxDxType::Identity()));
}

TEST(DdpOptimizerTest, ZeroSemiPositiveTest) {
  DdpOptimizer<Tob> solver = CreateSimplySolver();
  EXPECT_TRUE(
      solver.IsDdJdxdxSemiPositive(DdpOptimizer<Tob>::DDJDxDxType::Zero()));
}

TEST(DdpOptimizerTest, FullRankSemiPositiveTest) {
  DdpOptimizer<Tob> solver = CreateSimplySolver();
  DdpOptimizer<Tob>::StateType vec =
      Tob::MakeState(1.0, 2.0, 0.5, 5.0, 0.01, -2.0, 300.0);
  DdpOptimizer<Tob>::DDJDxDxType positive_matrix = vec * vec.transpose();

  EXPECT_TRUE(solver.IsDdJdxdxSemiPositive(positive_matrix));
  EXPECT_FALSE(solver.IsDdJdxdxSemiPositive(-positive_matrix));

  DdpOptimizer<Tob>::DDJDxDxType indefinite_matrix;
  indefinite_matrix << 1.0, 2.0, 3.0, 2.1, 4.0, 9.2, 4.0, 2.0, 4.5, 1.2, 3.2,
      1.7, 0.2, 5.0, 3.0, 1.2, 4.2, 6.0, 3.0, 0.3, 3.2, 2.1, 3.2, 6.0, 5.5, 3.5,
      0.2, 4.0, 4.0, 1.7, 3.0, 3.5, 6.0, 0.3, 0.1, 9.2, 0.2, 0.3, 0.2, 0.3, 2.7,
      0.9, 4.0, 5.0, 3.2, 4.0, 0.1, 0.9, 2.1;

  EXPECT_FALSE(solver.IsDdJdxdxSemiPositive(indefinite_matrix));
  EXPECT_FALSE(solver.IsDdJdxdxSemiPositive(-indefinite_matrix));
}

TEST(DdpOptimizerTest, NonFullRankSemiPositiveTest) {
  DdpOptimizer<Tob> solver = CreateSimplySolver();
  DdpOptimizer<Tob>::StateType vec =
      Tob::MakeState(1.0, 2.0, 0.5, 0.0, 0.01, -2.0, 300.0);
  DdpOptimizer<Tob>::DDJDxDxType semi_positive_matrix = vec * vec.transpose();

  EXPECT_TRUE(solver.IsDdJdxdxSemiPositive(semi_positive_matrix));
  EXPECT_FALSE(solver.IsDdJdxdxSemiPositive(-semi_positive_matrix));

  DdpOptimizer<Tob>::DDJDxDxType indefinite_matrix;
  indefinite_matrix << 1.0, 2.0, 3.0, 2.1, 0.0, 9.2, 4.0, 2.0, 4.5, 1.2, 3.2,
      0.0, 0.2, 5.0, 3.0, 1.2, 4.2, 6.0, 0.0, 0.3, 3.2, 2.1, 3.2, 6.0, 5.5, 0.0,
      0.2, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.2, 0.2, 0.3, 0.2, 0.0, 2.7,
      0.9, 4.0, 5.0, 3.2, 4.0, 0.0, 0.9, 2.1;
  EXPECT_FALSE(solver.IsDdJdxdxSemiPositive(indefinite_matrix));
  EXPECT_FALSE(solver.IsDdJdxdxSemiPositive(-indefinite_matrix));
}

using Mfob = MixedFourthOrderBicycle;

void TestMfobDdpOptimizer(
    const std::string& name, const std::vector<Vec2d>& ref_points,
    const std::vector<TrajectoryPoint>& init_traj,
    const std::function<void(Mfob*)>& problem_mutator = [](Mfob*) {},
    const std::function<void(DdpOptimizer<Mfob>*)>& dopt_mutator =
        [](DdpOptimizer<Mfob>*) {}) {
  Mfob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
               kSteps, kDt, /*enable_dynamic_2nd_derivatives=*/false,
               /*enable_post_process=*/false);
  constexpr double kAccelerationBufferRatio = 0.75;
  constexpr double kJerkBufferRatio = 0.75;
  constexpr double kCurvatureBufferRatio = 0.75;
  constexpr double kCurvatureRateBufferRatio = 0.75;
  constexpr double kCurvatureRateRateBufferRatio = 0.75;

  problem.AddCost(std::make_unique<IntrinsicJerkCost<Mfob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio));
  problem.AddCost(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio));
  problem.AddCost(std::make_unique<MfobCurvatureRateRateCost<Mfob>>(
      motion_constraint_params.max_chi() * kCurvatureRateRateBufferRatio));
  problem.AddCost(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio));
  problem.AddCost(std::make_unique<CurvatureCost<Mfob>>(
      ComputeRelaxedCenterMaxCurvature(veh_geo_params, veh_drive_params) *
          kCurvatureBufferRatio,
      kSteps));

  constexpr double kPathGain = 1.0;
  constexpr double kEndStateGain = 1.0;
  std::vector<double> ref_ls(ref_points.size() - 1, 0.0);
  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      kSteps, kPathGain, kEndStateGain, std::move(ref_ls), ref_points,
      /*center_line_helper=*/nullptr));

  std::vector<double> speed_limit(ref_points.size() - 1);
  for (int i = 0; i < speed_limit.size(); ++i) {
    speed_limit[i] = 10.0;
  }

  // Use normal speed limit info instead of free speed limit info.
  problem.AddCost(std::make_unique<SegmentedSpeedLimitCost<Mfob>>(
      kSteps, ref_points, speed_limit, ref_points, speed_limit, kSteps));

  problem_mutator(&problem);

  DdpOptimizer<Mfob> dopt(&problem, kSteps);
  dopt_mutator(&dopt);

  IterationVisualizerHook<Mfob> hook(kSteps, name,
                                     /*canvas_render_indices=*/true);
  dopt.AddHook(&hook);

  CanvasDrawTrajectory(
      VisIndexTrajToVector(
          [&ref_points](int index) { return ref_points[index]; },
          ref_points.size(), /*z_inc=*/0.0, /*z0=*/0.0),
      vis::Color(0.8, 0.4, 0.8),
      /*render_indices=*/true, "dopt_test/" + name + "/ref_line");

  std::vector<TrajectoryPoint> trajectory;
  const auto output = dopt.Solve(init_traj);
  if (output.ok()) {
    trajectory = *output;
  } else {
    trajectory = init_traj;
  }

  for (int k = 0; k < kSteps; ++k) {
    EXPECT_EQ(trajectory[k].pos(), trajectory[k].pos());
  }

  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(DdpOptimizerTest, MfobConstSpeedTest) {
  std::vector<Vec2d> ref_points(50);
  for (int i = 0; i < 50; ++i) {
    ref_points[i] = {i * 2.0, 0.0};
  }

  // Initial guess trajectory: going north.
  std::vector<TrajectoryPoint> init_traj(kSteps);
  for (int k = 0; k < kSteps; ++k) {
    init_traj[k].set_pos({1.0 * k, 0.0});
    init_traj[k].set_v(10.0);
    init_traj[k].set_theta(0.0);
    init_traj[k].set_a(0.0);
    init_traj[k].set_kappa(0.0);
  }

  TestMfobDdpOptimizer("TobConstSpeedTest", ref_points, init_traj);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
