#include "onboard/planner/mfob_trajectory_smoother.h"

#include <algorithm>
#include <complex>  // IWYU pragma: keep
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "Eigen/Householder"  // IWYU pragma: keep
#include "Eigen/Jacobi"       // IWYU pragma: keep
#include "Eigen/LU"           // IWYU pragma: keep
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/static_boundary_cost_lite.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/vis/common/color.h"

DEFINE_int32(mfob_trajectory_smoother_canvas_level, 0,
             "Mfob trajectory smoother canvas verbosity level");
DEFINE_int32(mfob_trajectory_smoother_verbosity_level, 0,
             "Mfob trajectory smoother verbosity level");

namespace qcraft {
namespace planner {
namespace {

using Mfob = MixedFourthOrderBicycle;
using StatesType = Mfob::StatesType;
using StateType = Mfob::StateType;
using ControlsType = Mfob::ControlsType;
using ControlType = Mfob::ControlType;

std::vector<TrajectoryPoint> GeneratePurePursuitInitTraj(
    int trajectory_steps, double trajectory_time_step, const Mfob& problem,
    const std::vector<TrajectoryPoint>& init_traj,
    const VehicleGeometryParamsProto& veh_geo_params) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  // Pure pursuit on the initial trajectory within control bounds.
  constexpr double kLongitudinalLookAhead = 0.5;  // s.
  const int longitudinal_lookahead_steps =
      RoundToInt(kLongitudinalLookAhead / trajectory_time_step);

  StatesType init_xs = Mfob::FitState(init_traj);

  // Get longitudinal reference trajectory
  std::vector<StateType> longitudinal_extended_xs(trajectory_steps +
                                                  longitudinal_lookahead_steps);
  for (int k = 0; k < trajectory_steps; ++k) {
    longitudinal_extended_xs[k] = Mfob::GetStateAtStep(init_xs, k);
  }
  StateType x = Mfob::GetStateAtStep(init_xs, trajectory_steps - 1);
  for (int k = 0; k < longitudinal_lookahead_steps; ++k) {
    x = problem.EvaluateF(k, x, ControlType::Zero());
    longitudinal_extended_xs[trajectory_steps + k] = x;
  }

  const auto get_longitudinal_target =
      [&longitudinal_extended_xs,
       longitudinal_lookahead_steps](int index) -> StateType {
    QCHECK_LT(index + longitudinal_lookahead_steps,
              longitudinal_extended_xs.size());
    return longitudinal_extended_xs[index + longitudinal_lookahead_steps];
  };

  // Get lateral reference trajectory
  std::vector<StateType> lateral_extended_xs(trajectory_steps);
  std::vector<double> lateral_extened_xs_s(trajectory_steps);
  for (int k = 0; k < trajectory_steps; ++k) {
    lateral_extended_xs[k] = Mfob::GetStateAtStep(init_xs, k);
    lateral_extened_xs_s[k] = Mfob::s(init_xs, k);
  }

  const double init_traj_s = Mfob::StateGetS(lateral_extended_xs.back());
  const PiecewiseLinearFunction<StateType, double> lateral_reference_plf(
      lateral_extened_xs_s, lateral_extended_xs);
  const auto get_lateral_target =
      [&lateral_reference_plf, &lateral_extended_xs, &init_traj_s](
          const StateType& state, double lateral_look_ahead_dist) -> StateType {
    const double target_s = lateral_look_ahead_dist + Mfob::StateGetS(state);
    if (target_s >= init_traj_s) {
      StateType target = lateral_extended_xs.back();
      Vec2d target_theta_tangent =
          Vec2d::UnitFromAngle(Mfob::StateGetTheta(target));
      Mfob::StateSetPos(Mfob::StateGetPos(target) +
                            (target_s - init_traj_s) * target_theta_tangent,
                        &target);
      return target;
    } else {
      return lateral_reference_plf(target_s);
    }
  };

  const PiecewiseLinearFunction<double> lateral_look_ahead_v_plf(
      {0.0, 10.0, 20.0, 30.0, 50.0}, {0.5, 0.7, 0.9, 1.2, 1.5});

  ControlsType us = ControlsType::Zero(trajectory_steps * Mfob::kControlSize);
  StatesType xs = StatesType::Zero(trajectory_steps * Mfob::kStateSize);
  x = Mfob::GetStateAtStep(init_xs, 0);
  for (int k = 0; k < trajectory_steps; ++k) {
    Mfob::SetStateAtStep(x, k, &xs);
    StateType longitudinal_target = get_longitudinal_target(k);
    const double lateral_look_ahead_dist =
        lateral_look_ahead_v_plf(Mfob::StateGetV(x)) * Mfob::StateGetV(x) +
        veh_geo_params.wheel_base();
    StateType lateral_target = get_lateral_target(x, lateral_look_ahead_dist);
    ControlType u = problem.PurePursuitController(
        x, longitudinal_target, lateral_target, longitudinal_lookahead_steps,
        lateral_look_ahead_dist);
    x = problem.EvaluateF(k, x, u);
    Mfob::SetControlAtStep(u, k, &us);
  }

  std::vector<TrajectoryPoint> res(trajectory_steps);
  for (int k = 0; k < trajectory_steps; ++k) {
    TrajectoryPoint& point = res[k];
    problem.ExtractTrajectoryPoint(k, Mfob::GetStateAtStep(xs, k),
                                   Mfob::GetControlAtStep(us, k), &point);
  }
  return res;
}

void AddStaticBoundaryCost(
    int trajectory_steps, const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectorySmootherCostWeightParamsProto& smoother_params,
    Mfob* problem) {
  QCHECK_NOTNULL(problem);
  QCHECK_GT(trajectory_steps, 0);
  const int drive_passage_size = drive_passage.stations().size();

  std::vector<double> l_max;
  std::vector<double> l_min;

  l_min.reserve(drive_passage_size);
  l_max.reserve(drive_passage_size);

  for (int i = 0; i < drive_passage_size; ++i) {
    const Station& station = drive_passage.stations()[StationIndex(i)];
    const auto curb_pair_or = station.QueryCurbOffsetAt(/*signed_lat=*/0.0);
    QCHECK_OK(curb_pair_or.status());
    l_min.push_back(curb_pair_or->first);
    l_max.push_back(curb_pair_or->second);
  }

  problem->AddCost(std::make_unique<StaticBoundaryCostLite<Mfob>>(
      trajectory_steps, drive_passage.frenet_frame(),
      drive_passage.frenet_frame()->points(), std::move(l_min),
      std::move(l_max),
      veh_geo_params.width() * 0.5 +
          smoother_params.static_boundary_cost_buffer(),
      "StaticBoundaryCostLite", smoother_params.static_boundary_cost_weight()));
}

}  // namespace

std::vector<TrajectoryPoint> SmoothTrajectoryByMixedFourthOrderDdp(
    int trajectory_steps, double trajectory_time_step,
    const std::vector<TrajectoryPoint>& ref_traj,
    const DrivePassage& drive_passage, const std::string& owner,
    const TrajectorySmootherCostWeightParamsProto& smoother_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params) {
  SCOPED_QTRACE("SmoothTrajectoryByMixedFourthOrderDdp");
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);

  ControlsType ref_us =
      ControlsType::Zero(trajectory_steps * Mfob::kControlSize);
  StatesType ref_xs = Mfob::FitState(ref_traj);

  Mfob problem(&motion_constraint_params, &veh_geo_params, &veh_drive_params,
               trajectory_steps, trajectory_time_step,
               /*enable_dynamic_2nd_derivatives=*/false,
               /*enable_post_process=*/false);

  std::vector<Vec2d> ref_path_points;
  ref_path_points.reserve(ref_traj.size() + 1);
  for (int i = 0; i < ref_traj.size(); ++i) {
    ref_path_points.push_back(ref_traj[i].pos());
  }
  // Add one more point at tail in case of short trajectory.
  constexpr double kTailLength = 0.2;  // m.
  ref_path_points.push_back(
      ref_traj.back().pos() +
      kTailLength * Vec2d::FastUnitFromAngle(ref_traj.back().theta()));

  std::vector<double> state_deviation_weights(
      trajectory_steps * Mfob::kStateSize,
      smoother_params.state_deviation_gain());
  std::vector<double> control_penalty_weights(
      trajectory_steps * Mfob::kControlSize,
      smoother_params.jerk_penalty_gain());
  const double corrected_chi_penalty_gain =
      smoother_params.chi_penalty_gain() *
      std::max(
          smoother_params.chi_penalty_gain_min_factor(),
          smoother_params.chi_penalty_speed_gain() * Sqr(ref_traj.front().v()));
  for (int i = 0; i < trajectory_steps; ++i) {
    state_deviation_weights[i * Mfob::kStateSize + Mfob::kStateVIndex] =
        smoother_params.v_deviation_gain();
    control_penalty_weights[i * Mfob::kControlSize + Mfob::kControlChiIndex] =
        corrected_chi_penalty_gain;
  }

  // For x, y and theta end
  state_deviation_weights[(trajectory_steps - 1) * Mfob::kStateSize +
                          Mfob::kStateXIndex] =
      smoother_params.end_pose_deviation_gain();
  state_deviation_weights[(trajectory_steps - 1) * Mfob::kStateSize +
                          Mfob::kStateYIndex] =
      smoother_params.end_pose_deviation_gain();
  state_deviation_weights[(trajectory_steps - 1) * Mfob::kStateSize +
                          Mfob::kStateThetaIndex] =
      smoother_params.end_heading_deviation_gain();

  std::vector<Vec2d> points;
  points.reserve(ref_path_points.size());
  constexpr double kMinSampleDistanceSqr = 0.01;
  points.push_back(ref_path_points.front());
  for (int i = 1; i < ref_path_points.size(); ++i) {
    const double distance_sqr =
        ref_path_points[i].DistanceSquareTo(points.back());
    if (distance_sqr < kMinSampleDistanceSqr) {
      continue;  // Skip if too close.
    }
    points.push_back(ref_path_points[i]);
  }
  std::vector<double> ref_ls(points.size() - 1, 0.0);
  std::vector<double> ref_path_deviation_gains(points.size() - 1, 1.0);

  problem.AddCost(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      trajectory_steps, smoother_params.path_gain(),
      smoother_params.end_state_gain(), std::move(ref_ls), points,
      /*center_line_helper=*/nullptr, std::move(ref_path_deviation_gains),
      "MfobTrajSmooth::ReferenceLineDeviationCost",
      smoother_params.ref_path_deviation_gain()));
  problem.AddCost(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      std::move(ref_xs), std::move(state_deviation_weights),
      "MfobTrajSmooth::StateRegularization", smoother_params.scale()));
  problem.AddCost(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
      std::move(ref_us), std::move(control_penalty_weights),
      "MfobTrajSmooth::ControlRegularization", smoother_params.scale()));
  constexpr double kCurvatureBufferRatio = 0.8;
  constexpr double kCurvatureRateBufferRatio = 0.8;
  problem.AddCost(std::make_unique<CurvatureCost<Mfob>>(
      ComputeCenterMaxCurvature(veh_geo_params, veh_drive_params) *
          kCurvatureBufferRatio,
      trajectory_steps, "MfobTrajSmooth::MfobCurvatureCost",
      smoother_params.curvature_penalty()));
  problem.AddCost(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio,
      "MfobTrajSmooth::MfobCurvatureRateCost",
      smoother_params.curvature_rate_penalty()));
  problem.AddCost(std::make_unique<ForwardSpeedCost<Mfob>>(
      "MfobTrajSmooth::MfobForwardSpeedCost",
      smoother_params.forward_speed_cost_weight()));
  AddStaticBoundaryCost(trajectory_steps, drive_passage, veh_geo_params,
                        smoother_params, &problem);

  constexpr double kAccelerationBufferRatio = 1.0;
  const std::vector<double> accel_cascade_buffers = {0.0};
  const std::vector<double> accel_cascade_gains = {smoother_params.scale()};
  const std::vector<double> decel_cascade_buffers = {0.0};
  const std::vector<double> decel_cascade_gains = {smoother_params.scale()};

  problem.AddCost(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio,
      accel_cascade_buffers, accel_cascade_gains, decel_cascade_buffers,
      decel_cascade_gains, "MfobTrajSmooth::MfobLongitudinalAccelerationCost",
      smoother_params.longitudinal_acceleration_cost_weight()));

  DdpOptimizer<Mfob> smoother(
      &problem, trajectory_steps, /*owner=*/"mfob_smoother",
      /*verbosity=*/FLAGS_mfob_trajectory_smoother_verbosity_level,
      smoother_params.optimizer_params());
  std::vector<TrajectoryPoint> init_vals =
      GeneratePurePursuitInitTraj(trajectory_steps, trajectory_time_step,
                                  problem, ref_traj, veh_geo_params);

  const auto solve_start_time = absl::Now();
  auto output =
      smoother.Solve(init_vals, DdpOptimizer<Mfob>::SolveConfig::Onboard());
  const absl::Duration time_consuming = absl::Now() - solve_start_time;
  VLOG(2) << owner + " ddp smooth time consuming: "
          << absl::ToDoubleMilliseconds(time_consuming);

  if (!output.ok()) {
    QLOG(WARNING) << "trajectory smoother solve failed: "
                  << output.status().message();
    return ref_traj;
  }

  std::vector<TrajectoryPoint> res = std::move(*output);

  if (VLOG_IS_ON(4)) {
    for (int i = 0; i < ref_traj.size(); i++) {
      VLOG(4) << "ref_traj " << i << "  " << ref_traj[i].DebugString();
    }
    for (int i = 0; i < init_vals.size(); i++) {
      VLOG(4) << "pp_traj " << i << "  " << init_vals[i].DebugString();
    }
    for (int i = 0; i < res.size(); i++) {
      VLOG(4) << "res " << i << "  " << res[i].DebugString();
    }
  }

  if (FLAGS_mfob_trajectory_smoother_canvas_level >= 2) {
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&ref_path_points](int index) { return ref_path_points[index]; },
            ref_path_points.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        vis::Color(0.8, 0.4, 0.8),
        /*render_indices=*/true, owner + "/traj_smooth/input");
    CanvasDrawTrajectory(
        VisIndexTrajToVector(
            [&init_vals](int index) { return init_vals[index].pos(); },
            init_vals.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        vis::Color(0.7, 0.6, 0.2),
        /*render_indices=*/true, owner + "/traj_smooth/pp");
    CanvasDrawTrajectory(
        VisIndexTrajToVector([res](int index) { return res[index].pos(); },
                             res.size(), /*z_inc=*/0.0, /*z0=*/0.0),
        vis::Color(0.5, 0.5, 0.7),
        /*render_indices=*/true, owner + "/traj_smooth/final");
  }
  return res;
}

}  // namespace planner
}  // namespace qcraft
