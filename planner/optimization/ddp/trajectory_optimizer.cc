#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/hmi_util.h"
#include "onboard/planner/mfob_trajectory_smoother.h"
#include "onboard/planner/ml/optimizer_auto_tuning/auto_tuning_common_flags.h"
#include "onboard/planner/ml/optimizer_auto_tuning/auto_tuning_utils.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_debug_hook.h"
#include "onboard/planner/optimization/ddp/object_cost_util.h"
#include "onboard/planner/optimization/ddp/path_time_corridor.h"
#include "onboard/planner/optimization/ddp/speed_limit_cost_util.h"
#include "onboard/planner/optimization/ddp/static_boundary_cost_util.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_state.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/optimization/problem/av_model_helper.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/curvature_cost.h"
#include "onboard/planner/optimization/problem/curvature_deviation_cost.h"
#include "onboard/planner/optimization/problem/end_heading_cost.h"
#include "onboard/planner/optimization/problem/forward_speed_cost.h"
#include "onboard/planner/optimization/problem/intrinsic_jerk_cost.h"
#include "onboard/planner/optimization/problem/lateral_acceleration_cost.h"
#include "onboard/planner/optimization/problem/longitudinal_acceleration_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_cost.h"
#include "onboard/planner/optimization/problem/mfob_curvature_rate_rate_cost.h"
#include "onboard/planner/optimization/problem/mfob_intrinsic_lateral_snap_cost.h"
#include "onboard/planner/optimization/problem/mfob_lateral_jerk_cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/reference_control_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_line_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_longitudinal_jerk_deviation_cost.h"  // NOLINT
#include "onboard/planner/optimization/problem/reference_single_state_deviation_cost.h"
#include "onboard/planner/optimization/problem/reference_state_deviation_cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/color.h"

DEFINE_bool(send_traj_optimizer_result_to_canvas, false,
            "Whether to send trajectory optimizer result to canvas.");
DEFINE_int32(traj_opt_canvas_level, 0, "Traj opt canvas level.");
DEFINE_int32(traj_opt_verbosity_level, 2, "Traj opt verbosity level.");
DEFINE_double(auto_tuning_gamma, 0.99,
              "Only used in auto tuning mode, gamma is the discounted rate.");
DEFINE_bool(enable_ipopt_solver, false,
            "Whether to enable ipopt solver to be used for being compared with "
            "ddp optimizer.");
DEFINE_bool(traj_opt_draw_circle, false, "Whether to draw av model.");
DEFINE_bool(use_static_boundary_cost_in_vision_map, true,
            "Use the old curb cost when using online map");
DEFINE_bool(msd_static_boundary_cost_v2, true,
            "V2 version of msd_static_boundary_cost, the curb buffer is speed "
            "variant.");

namespace qcraft {
namespace planner {
namespace {

using Mfob = optimizer::Mfob;

void ToDebugProto(const std::vector<TrajectoryPoint>& init_traj,
                  const std::vector<TrajectoryPoint>& solver_init_traj,
                  const std::vector<TrajectoryPoint>& result_traj,
                  const OptimizerSolverDebugHook<Mfob>& solver_debug_hook,
                  const DdpOptimizerDebugProto::SolverInitialTrajectorySource&
                      solver_init_traj_source,
                  TrajectoryOptimizerDebugProto* traj_opt_debug_proto,
                  ThreadPool* thread_pool) {
  SCOPED_QTRACE("TrajectoryOptimizer/ToDebugProto");
  // Write DdpDebugProto data.
  DdpOptimizerDebugProto* ddp_debug = traj_opt_debug_proto->mutable_ddp();
  ddp_debug->mutable_init_traj()->Reserve(init_traj.size());
  for (int k = 0; k < init_traj.size(); ++k) {
    init_traj[k].ToProto(ddp_debug->add_init_traj());
  }

  ddp_debug->mutable_solver_initial_trajectory()->Reserve(
      solver_init_traj.size());
  for (int k = 0; k < solver_init_traj.size(); ++k) {
    solver_init_traj[k].ToProto(ddp_debug->add_solver_initial_trajectory());
  }
  ddp_debug->set_solver_initial_trajectory_source(solver_init_traj_source);

  ddp_debug->mutable_final_traj()->Reserve(result_traj.size());
  for (int k = 0; k < result_traj.size(); ++k) {
    result_traj[k].ToProto(ddp_debug->add_final_traj());
  }

  const auto& init_costs = solver_debug_hook.init_costs;
  ddp_debug->mutable_init_costs()->set_cost(init_costs.cost);
  for (int i = 0; i < init_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ddp_debug->mutable_init_costs()->add_costs();
    cost_proto->set_name(init_costs.ddp_costs[i].name);
    cost_proto->set_cost(init_costs.ddp_costs[i].value);
  }
  const auto& final_costs = solver_debug_hook.final_costs;
  ddp_debug->mutable_final_costs()->set_cost(final_costs.cost);
  for (int i = 0; i < final_costs.ddp_costs.size(); ++i) {
    TrajectoryOptimizerCost* cost_proto =
        ddp_debug->mutable_final_costs()->add_costs();
    cost_proto->set_name(final_costs.ddp_costs[i].name);
    cost_proto->set_cost(final_costs.ddp_costs[i].value);
  }

  const int num_iters = solver_debug_hook.iterations.size();
  ddp_debug->mutable_iterations()->Reserve(num_iters);
  for (int i = 0; i < num_iters; ++i) {
    ddp_debug->add_iterations();
  }

  ParallelFor(0, num_iters, thread_pool, [&](int i) {
    const auto& iteration = solver_debug_hook.iterations[i];
    DdpOptimizerDebugProto::Iteration* iteration_proto =
        ddp_debug->mutable_iterations()->Mutable(i);
    QCHECK_EQ(iteration.alphas.size(), iteration.line_search_costs.size());
    for (int i = 0; i < iteration.alphas.size(); ++i) {
      iteration_proto->add_line_search_alphas(iteration.alphas[i]);
      iteration_proto->add_line_search_costs(iteration.line_search_costs[i]);
    }
    QCHECK_EQ(iteration.k_s.size(), iteration.stepsize_adjustment_costs.size());
    for (int i = 0; i < iteration.k_s.size(); ++i) {
      iteration_proto->add_step_size_adjustment_ks(iteration.k_s[i]);
      iteration_proto->add_step_size_adjustment_costs(
          iteration.stepsize_adjustment_costs[i]);
    }
    iteration_proto->set_final_cost(iteration.final_cost);
    iteration_proto->set_js0(iteration.js0);
  });

  ddp_debug->set_num_iters(num_iters);

  for (const auto& response : solver_debug_hook.object_responses) {
    *traj_opt_debug_proto->add_object_responses() = response;
  }
}

absl::Status CheckInputQuality(const TrajectoryOptimizerInput& input) {
  constexpr int kMinTrajectoryLength = 10;  // 1s.
  if (input.trajectory.size() < kMinTrajectoryLength) {
    return absl::FailedPreconditionError(
        absl::StrFormat("Input trajectory is not long enough: %d time steps.",
                        input.trajectory.size()));
  }
  return absl::OkStatus();
}

void AddRegularizersCost(
    const std::vector<TrajectoryPoint>& init_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  const Mfob::StateType x0 = Mfob::FitInitialState(init_traj);
  const Mfob::ControlsType init_us = Mfob::FitControl(init_traj, x0);
  const Mfob::StatesType init_xs = Mfob::FitState(init_traj);

  std::vector<double> state_regularization_weights(init_xs.size(), 0.0);
  std::vector<double> control_regularization_weights(init_us.size(), 0.0);
  for (int i = 0; i < init_traj.size(); ++i) {
    for (int j = 0; j < Mfob::kStateSize; ++j) {
      state_regularization_weights[i * Mfob::kStateSize + j] =
          cost_weight_params.state_regularization_coeffs().w(j);
    }
    for (int j = 0; j < Mfob::kControlSize; ++j) {
      control_regularization_weights[i * Mfob::kControlSize + j] =
          cost_weight_params.control_regularization_coeffs().w(j);
    }
  }
  costs->emplace_back(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      init_xs, std::move(state_regularization_weights),
      "MfobReferenceStateDeviationCost: StateRegularization",
      cost_weight_params.state_regularization_coeffs().multiplier(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<ReferenceControlDeviationCost<Mfob>>(
      init_us, std::move(control_regularization_weights),
      "MfobReferenceControlDeviationCost: ControlRegularization",
      cost_weight_params.control_regularization_coeffs().multiplier(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddAccelAndJerkCost(
    const TrajectoryPoint& plan_start_point,
    const PlannerSemanticMapManager& psmm,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const std::optional<double>& extra_curb_buffer,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  std::vector<double> accel_cascade_buffers;
  std::vector<double> accel_cascade_gains;
  std::vector<double> decel_cascade_buffers;
  std::vector<double> decel_cascade_gains;
  for (const auto& cascade :
       cost_weight_params.longitudinal_acceleration_cost_params()
           .accel_cascade()) {
    accel_cascade_buffers.push_back(cascade.buffer());
    accel_cascade_gains.push_back(cascade.gain());
  }
  for (const auto& cascade :
       cost_weight_params.longitudinal_acceleration_cost_params()
           .decel_cascade()) {
    decel_cascade_buffers.push_back(cascade.buffer());
    decel_cascade_gains.push_back(cascade.gain());
  }
  constexpr double kAccelerationBufferRatio = 1.0;
  constexpr double kJerkBufferRatio = 0.75;
  costs->emplace_back(std::make_unique<LongitudinalAccelerationCost<Mfob>>(
      motion_constraint_params.max_acceleration() * kAccelerationBufferRatio,
      motion_constraint_params.max_deceleration() * kAccelerationBufferRatio,
      std::move(accel_cascade_buffers), std::move(accel_cascade_gains),
      std::move(decel_cascade_buffers), std::move(decel_cascade_gains),
      "MfobLongitudinalAccelerationCost",
      cost_weight_params.longitudinal_acceleration_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<LateralAccelerationCost<Mfob>>(
      /*using_hessian_approximate=*/true, "MfobLateralAccelerationCost",
      cost_weight_params.lateral_acceleration_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  // TODO(renjie, runbing): Try to use MfobLongitudinalJerkCost.
  costs->emplace_back(std::make_unique<IntrinsicJerkCost<Mfob>>(
      motion_constraint_params.max_accel_jerk() * kJerkBufferRatio,
      motion_constraint_params.max_decel_jerk() * kJerkBufferRatio,
      "MfobIntrinsicJerkCost", cost_weight_params.intrinsic_jerk_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<MfobLateralJerkCost<Mfob>>(
      /*using_hessian_approximate=*/true, "MfobLateralJerkCost",
      cost_weight_params.lateral_jerk_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  // BANDAID(fengzhuang): This is a hack to prevent fast steering change when AV
  // is close to curb due to control/localization error.
  std::vector<std::pair<double, Vec2d>> plan_start_circles;
  std::vector<std::pair<double, Vec2d>> plan_start_mirror_circles;
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta() +
                                                   circle.angle_to_axis());
    plan_start_circles.emplace_back(
        circle.radius(),
        plan_start_point.pos() + circle.dist_to_rac() * tangent);
  }
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.mirror_circles()) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(plan_start_point.theta() +
                                                   circle.angle_to_axis());
    plan_start_mirror_circles.emplace_back(
        circle.radius(),
        plan_start_point.pos() + circle.dist_to_rac() * tangent);
  }
  const double min_mirror_height_avg =
      ComputeMinMaxMirrorAverageHeight(veh_geo_params).first;

  constexpr double kNearbyDistance = 10.0;  // m.
  double nearest_dist = kNearbyDistance;
  const std::vector<ImpassableBoundaryInfo> boundaries_info =
      psmm.GetImpassableBoundariesInfoAtLevel(
          psmm.GetLevel(), plan_start_point.pos(), kNearbyDistance);
  for (const auto& boundary_info : boundaries_info) {
    const bool consider_mirrors =
        boundary_info.height.has_value()
            ? (boundary_info.height.value() > min_mirror_height_avg)
            : trajectory_optimizer_vehicle_model_params
                  .consider_mirrors_by_default();
    for (const auto& circle : plan_start_circles) {
      nearest_dist = std::min(
          nearest_dist,
          boundary_info.segment.DistanceTo(circle.second) - circle.first);
    }
    if (consider_mirrors) {
      for (const auto& circle : plan_start_mirror_circles) {
        nearest_dist = std::min(
            nearest_dist,
            boundary_info.segment.DistanceTo(circle.second) - circle.first);
      }
    }
  }

  double penetration_distance = 0.0;
  const double extra_buffer =
      extra_curb_buffer.has_value() ? *extra_curb_buffer : 0.0;
  if (FLAGS_msd_static_boundary_cost_v2) {
    const auto speed_buffer_plf = PiecewiseLinearFunctionFromProto(
        cost_weight_params.speed_rel_hard_curb_clearance_plf());
    penetration_distance =
        std::max(0.0, extra_buffer + speed_buffer_plf(plan_start_point.v()) -
                          nearest_dist);
  }
  const auto penetration_gain_plf = PiecewiseLinearFunctionFromProto(
      cost_weight_params.curb_penetration_lateral_snap_gain_plf());
  const double lateral_snap_gain = penetration_gain_plf(penetration_distance);
  if (lateral_snap_gain > 1.0) {
    QEVENT_EVERY_N_SECONDS("fengzhuang", "lateral_snap_curb_gain_triggered",
                           5.0, [&](QEvent* qevent) {
                             qevent->AddField("penetration_dist_m",
                                              penetration_distance);
                           });
  }

  const PiecewiseLinearFunction<double> speed_relative_gain =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.lateral_snap_speed_gain_plf());
  costs->emplace_back(std::make_unique<MfobIntrinsicLateralSnapCost<Mfob>>(
      "MfobIntrinsicLateralSnapCost",
      lateral_snap_gain * cost_weight_params.intrinsic_lateral_snap_weight() *
          speed_relative_gain(plan_start_point.v()),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddCurvatureCost(
    double trajectory_time_step,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  FUNC_QTRACE();
  QCHECK_GT(trajectory_time_step, 0.0);
  const int curvature_limit_index =
      static_cast<int>(kCurvatureLimitRange / trajectory_time_step);

  constexpr double kCurvatureBufferRatio = 0.98;
  constexpr double kCurvatureRateBufferRatio = 0.75;
  costs->emplace_back(std::make_unique<CurvatureCost<Mfob>>(
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params) *
          kCurvatureBufferRatio,
      curvature_limit_index, "MfobCurvatureCost",
      cost_weight_params.curvature_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<MfobCurvatureRateCost<Mfob>>(
      motion_constraint_params.max_psi() * kCurvatureRateBufferRatio,
      "MfobCurvatureRateCost", cost_weight_params.curvature_rate_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  costs->emplace_back(std::make_unique<MfobCurvatureRateRateCost<Mfob>>(
      motion_constraint_params.max_chi(), "MfobCurvatureRateRateCost",
      cost_weight_params.curvature_rate_rate_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddImmediateFutureCost(
    int trajectory_steps, double trajectory_time_step,
    const std::vector<TrajectoryPoint>& prev_traj,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  FUNC_QTRACE();
  if (prev_traj.size() < 2) return;
  const Mfob::StateType x0 = Mfob::FitInitialState(prev_traj);
  const Mfob::ControlsType ref_us = Mfob::FitControl(prev_traj, x0);
  const Mfob::StatesType ref_xs = Mfob::FitState(prev_traj);

  // Make reference variables
  std::vector<double> ref_jerk;
  std::vector<double> ref_s;
  std::vector<double> ref_kappa;
  ref_jerk.reserve(trajectory_steps);
  ref_s.reserve(trajectory_steps);
  ref_kappa.reserve(trajectory_steps);
  for (int i = 0; i < trajectory_steps; ++i) {
    ref_jerk.push_back(Mfob::j(ref_us, i));
    ref_s.push_back(Mfob::s(ref_xs, i));
    ref_kappa.push_back(Mfob::kappa(ref_xs, i));
  }

  constexpr double kImmediateJGain = 0.1;
  const PiecewiseLinearFunction<double> lon_weight_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.immediate_future_cost_params().lon_weight_plf());
  std::vector<double> lon_weights;
  lon_weights.reserve(trajectory_steps);
  QCHECK_EQ(prev_traj.size(), trajectory_steps);
  for (int i = 0; i < trajectory_steps; ++i) {
    const auto& prev_traj_point = prev_traj[i];
    const double lon_weight_plf_t = lon_weight_plf(prev_traj_point.t());
    lon_weights.push_back(kImmediateJGain * lon_weight_plf_t);
  }
  costs->push_back(
      std::make_unique<ReferenceLongitudinalJerkDeviationCost<Mfob>>(
          std::move(ref_jerk), std::move(lon_weights),
          "MfobReferenceLongitudinalJerkDeviationCost: ImmediateFuture",
          cost_weight_params.immediate_future_cost_weight(),
          /*cost_type=*/Cost<Mfob>::CostType::GROUP_IMMEDIATE_FUTURE));

  constexpr double kCurvatureDeviationCostWeight = 500.0;
  const PiecewiseLinearFunction<double> lat_weight_plf =
      PiecewiseLinearFunctionFromProto(
          cost_weight_params.immediate_future_cost_params().lat_weight_plf());
  std::vector<double> lat_weights;
  lat_weights.reserve(trajectory_steps);
  for (int i = 0; i < trajectory_steps; ++i) {
    lat_weights.push_back(
        lat_weight_plf(static_cast<double>(i * trajectory_time_step)));
  }
  costs->push_back(std::make_unique<CurvatureDeviationCost<Mfob>>(
      ref_s, ref_kappa, std::move(lat_weights),
      "MfobCurvatureDeviationCost: ImmediateFuture",
      cost_weight_params.curvature_deviation_immediate_future_cost_weight() *
          kCurvatureDeviationCostWeight));
}

void GetReferencePathGainForRouteDestinationStopLine(
    const DrivePassage& drive_passage,
    const ConstraintManager& constraint_manager,
    const VehicleGeometryParamsProto& veh_geo_params,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<double>* ref_path_deviation_gains,
    std::vector<double>* ref_heading_deviation_gains) {
  const auto& stop_lines = constraint_manager.StopLine();
  QCHECK_EQ(ref_path_deviation_gains->size(),
            ref_heading_deviation_gains->size());

  std::optional<double> route_destination_stop_line_s;
  for (const auto& stop_line : stop_lines) {
    if (stop_line.source().type_case() ==
        SourceProto::TypeCase::kRouteDestination) {
      route_destination_stop_line_s = stop_line.s() - stop_line.standoff() -
                                      veh_geo_params.front_edge_to_center();
      break;
    }
  }
  if (!route_destination_stop_line_s.has_value()) {
    return;
  }

  const int count = ref_path_deviation_gains->size();
  const StationIndex route_destination_index =
      drive_passage.FindNearestStationIndexAtS(*route_destination_stop_line_s);
  for (int k = route_destination_index.value(); k < count; ++k) {
    (*ref_path_deviation_gains)[k] =
        cost_weight_params.reference_path_cost_weight().destination_path_gain();
    (*ref_heading_deviation_gains)[k] =
        cost_weight_params.reference_path_cost_weight()
            .destination_theta_gain();
  }
}

void AddReferencePathCost(
    int trajectory_steps, const PathSlBoundary& path_sl_boundary,
    std::vector<double> ref_path_deviation_gains,
    std::vector<double> ref_heading_deviation_gains,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>&
        reference_center_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  FUNC_QTRACE();
  std::vector<double> ref_ls(
      path_sl_boundary.reference_center_l_vector().begin(),
      path_sl_boundary.reference_center_l_vector().end());
  // ref_ls.size() should be the same with ref_path_deviation_gains.size().
  ref_ls.pop_back();
  costs->emplace_back(std::make_unique<ReferenceLineDeviationCost<Mfob>>(
      trajectory_steps,
      cost_weight_params.reference_path_cost_weight().path_gain(),
      cost_weight_params.reference_path_cost_weight().end_state_gain(),
      std::move(ref_ls), reference_center_query_helper->points(),
      reference_center_query_helper.get(), std::move(ref_path_deviation_gains),
      "MfobReferenceLineDeviationCost",
      cost_weight_params.reference_path_cost_weight()
          .reference_path_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));

  // Compute ref_thetas.
  const std::vector<Vec2d> ref_path_vector(
      path_sl_boundary.reference_center_xy_vector().begin(),
      path_sl_boundary.reference_center_xy_vector().end());
  std::vector<double> ref_thetas;
  ref_thetas.reserve(ref_path_vector.size() - 1);
  for (int i = 1; i < ref_path_vector.size(); ++i) {
    ref_thetas.push_back(
        (ref_path_vector[i] - ref_path_vector[i - 1]).FastAngle());
  }
  costs->emplace_back(std::make_unique<EndHeadingCost<Mfob>>(
      trajectory_steps, std::move(ref_thetas),
      reference_center_query_helper->points(),
      reference_center_query_helper.get(),
      std::move(ref_heading_deviation_gains), "MfobEndHeadingCost",
      cost_weight_params.reference_path_cost_weight()
          .reference_heading_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddCaptainReferenceTrajectoryCost(
    int trajectory_steps, const std::vector<TrajectoryPoint>& captain_traj,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  if (captain_traj.empty()) return;
  // If captain trajectory size is smaller than ddp horizon, fill a pseudo
  // trajectory up to the horizon.
  auto captain_ref_traj = captain_traj;
  while (captain_ref_traj.size() < trajectory_steps) {
    captain_ref_traj.push_back(TrajectoryPoint());
  }
  auto ref_xs = Mfob::FitState(captain_ref_traj);
  std::vector<double> captain_ref_weights(ref_xs.size(), 0.0);
  // TODO(renjie): Move the weights to params and tune them.
  // TODO(renjie): Add more weights if captain trajectory provides more
  // quantities.
  constexpr double kCaptainRefPosWeight = 1.0;
  constexpr double kCaptainRefHeadingWeight = 5.0;
  for (int i = 0; i < captain_traj.size(); ++i) {
    captain_ref_weights[i * Mfob::kStateSize + Mfob::kStateXIndex] =
        kCaptainRefPosWeight;
    captain_ref_weights[i * Mfob::kStateSize + Mfob::kStateYIndex] =
        kCaptainRefPosWeight;
    captain_ref_weights[i * Mfob::kStateSize + Mfob::kStateThetaIndex] =
        kCaptainRefHeadingWeight;
  }
  costs->emplace_back(std::make_unique<ReferenceStateDeviationCost<Mfob>>(
      std::move(ref_xs), std::move(captain_ref_weights),
      "MfobReferenceStateDeviationCost: CaptainReference",
      /*scale=*/1.0,
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

void AddReferenceEndStateCost(
    int trajectory_steps, double ref_end_state_s,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  const auto& end_state_cost_params =
      cost_weight_params.end_state_cost_params();
  Mfob::StateType end_state = Mfob::StateType::Zero();
  end_state[Mfob::kStateSIndex] = ref_end_state_s;

  std::vector<double> state_regularization_weights(end_state.size(), 0.0);
  std::vector<double> base_numbers(end_state.size(),
                                   std::numeric_limits<double>::infinity());
  for (int i = 0; i < Mfob::kStateSize; ++i) {
    state_regularization_weights[i] = end_state_cost_params.w(i);
    base_numbers[i] = end_state_cost_params.base_numbers(i);
  }
  costs->emplace_back(std::make_unique<ReferenceSingleStateDeviationCost<Mfob>>(
      std::move(end_state), /*ref_index=*/trajectory_steps - 1,
      std::move(state_regularization_weights), std::move(base_numbers),
      "MfobReferenceStateDeviationCost: EndStateAttraction",
      /*scale=*/end_state_cost_params.weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
}

absl::Status AddCosts(
    int trajectory_steps, double trajectory_time_step,
    std::string_view base_name,
    const std::vector<TrajectoryPoint>& initializer_traj,
    const std::vector<TrajectoryPoint>& prev_traj,
    const std::vector<TrajectoryPoint>& captain_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const ConstraintManager& constraint_manager,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const PlannerSemanticMapManager& psmm,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const TrajectoryOptimizerCostConfigProto& cost_config,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const PlannerFunctionsParamsProto& planner_functions_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::unique_ptr<AvModelHelper<Mfob>>& av_model_helpers,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs, ThreadPool* thread_pool) {
  FUNC_QTRACE();
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);

  if (cost_config.enable_regularizers_cost()) {
    AddRegularizersCost(initializer_traj, cost_weight_params, costs);
  }
  if (cost_config.enable_curvature_cost()) {
    AddCurvatureCost(trajectory_time_step, cost_weight_params, veh_geo_params,
                     vehicle_drive_params, motion_constraint_params, costs);
  }
  if (cost_config.enable_forward_speed_cost()) {
    // Forward speed cost.
    costs->emplace_back(std::make_unique<ForwardSpeedCost<Mfob>>(
        "MfobForwardSpeedCost", cost_weight_params.forward_speed_cost_weight(),

        /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE));
  }
  if (cost_config.enable_immediate_future_cost()) {
    AddImmediateFutureCost(trajectory_steps, trajectory_time_step, prev_traj,
                           cost_weight_params, costs);
  }

  std::vector<double> left_l_boundary_for_nudge(path_sl_boundary.size(), 0.0);
  std::vector<double> right_l_boundary_for_nudge(path_sl_boundary.size(), 0.0);
  std::vector<optimizer::LeadingInfo> leading_min_s(
      free_index + 1, {
                          std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(),
                      });

  std::vector<Vec2d> init_points;
  init_points.reserve(initializer_traj.size());
  for (const auto& pt : initializer_traj) {
    init_points.push_back(pt.pos());
  }
  const auto init_traj_frenet_frame =
      BuildKdTreeFrenetFrame(init_points,
                             /*down_sample_raw_points=*/true);
  if (!init_traj_frenet_frame.ok()) {
    return absl::OutOfRangeError("Init_frenet_frame build failed.");
  }

  const auto path_time_corridor = optimizer::BuildPathTimeCorridor(
      base_name, initializer_traj, drive_passage, path_sl_boundary,
      *init_traj_frenet_frame, leading_trajs, st_planner_object_traj,
      veh_geo_params);
  if (!path_time_corridor.ok()) {
    return absl::OkStatus();
  }

  if (cost_config.enable_object_cost()) {
    optimizer::AddObjectCosts(
        trajectory_steps, trajectory_time_step, base_name, initializer_traj,
        drive_passage, path_sl_boundary, *path_time_corridor,
        *init_traj_frenet_frame, leading_trajs, st_planner_object_traj,
        cost_weight_params, veh_geo_params, motion_constraint_params,
        trajectory_optimizer_vehicle_model_params, av_model_helpers,
        &leading_min_s, &left_l_boundary_for_nudge, &right_l_boundary_for_nudge,
        costs, thread_pool);
  }

  double ref_end_state_s = std::numeric_limits<double>::infinity();
  if (cost_config.enable_speed_limit_cost()) {
    optimizer::AddSpeedLimitCost(
        trajectory_steps, trajectory_time_step, initializer_traj.front(),
        drive_passage, constraint_manager, cost_weight_params,
        motion_constraint_params, veh_geo_params, stations_query_helper,
        leading_min_s, &ref_end_state_s, costs);
  }

  if (cost_config.enable_reference_end_state_cost()) {
    AddReferenceEndStateCost(trajectory_steps, ref_end_state_s,
                             cost_weight_params, costs);
  }

  if (cost_config.enable_reference_path_cost()) {
    std::vector<double> ref_path_deviation_gains(
        stations_query_helper->points().size() - 1, 1.0);
    std::vector<double> ref_heading_deviation_gains(
        stations_query_helper->points().size() - 1, 1.0);
    GetReferencePathGainForRouteDestinationStopLine(
        drive_passage, constraint_manager, veh_geo_params, cost_weight_params,
        &ref_path_deviation_gains, &ref_heading_deviation_gains);
    AddReferencePathCost(trajectory_steps, path_sl_boundary,
                         std::move(ref_path_deviation_gains),
                         std::move(ref_heading_deviation_gains),
                         cost_weight_params, stations_query_helper, costs);
  }
  std::optional<double> extra_curb_buffer = std::nullopt;
  if (cost_config.enable_static_boundary_cost()) {
    const bool enable_three_point_turn =
        (FLAGS_planner_runtime_uturn_level == 2 ||
         (FLAGS_planner_runtime_uturn_level == 1 &&
          planner_functions_params.enable_three_point_turn()));
    // Set curb cost type.
    optimizer::CurbCostType curb_cost_type;
    if (FLAGS_use_static_boundary_cost_in_vision_map && psmm.IsOnVisionMap()) {
      curb_cost_type = optimizer::CurbCostType::kStaticBoundary;
    } else {
      curb_cost_type = FLAGS_msd_static_boundary_cost_v2
                           ? optimizer::CurbCostType::kMsdV2
                           : optimizer::CurbCostType::kMsdV1;
    }
    // Curb and path boundary costs.
    optimizer::AddStaticBoundaryCosts(
        trajectory_steps, base_name, enable_three_point_turn,
        initializer_traj.front(), drive_passage, psmm, path_sl_boundary,
        *path_time_corridor, left_l_boundary_for_nudge,
        right_l_boundary_for_nudge, cost_weight_params, veh_geo_params,
        trajectory_optimizer_vehicle_model_params, stations_query_helper,
        curb_cost_type, &extra_curb_buffer, costs);
  }
  if (cost_config.enable_acceleration_and_jerk_cost()) {
    AddAccelAndJerkCost(initializer_traj.front(), psmm,
                        trajectory_optimizer_vehicle_model_params,
                        veh_geo_params, cost_weight_params,
                        motion_constraint_params, extra_curb_buffer, costs);
  }
  if (cost_config.enable_captain_reference_trajectory_cost()) {
    AddCaptainReferenceTrajectoryCost(trajectory_steps, captain_traj, costs);
  }
  return absl::OkStatus();
}

absl::Status AddStationQueryHelper(
    int trajectory_steps,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const DrivePassage& drive_passage,
    std::unique_ptr<CenterLineQueryHelper<Mfob>>* station_query_helper) {
  FUNC_QTRACE();
  std::vector<VehicleCircleModelParamsProto::CircleParams> query_circles;
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    if (circle.type() == VehicleCircleModelParamsProto::REAR_AXIS_CENTER ||
        circle.type() == VehicleCircleModelParamsProto::MID_AXIS_CENTER ||
        circle.type() == VehicleCircleModelParamsProto::FRONT_AXIS_CENTER) {
      query_circles.push_back(circle);
    }
  }
  *station_query_helper = std::make_unique<CenterLineQueryHelper<Mfob>>(
      trajectory_steps, std::move(query_circles), drive_passage.frenet_frame(),
      drive_passage.last_real_station_index().value(),
      "MfobStationQueryHelper");
  return absl::OkStatus();
}

std::unique_ptr<AvModelHelper<Mfob>> AddAvModelHelpers(
    int trajectory_steps, const VehicleCircleModelParamsProto&
                              trajectory_optimizer_vehicle_model_params) {
  FUNC_QTRACE();
  std::vector<double> dists_to_rac;
  std::vector<double> angles_to_axis;
  const int circle_size =
      trajectory_optimizer_vehicle_model_params.circles_size() +
      trajectory_optimizer_vehicle_model_params.mirror_circles_size();
  dists_to_rac.reserve(circle_size);
  angles_to_axis.reserve(circle_size);
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.circles()) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
  }
  for (const auto& circle :
       trajectory_optimizer_vehicle_model_params.mirror_circles()) {
    dists_to_rac.push_back(circle.dist_to_rac());
    angles_to_axis.push_back(circle.angle_to_axis());
  }
  return std::make_unique<AvModelHelper<Mfob>>(
      trajectory_steps, dists_to_rac, angles_to_axis, "MfobAvModelHelper");
}

// Extend by PurePursuit with a constant speed.
std::vector<TrajectoryPoint> GetExtendStateByPurePursuit(
    double trajectory_time_step, const TrajectoryPoint& extend_start_point,
    int k_extend_steps, double target_v, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& veh_geo_params) {
  QCHECK_GT(trajectory_time_step, 0.0);
  // Extend by PurePursuit with a constant speed. Copied from
  // GetExtendStateByPurePursuit.
  constexpr double kLateralLookAhead = 1.0;
  Mfob::StateType x = Mfob::MakeState(
      extend_start_point.pos().x(), extend_start_point.pos().y(),
      extend_start_point.theta(), extend_start_point.v(),
      extend_start_point.kappa(), extend_start_point.a(),
      extend_start_point.psi(), extend_start_point.s());
  std::vector<TrajectoryPoint> res;
  const double time_base = extend_start_point.t();
  while (res.size() < k_extend_steps) {
    const double a = (target_v - Mfob::StateGetV(x)) / trajectory_time_step;
    const double j = (a - Mfob::StateGetA(x)) / trajectory_time_step;
    Vec2d lateral_target_pos = drive_passage.stations().back().xy();
    const double lateral_look_ahead_dist =
        kLateralLookAhead * Mfob::StateGetV(x) + veh_geo_params.wheel_base();
    const auto lateral_nearest_point_status =
        drive_passage.QueryFrenetCoordinateAt(Mfob::StateGetPos(x));
    if (lateral_nearest_point_status.ok()) {
      lateral_target_pos = path_sl_boundary.QueryReferenceCenterXY(
          lateral_look_ahead_dist + lateral_nearest_point_status->s);
    }
    const double alpha =
        Vec2d(lateral_target_pos - Mfob::StateGetPos(x)).Angle() -
        Mfob::StateGetTheta(x);
    const double kappa = 2.0 * fast_math::Sin(alpha) / lateral_look_ahead_dist;
    const double psi = (kappa - Mfob::StateGetKappa(x)) / trajectory_time_step;

    const double chi = (psi - Mfob::StateGetPsi(x)) / trajectory_time_step;

    const auto u = Mfob::MakeControl(chi, j);
    x = Mfob::EvaluateF(res.size() - 1, x, u, trajectory_time_step);

    TrajectoryPoint next_pt;
    next_pt.set_pos(Mfob::StateGetPos(x));
    next_pt.set_theta(Mfob::StateGetTheta(x));
    next_pt.set_kappa(Mfob::StateGetKappa(x));
    next_pt.set_psi(Mfob::StateGetPsi(x));
    next_pt.set_v(Mfob::StateGetV(x));
    next_pt.set_a(Mfob::StateGetA(x));
    next_pt.set_s(Mfob::StateGetS(x));
    next_pt.set_t((res.empty() ? time_base : res.back().t()) +
                  trajectory_time_step);
    res.push_back(std::move(next_pt));
  }
  return res;
}

// Convert an input trajectory with time step kTrajectoryTimeStep to comply
// with trajectory optimizer input format: horizon target_trajectory_steps and
// time step target_trajectory_time_step.
std::vector<TrajectoryPoint> ToTrajectoryOptimizerInput(
    const std::vector<TrajectoryPoint>& input_traj, int target_trajectory_steps,
    double target_trajectory_time_step, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params) {
  QCHECK_GT(target_trajectory_time_step, 0.0);
  QCHECK_GT(target_trajectory_steps, 0);
  const int sample_step =
      static_cast<int>(target_trajectory_time_step / kTrajectoryTimeStep + 0.5);

  if (input_traj.empty()) return input_traj;
  std::vector<TrajectoryPoint> res;
  res.reserve(target_trajectory_steps);
  for (int i = 0; i < target_trajectory_steps; ++i) {
    if (i * sample_step >= input_traj.size()) break;
    res.push_back(input_traj[i * sample_step]);
  }
  if (res.size() == target_trajectory_steps) return res;

  const double input_traj_end_v = input_traj.back().v();
  const auto traj_end_sl_status =
      drive_passage.QueryFrenetCoordinateAt(input_traj.back().pos());
  if (!traj_end_sl_status.ok() ||
      (traj_end_sl_status->s > path_sl_boundary.end_s())) {
    QLOG(ERROR) << "Input trajectory end is out of drive passage or trajectory "
                   "end if out of path boundary: "
                << input_traj.back().DebugString();
    // Extend by constant acceleration and curvature.
    constexpr double kMinSpeed = 1e-6;
    constexpr double kMinDist = 1e-6;
    while (res.size() < target_trajectory_steps) {
      auto& pt = res.back();
      const double dist = std::max(
          kMinDist, pt.v() * target_trajectory_time_step +
                        0.5 * pt.a() * Sqr(target_trajectory_time_step));
      PathPoint path_point;
      path_point.set_x(pt.pos().x());
      path_point.set_y(pt.pos().y());
      path_point.set_s(pt.s());
      path_point.set_theta(pt.theta());
      path_point.set_kappa(pt.kappa());
      path_point.set_lambda(pt.lambda());
      const auto next_path_point = GetPathPointAlongCircle(path_point, dist);
      auto next_pt = pt;
      next_pt.set_t(pt.t() + target_trajectory_time_step);
      next_pt.set_pos(ToVec2d(next_path_point));
      next_pt.set_s(next_path_point.s());
      next_pt.set_theta(next_path_point.theta());
      next_pt.set_kappa(next_path_point.kappa());
      next_pt.set_v(
          std::max(kMinSpeed, pt.v() + pt.a() * target_trajectory_time_step));
      next_pt.set_a(
          std::max(pt.a(), -1.0 * next_pt.v() / target_trajectory_time_step));
      pt.set_j(
          std::clamp((next_pt.a() - next_pt.a()) / target_trajectory_time_step,
                     motion_constraint_params.max_decel_jerk(),
                     motion_constraint_params.max_accel_jerk()));
      next_pt.set_psi(next_path_point.lambda() * next_pt.v());
      res.push_back(std::move(next_pt));
    }
    res.back().set_j((res.rbegin() + 1)->j());
    return res;
  }

  const double time_to_extend =
      target_trajectory_time_step * (target_trajectory_steps - res.size() + 1);
  constexpr double kMinEndSpeed = 2.0;  // m/s.
  const double target_v = std::min(
      std::max(kMinEndSpeed, input_traj_end_v),
      (path_sl_boundary.end_s() - traj_end_sl_status->s) / time_to_extend);

  const auto extend_state = GetExtendStateByPurePursuit(
      target_trajectory_time_step, res.back(),
      target_trajectory_steps - res.size(), target_v, drive_passage,
      path_sl_boundary, veh_geo_params);

  for (int k = 0; k < extend_state.size(); ++k) {
    res.push_back(extend_state[k]);
  }
  return res;
}

// Try to generate a solver init trajectory using
// last optimization result, according to the current
// problem to be solved.
// If such generation failed, std::nullopt will be returned.
std::optional<std::vector<TrajectoryPoint>> BuildUsablePrevInitTrajectory(
    int trajectory_steps, double trajectory_time_step, const Mfob& problem,
    const DdpOptimizerParamsProto& params, absl::Time plan_start_time,
    absl::Time last_plan_start_time,
    const std::vector<TrajectoryPoint>& initializer_traj,
    const std::vector<TrajectoryPoint>& last_optimized_traj,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  // Check input.
  if (initializer_traj.empty()) {
    return std::nullopt;
  }
  if (last_optimized_traj.size() < trajectory_steps) {
    return std::nullopt;
  }

  constexpr double kMaxShiftTime = 1.0;
  const absl::Duration dt = plan_start_time - last_plan_start_time;
  const double shift_time = absl::ToDoubleSeconds(dt);

  if (shift_time < 0 || shift_time > kMaxShiftTime) {
    // Long has passed since last optimization.
    return std::nullopt;
  }

  // Shift last trajectory by time.
  std::vector<TrajectoryPoint> shifted_last_optimized_traj =
      last_optimized_traj;
  ShiftTrajectoryByTime(shift_time, &shifted_last_optimized_traj);

  // Adapt last trajectory to plan start point.
  constexpr double kMaxAdaptionCost = 150.0;
  std::optional<std::vector<TrajectoryPoint>> prev_init_traj =
      optimizer::AdaptTrajectoryToGivenPlanStartPoint(
          trajectory_steps, problem, params, kMaxAdaptionCost,
          initializer_traj.front(), shifted_last_optimized_traj);

  if (prev_init_traj.has_value()) {
    // Prev init is not usable if it is much shorter than init traj.
    const int free_index = static_cast<int>(
        (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);
    constexpr double kTrajLengthTimesThreshold = 2.5;
    if (initializer_traj[free_index].s() >
        kTrajLengthTimesThreshold * (*prev_init_traj)[free_index].s()) {
      return std::nullopt;
    }
    // Prev init is not usable when decision different.
    for (const auto& obj_traj : st_planner_object_traj.trajectories) {
      if (!optimizer::HasSameDecisionOverSpacetimeObject(
              initializer_traj, *prev_init_traj,
              optimizer::SampleObjectStates(
                  trajectory_steps, trajectory_time_step, obj_traj.states()))) {
        return std::nullopt;
      }
    }
  }

  return prev_init_traj;
}

void SetIfInAutoTuningMode(
    const DdpOptimizer<Mfob>& solver,
    const OptimizerSolverDebugHook<Mfob>& solver_debug_hook,
    const std::vector<TrajectoryPoint>& result_points,
    bool optimizer_solve_success, const int trajectory_steps_dat,
    TrajectoryOptimizerOutput* output) {
  if (!FLAGS_auto_tuning_mode) {
    return;
  }
  VLOG(3) << "Final xs after solve end: "
          << solver_debug_hook.solve_end_xs.transpose();
  VLOG(3) << "Final us after solve end: "
          << solver_debug_hook.solve_end_us.transpose();
  *(output->candidate_auto_tuning_traj_proto.mutable_costs()) =
      solver.EvaluateEachDiscountedAccumulativeCost(
          Mfob::GetStatesBeforeStep(solver_debug_hook.solve_end_xs,
                                    trajectory_steps_dat),
          Mfob::GetControlsBeforeStep(solver_debug_hook.solve_end_us,
                                      trajectory_steps_dat),
          trajectory_steps_dat, FLAGS_auto_tuning_gamma);
  output->candidate_auto_tuning_traj_proto.set_valid_for_train(
      optimizer_solve_success);

  if (trajectory_steps_dat <= result_points.size()) {
    auto* traj_proto =
        output->candidate_auto_tuning_traj_proto.mutable_trajectory();
    for (int i = 0; i < trajectory_steps_dat; ++i) {
      result_points[i].ToProto(traj_proto->add_trajectory_point());
    }
  } else {
    QLOG_EVERY_N_SEC(ERROR, 4.0)
        << "Optimizer output trajectory should has at least "
        << trajectory_steps_dat << " points but only has "
        << result_points.size() << " points.";
  }
}

}  // namespace

// NOLINTNEXTLINE
absl::StatusOr<TrajectoryOptimizerOutput> OptimizeTrajectory(
    const TrajectoryOptimizerInput& input,
    TrajectoryOptimizerDebugProto* optimizer_debug,
    vis::vantage::ChartDataBundleProto* charts_data, ThreadPool* thread_pool) {
  SCOPED_QTRACE("EstPlanner/OptimizeTrajectory");

  const absl::Cleanup timeout_trigger = [start_time = absl::Now()]() {
    constexpr double kTimeoutLimitMs = 50.0;
    const double runtime = absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (runtime > kTimeoutLimitMs) {
      QEVENT_EVERY_N_SECONDS(
          "runbing", "optimize_trajectory_timeout", 5.0,
          [&](QEvent* qevent) { qevent->AddField("time_ms", runtime); });
    }
  };

  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.drive_passage);
  QCHECK_NOTNULL(input.path_sl_boundary);
  QCHECK_NOTNULL(input.constraint_mgr);
  QCHECK_NOTNULL(input.leading_trajs);
  QCHECK_NOTNULL(input.planner_semantic_map_mgr);
  QCHECK_NOTNULL(input.st_planner_object_traj);
  QCHECK_NOTNULL(input.captain_trajectory);

  const auto& trajectory_optimizer_params =
      *QCHECK_NOTNULL(input.trajectory_optimizer_params);
  const auto& motion_constraint_params =
      *QCHECK_NOTNULL(input.motion_constraint_params);
  const auto& planner_functions_params =
      *QCHECK_NOTNULL(input.planner_functions_params);
  const auto& vehicle_models_params =
      *QCHECK_NOTNULL(input.vehicle_models_params);
  const auto& veh_geo_params = *QCHECK_NOTNULL(input.veh_geo_params);
  const auto& veh_drive_params = *QCHECK_NOTNULL(input.veh_drive_params);

  if (const auto input_status = CheckInputQuality(input); !input_status.ok()) {
    return input_status;
  }

  const int trajectory_steps = trajectory_optimizer_params.trajectory_steps();
  const double trajectory_time_step =
      trajectory_optimizer_params.trajectory_time_step();
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);

  const auto& plan_start_point = input.plan_start_point;

  optimizer_debug->set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_start_time));

  // Convert from ApolloTrajectoryPoint to TrajectoryPoint.
  // TODO(renjie): Replace with ToTrajectoryPoint once initializer can produce
  // third-order Apollo trajectory.
  std::vector<TrajectoryPoint> initializer_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPointFromSecondOrderApollo(input.trajectory),
      trajectory_steps, trajectory_time_step, *input.drive_passage,
      *input.path_sl_boundary, motion_constraint_params, veh_geo_params);

  // The size of prev_traj should be either 0 or equal to initializer_traj.
  // Note(renjie): make sure that prev_traj has been shifted in time to now.
  const std::vector<TrajectoryPoint> prev_traj = ToTrajectoryOptimizerInput(
      ToTrajectoryPoint(input.previous_trajectory), trajectory_steps,
      trajectory_time_step, *input.drive_passage, *input.path_sl_boundary,
      motion_constraint_params, veh_geo_params);
  QCHECK(prev_traj.empty() || prev_traj.size() == initializer_traj.size());

  ScopedMultiTimer timer("trajectory_optimizer");

  const std::string base_name_with_plan_id =
      absl::StrFormat("traj_opt_%d", input.plan_id);
  const std::string base_name = "traj_opt";

  constexpr double kStartSquareDistThreshold = 1.0;
  constexpr double kStartVDiffThreshold = 2.0;
  const auto& initializer_traj_start_point = initializer_traj.front();
  const double start_square_dist = (Vec2d(plan_start_point.path_point().x(),
                                          plan_start_point.path_point().y()) -
                                    initializer_traj_start_point.pos())
                                       .squaredNorm();
  const double start_v_diff =
      plan_start_point.v() - initializer_traj_start_point.v();
  if (start_square_dist < kStartSquareDistThreshold &&
      std::abs(start_v_diff) < kStartVDiffThreshold) {
    initializer_traj.begin()->FromProto(plan_start_point);
  } else {
    QLOG(FATAL) << "Initializer first state("
                << initializer_traj_start_point.pos().x() << ","
                << initializer_traj_start_point.pos().y() << ","
                << initializer_traj_start_point.v()
                << ") too far from plan start point("
                << plan_start_point.path_point().x() << ","
                << plan_start_point.path_point().y() << ","
                << plan_start_point.v() << ")!";
  }

  const auto& drive_passage = *input.drive_passage;
  const auto& path_sl_boundary = *input.path_sl_boundary;
  const auto& constraint_manager = *input.constraint_mgr;
  const auto& leading_trajs = *input.leading_trajs;
  const auto& st_traj_mgr = *input.st_traj_mgr;
  const auto& st_planner_object_traj = *input.st_planner_object_traj;

  // Build stations_query_helper and av_model_helpers.
  std::unique_ptr<CenterLineQueryHelper<Mfob>> stations_query_helper;
  QCHECK_OK(AddStationQueryHelper(
      trajectory_steps,
      vehicle_models_params.trajectory_optimizer_vehicle_model_params(),
      drive_passage, &stations_query_helper));

  std::unique_ptr<AvModelHelper<Mfob>> av_model_helpers = AddAvModelHelpers(
      trajectory_steps,
      vehicle_models_params.trajectory_optimizer_vehicle_model_params());

  // Add problem costs.
  auto add_cost_start_time = absl::Now();
  std::vector<std::unique_ptr<Cost<Mfob>>> costs;
  QCHECK_OK(AddCosts(
      trajectory_steps, trajectory_time_step, base_name_with_plan_id,
      initializer_traj, prev_traj, *input.captain_trajectory, drive_passage,
      path_sl_boundary, constraint_manager, leading_trajs,
      st_planner_object_traj, *input.planner_semantic_map_mgr,
      trajectory_optimizer_params.cost_weight_params(),
      trajectory_optimizer_params.cost_config(), veh_geo_params,
      veh_drive_params, motion_constraint_params,
      vehicle_models_params.trajectory_optimizer_vehicle_model_params(),
      planner_functions_params, stations_query_helper, av_model_helpers, &costs,
      thread_pool));

  const double add_cost_time =
      absl::ToDoubleMilliseconds(absl::Now() - add_cost_start_time);

  // Build problem.
  auto problem = std::make_unique<Mfob>(
      &motion_constraint_params, &veh_geo_params, &veh_drive_params,
      trajectory_steps, trajectory_time_step,
      /*enable_dynamic_2nd_derivatives=*/false,
      /*enable_post_process=*/false);
  problem->AddCostHelper(std::move(stations_query_helper));
  problem->AddCostHelper(std::move(av_model_helpers));
  for (auto& cost : costs) {
    problem->AddCost(std::move(cost));
  }

  // Build solver init trajectory.
  std::string solver_init_traj_name = "none";
  DdpOptimizerDebugProto::SolverInitialTrajectorySource solver_init_traj_source;
  std::optional<std::vector<TrajectoryPoint>> solver_init_traj;
  // First try prev_init trajectory.
  if (FLAGS_traj_opt_init_traj_uses_last_optimized_trajectory) {
    if (input.trajectory_optimizer_state.has_value() &&
        input.trajectory_optimizer_state->last_optimized_trajectory.size() >
            0) {
      solver_init_traj = BuildUsablePrevInitTrajectory(
          trajectory_steps, trajectory_time_step, *problem,
          trajectory_optimizer_params.optimizer_params(), input.plan_start_time,
          input.trajectory_optimizer_state->last_plan_start_time,
          initializer_traj,
          input.trajectory_optimizer_state->last_optimized_trajectory,
          st_planner_object_traj);
      if (solver_init_traj.has_value()) {
        solver_init_traj_source = DdpOptimizerDebugProto::PREV_OPTIMIZATION;
        solver_init_traj_name = "prev_init";
      }
    }
  }

  // By default use smoothed initializer trajectory.
  if (!solver_init_traj.has_value()) {
    solver_init_traj = SmoothTrajectoryByMixedFourthOrderDdp(
        trajectory_steps, trajectory_time_step, initializer_traj, drive_passage,
        base_name_with_plan_id, trajectory_optimizer_params.smoother_params(),
        motion_constraint_params, veh_geo_params, veh_drive_params);
    solver_init_traj_source = DdpOptimizerDebugProto::SMOOTHED_INITIALIZER;
    solver_init_traj_name = "smooth_init";
  }
  QCHECK(solver_init_traj.has_value());

  // Add solver helper costs to problem.
  std::vector<std::unique_ptr<Cost<Mfob>>> solver_helper_costs;
  optimizer::AddSolidWhiteLineCost(
      trajectory_steps, base_name_with_plan_id, *solver_init_traj,
      constraint_manager, trajectory_optimizer_params.cost_weight_params(),
      veh_geo_params,
      vehicle_models_params.trajectory_optimizer_vehicle_model_params(),
      stations_query_helper, &solver_helper_costs);
  for (auto& cost : solver_helper_costs) {
    problem->AddCost(std::move(cost));
  }

  DdpOptimizer<Mfob> solver(problem.get(), trajectory_steps,
                            /*owner=*/"trajectory_optimizer",
                            /*verbosity=*/FLAGS_traj_opt_verbosity_level,
                            trajectory_optimizer_params.optimizer_params());
  IterationVisualizerHook<Mfob> iteration_visualizer_hook(
      trajectory_steps, base_name_with_plan_id,
      /*canvas_render_indices=*/true);
  if (FLAGS_traj_opt_canvas_level >= 2) {
    solver.AddHook(&iteration_visualizer_hook);
  }
  OptimizerSolverDebugHook<Mfob> solver_debug_hook(
      trajectory_steps, initializer_traj_start_point, st_traj_mgr);
  solver.AddHook(&solver_debug_hook);

  const auto& vehicle_model_params =
      vehicle_models_params.trajectory_optimizer_vehicle_model_params();
  std::vector<IterationAvModelVisualizerHook<Mfob>::Circle> av_model_circle;
  av_model_circle.reserve(vehicle_model_params.circles_size());
  for (const auto& circle : vehicle_model_params.circles()) {
    av_model_circle.push_back(
        {circle.dist_to_rac(), circle.angle_to_axis(), circle.radius()});
  }
  IterationAvModelVisualizerHook<Mfob> iteration_av_model_visualizer_hook(
      trajectory_steps, base_name_with_plan_id, av_model_circle,
      &veh_geo_params);
  if (FLAGS_traj_opt_draw_circle && FLAGS_traj_opt_canvas_level >= 2) {
    solver.AddHook(&iteration_av_model_visualizer_hook);
  }

  std::vector<TrajectoryPoint> result_points;

  std::string error_code = "";
  bool optimizer_solve_success = true;

  const absl::Time solve_start_time = absl::Now();
  auto solver_output = solver.Solve(*solver_init_traj,
                                    DdpOptimizer<Mfob>::SolveConfig::Onboard());
  const double solve_time =
      absl::ToDoubleMilliseconds(absl::Now() - solve_start_time);

  if (!solver_output.ok()) {
    error_code = solver_output.status().message();
    optimizer_solve_success = false;
  } else {
    result_points = std::move(*solver_output);
  }
  timer.Mark("solve");

  TrajectoryOptimizerOutput output;
  // ----------------------------------------------------------
  // --------------- Optimizer Auto Tuning --------------------
  // ----------------------------------------------------------
  // BANDAID(jingqiao): Refactor to solve code divergence in the future.
  const int trajectory_steps_dat =
      std::min(kDdpTrajectoryStepsDATHint, trajectory_steps);
  if (FLAGS_dump_expert_policy || FLAGS_optimizer_data_cleaning) {
    // TODO(jingqiao): Use input instead of directly reading file.
    const auto& expert_file =
        absl::StrCat(FLAGS_specific_snapshot_folder, "pose_trajectory.pb.txt");
    QCHECK(file_util::TextFileToProto(
        expert_file, output.expert_auto_tuning_traj_proto.mutable_trajectory()))
        << "Read expert policy failed!!!! File address: " << expert_file;

    *(output.expert_auto_tuning_traj_proto.mutable_costs()) =
        PoseTrajectoryToPolicy(
            trajectory_steps_dat, solver,
            output.expert_auto_tuning_traj_proto.trajectory(),
            FLAGS_auto_tuning_gamma);
  }

  SetIfInAutoTuningMode(solver, solver_debug_hook, result_points,
                        optimizer_solve_success, trajectory_steps_dat, &output);

  ToDebugProto(initializer_traj, *solver_init_traj, result_points,
               solver_debug_hook, solver_init_traj_source, optimizer_debug,
               thread_pool);

  std::vector<TrajectoryPlotInfo> trajs = {
      {.traj = initializer_traj, .name = "init", .color = vis::Color::kRed},
      {.traj = *solver_init_traj,
       .name = solver_init_traj_name,
       .color = vis::Color::kYellow},
      {.traj = result_points, .name = "res", .color = vis::Color::kBlue}};
  if (FLAGS_send_traj_optimizer_result_to_canvas) {
    CanvasDrawTrajectories(base_name_with_plan_id, trajs);
  }
  if (charts_data != nullptr) {
    optimizer::AddTrajCharts("traj_opt", trajs, charts_data->mutable_charts());
  }

  if (FLAGS_enable_ipopt_solver) {
    const auto ipopt_output = optimizer::CompareWithIpopt(
        base_name, base_name_with_plan_id, initializer_traj, *solver_init_traj,
        result_points,
        /*enable_comparison_debug_info_output=*/optimizer_solve_success,
        optimizer_debug->ddp(), trajs.back(), problem.get(), charts_data);
    if (!ipopt_output.ok()) {
      LOG(INFO) << "Ipopt solve failed, " << ipopt_output.message();
    } else {
      LOG(INFO) << "Ipopt solve succeed. ";
    }
  }

  // And this at the end in case some one did optimizer_debug->reset()
  // somewhere.
  optimizer_debug->mutable_ddp()->mutable_run_time_profile()->set_add_cost_time(
      add_cost_time);
  optimizer_debug->mutable_ddp()->mutable_run_time_profile()->set_solve_time(
      solve_time);

  DestroyContainerAsyncMarkSource(std::move(problem), (QCRAFT_LOC).ToString());

  if (!optimizer_solve_success) {
    return absl::InternalError(error_code);
  }

  const auto validation_status = optimizer::ValidateTrajectory(
      result_points,
      trajectory_optimizer_params.trajectory_optimizer_validation_params(),
      *optimizer_debug);
  if (!validation_status.ok()) {
    return absl::InternalError(validation_status.message());
  }

  // Fill trajectory optimizer state.
  output.trajectory_optimizer_state.last_optimized_trajectory = result_points;
  output.trajectory_optimizer_state.last_plan_start_time =
      input.plan_start_time;

  // Export nudge object id
  const auto nudge_object_info = ExtractNudgeObjectId(
      trajectory_steps, trajectory_time_step, input.lc_stage, leading_trajs,
      drive_passage, path_sl_boundary, result_points, st_planner_object_traj,
      veh_geo_params);
  if (nudge_object_info.ok()) {
    output.nudge_object_info = *nudge_object_info;
  }

  // Add stationary nudge QEvent.
  const auto stationary_nudge_status =
      optimizer::ExtractStationaryNudgeObjectId(
          result_points, leading_trajs, st_planner_object_traj, veh_geo_params,
          veh_drive_params);
  if (stationary_nudge_status.ok()) {
    QEVENT_EVERY_N_SECONDS("fengzhuang", "stationary_nudge",
                           /*seconds=*/100.0, [&](QEvent* qevent) {
                             qevent->AddField("object_id",
                                              *stationary_nudge_status);
                           });
  }

  std::vector<ApolloTrajectoryPointProto> output_traj =
      ToApolloTrajectoryPointProto(result_points);
  output.trajectory_proto = ToApolloTrajectoryPointProto(result_points);
  output.trajectory = std::move(result_points);
  return output;
}

}  // namespace planner
}  // namespace qcraft
