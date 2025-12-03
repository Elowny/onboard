#include "onboard/planner/plan/acc/acc_speed_finder.h"

#include <algorithm>  // IWYU pragma: keep
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/offset_rect.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/common/path_approx.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/acc_speed_decision.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/path_speed_combiner.h"
#include "onboard/planner/speed/plot_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_util.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_optimizer.h"
#include "onboard/planner/speed/speed_optimizer_config_dispatcher.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {

namespace {
using ChartSeriesDataProto = vis::vantage::ChartSeriesDataProto;

void PlotDataToChart(
    const AccSpeedFinderInput& input,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedVector& sampling_dp_preliminary_speed,
    const SpeedVector& predicted_av_speed, const SpeedVector& optimized_speed,
    bool enable_comfort_brake_speed, AccSpeedFinderOutput* output) {
  // vt graphs.
  speed::ExportSpeedVtDataToChart(sampling_dp_preliminary_speed,
                                  "dp_preliminary_speed", vis::Color::kOrange,
                                  /*add_tips=*/true, &output->vt_graph_chart);
  speed::ExportSpeedVtDataToChart(predicted_av_speed, "predicted_av_speed",
                                  vis::Color::kDarkYellow, /*add_tips=*/true,
                                  &output->vt_graph_chart);
  speed::ExportSpeedLimitToChart(input.base_name, output->speed_finder_proto,
                                 &output->vt_graph_chart);
  speed::ExportSpeedVtDataToChart(optimized_speed, "optimized_speed",
                                  vis::Color::kDarkGreen, /*add_tips=*/true,
                                  &output->vt_graph_chart);
  const auto& ref_speed_proto =
      output->speed_finder_proto.speed_optimizer().ref_speed();
  SpeedVector ref_speeds;
  ref_speeds.reserve(ref_speed_proto.size());
  for (const auto& speed_pt : ref_speed_proto) {
    ref_speeds.emplace_back().FromProto(speed_pt);
  }
  speed::ExportSpeedVtDataToChart(ref_speeds, "internal_ref_speed",
                                  vis::Color::kDarkMagenta, /*add_tips=*/true,
                                  &output->vt_graph_chart);

  // st graphs.
  const auto& comfortable_brake_speed_proto =
      output->speed_finder_proto.speed_optimizer().comfortable_brake_speed();
  // You can judge whether to enter safe mode by whether
  // comfortable_brake_speed is displayed.
  if (enable_comfort_brake_speed) {
    SpeedVector comfortable_brake_speed;
    comfortable_brake_speed.reserve(comfortable_brake_speed_proto.size());
    for (const auto& speed_pt : comfortable_brake_speed_proto) {
      comfortable_brake_speed.emplace_back().FromProto(speed_pt);
    }
    speed::ExportSpeedStDataToChart(
        comfortable_brake_speed, "comfortable_brake_speed",
        vis::Color::kLightGray, ChartSeriesDataProto::SOLIDLINE,
        &output->st_graph_chart);
  }
  speed::ExportStBoundaryToChart(input.base_name, kAccTrajectorySteps,
                                 st_boundaries_with_decision,
                                 output->speed_finder_proto,
                                 input.path->length(), &output->st_graph_chart);
  speed::ExportSpeedStDataToChart(sampling_dp_preliminary_speed,
                                  "dp_preliminary_speed", vis::Color::kOrange,
                                  ChartSeriesDataProto::SOLIDLINE,
                                  &output->st_graph_chart);
  speed::ExportSpeedStDataToChart(
      ref_speeds, "internal_ref_speed", vis::Color::kDarkMagenta,
      ChartSeriesDataProto::SOLIDLINE, &output->st_graph_chart);
  speed::ExportSpeedStDataToChart(
      optimized_speed, "optimized_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::SOLIDLINE, &output->st_graph_chart);
  speed::ExportPathToChart(input.base_name, *input.path, &output->path_chart);
  speed::ExportTrajToChart(input.base_name, output->trajectory_points,
                           &output->traj_chart);
  std::unordered_map<std::string, const SpacetimeObjectTrajectory*>
      pred_st_trajs_map;
  for (const auto& stb_wd : st_boundaries_with_decision) {
    if (const auto traj_id = stb_wd.traj_id(); traj_id.has_value()) {
      pred_st_trajs_map.emplace(
          *traj_id,
          QCHECK_NOTNULL(input.traj_mgr->FindTrajectoryById(*traj_id)));
    }
  }
  speed::ExportPredVtDataToChart(pred_st_trajs_map, input.base_name,
                                 &output->pred_vt_chart);
}
}  // namespace

namespace internal {
absl::Status OptimizeAccSpeed(
    std::string_view base_name, double init_v, double init_a, double delta_t,
    const SpeedOptimizerObjectManager& opt_obj_mgr, double path_length,
    const SpeedBoundMapType& speed_bound_map,
    const SpeedVector& reference_speed,
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  SCOPED_QTRACE("OptimizeAccSpeed");

  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);
  QCHECK_GT(path_length, 0.0);

  SpeedOptimizer speed_optimizer(base_name, init_v, init_a,
                                 &motion_constraint_params,
                                 &speed_finder_params, path_length,
                                 Mps2Mph(kDefaultAccSpeedLimitMps), delta_t);

  const absl::Status status = speed_optimizer.Optimize(
      opt_obj_mgr, speed_bound_map, reference_speed, time_aligned_prev_traj,
      optimized_speed, speed_finder_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Acc speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

SpeedBoundMapType EstimateDefaultSpeedBound(
    const SpeedLimit& speed_limit,
    const SpeedVector& dp_sampling_preliminary_speed, double init_v,
    double allowed_max_speed, int knot_num, double delta_t) {
  // speed_limit is V-S limits
  SpeedBoundMapType speed_upper_bound_map;
  // Estimate speed bound based on acc generated vs speed_limit (curvature
  // limit).
  std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
  speed_bounds_with_info.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const auto t = delta_t * i;
    const auto preliminary_speed_point_opt =
        dp_sampling_preliminary_speed.EvaluateByTime(t);
    const auto estimate_s = preliminary_speed_point_opt.has_value()
                                ? preliminary_speed_point_opt->s()
                                : (init_v * t);
    const auto preliminary_speed_limit = preliminary_speed_point_opt.has_value()
                                             ? preliminary_speed_point_opt->v()
                                             : (allowed_max_speed);
    const auto speed_limit_opt = speed_limit.GetSpeedLimitByS(estimate_s);
    const auto curvature_speed_limit =
        speed_limit_opt.value_or(allowed_max_speed);
    const auto final_speed_lim =
        std::min<double>(preliminary_speed_limit, curvature_speed_limit);
    const auto info = absl::StrFormat(
        "min of dp_preliminary(%.6f), curvature(%.6f) and max(%.6f)",
        preliminary_speed_limit, curvature_speed_limit, allowed_max_speed);
    speed_bounds_with_info.push_back(SpeedBoundWithInfo{
        .bound = final_speed_lim,
        .info = info,
    });
  }
  speed_upper_bound_map.emplace(SpeedLimitTypeProto::DEFAULT,
                                speed_bounds_with_info);
  speed_upper_bound_map.emplace(SpeedLimitTypeProto::COMBINATION,
                                std::move(speed_bounds_with_info));
  return speed_upper_bound_map;
}

}  // namespace internal

absl::StatusOr<AccSpeedFinderOutput> FindAccSpeed(
    const AccSpeedFinderInput& input) {
  SCOPED_QTRACE("FindAccSpeed");
  QCHECK_NOTNULL(input.speed_limit);
  QCHECK_NOTNULL(input.vehicle_geom);
  QCHECK_NOTNULL(input.speed_finder_params);
  QCHECK_NOTNULL(input.motion_constraint_params);

  const auto& vehicle_geometry_params = *input.vehicle_geom;
  const auto& motion_constraint_params = *input.motion_constraint_params;
  const auto& speed_finder_params = *input.speed_finder_params;

  ScopedMultiTimer timer("acc_speed_finder");
  const absl::Cleanup timeout_trigger = [start_time = absl::Now(), &input]() {
    constexpr double kSpeedFinderTimeLimitMs = 30.0;
    const auto speed_total_time_ms =
        absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (speed_total_time_ms > kSpeedFinderTimeLimitMs) {
      QEVENT_EVERY_N_SECONDS("pingshi", "speed_finder_timeout", 10.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("time_ms", speed_total_time_ms)
                                   .AddField("base_name", input.base_name);
                             });
    }
  };

  AccSpeedFinderOutput output;

  QCHECK(!input.path->empty());
  if (input.path->size() < 2) {
    return absl::FailedPreconditionError(
        absl::StrCat("Input path has only 1 path point: ",
                     input.path->front().DebugString()));
  }

  if (FLAGS_planner_send_path_data_to_debug) {
    for (const auto& p : *input.path) {
      *output.speed_finder_proto.add_path() = p;
    }
  }

  // Generate path_approx.
  constexpr double kPathApproxTolerance = 0.05;  // m.
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(vehicle_geometry_params);
  const auto path_kd_tree = BuildPathKdTree(*input.path);
  const auto path_approx = BuildPathApprox(
      *input.path, vehicle_rect, kPathApproxTolerance, path_kd_tree.get());

  // Step 1: Speed decision
  const AccSpeedDecisionInput acc_speed_decision_input{
      .base_name = input.base_name,
      .traj_mgr = input.traj_mgr,
      .path = input.path,
      .speed_limit = input.speed_limit,
      .path_kd_tree = path_kd_tree.get(),
      .path_approx = &path_approx,
      .trajectory_steps = kAccTrajectorySteps,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .user_speed_limit = input.user_speed_limit,
      .crowded_side_objs = input.crowded_side_objs,
      .vehicle_geometry_params = &vehicle_geometry_params,
      .motion_constraint_params = &motion_constraint_params,
      .speed_finder_params = &speed_finder_params,
  };
  ASSIGN_OR_RETURN(auto speed_decision_output,
                   MakeAccSpeedDecision(acc_speed_decision_input));
  const auto& st_boundaries_with_decision =
      speed_decision_output.st_boundaries_with_decision;
  const auto& interactive_speed_debug =
      speed_decision_output.interactive_speed_debug;
  const auto& sampling_dp_preliminary_speed =
      speed_decision_output.sampling_dp_preliminary_speed;
  if (FLAGS_planner_send_interactive_speed_to_chart) {
    speed::ExportSamplingDpSpeedToChart(
        input.base_name, st_boundaries_with_decision,
        sampling_dp_preliminary_speed, interactive_speed_debug,
        output.speed_finder_proto, &output.sampling_dp_chart);
    speed::ExportInteractiveSpeedToChart(
        input.base_name, st_boundaries_with_decision,
        sampling_dp_preliminary_speed, interactive_speed_debug,
        output.speed_finder_proto, &output.interactive_speed_chart);
  }

  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.speed_finder_proto);

  /*
   *  Step 2: Speed optimization.
   *
   */
  auto start_time = absl::Now();
  const double plan_total_time = kTrajectoryTimeStep * kAccTrajectorySteps;
  const int knot_num = speed_finder_params.speed_optimizer_params().knot_num();
  QCHECK_GT(knot_num - 1, 0);
  const double plan_time_interval = plan_total_time / (knot_num - 1);
  // Construct speed bound map and reference speed.
  const auto speed_bound_map = EstimateSpeedBound(
      speed_decision_output.speed_limit_provider, sampling_dp_preliminary_speed,
      input.plan_start_v, input.user_speed_limit, knot_num, plan_time_interval,
      input.base_name);

  // Assume the step length of the path is a fixed value.
  const double path_step_length = input.path->at(1).s() - input.path->at(0).s();
  QCHECK_GT(path_step_length, 0.0);
  const SpeedFinderParamsProto* new_speed_finder_params_ptr =
      &speed_finder_params;
  std::unique_ptr<SpeedFinderParamsProto> dispatched_speed_finder_params_ptr;
  auto dispatched_speed_optimizer_params = DispatchSpeedOptimizerConfig(
      st_boundaries_with_decision, *input.traj_mgr, /*drive_passage=*/nullptr,
      path_approx, *path_kd_tree, vehicle_rect.radius(), path_step_length,
      static_cast<int>(input.path->size() - 1), input.plan_start_v,
      input.path->front(), vehicle_geometry_params,
      speed_finder_params.speed_optimizer_params(),
      speed_finder_params.speed_optimizer_config_dispatcher_params(),
      &output.speed_finder_proto);
  if (dispatched_speed_optimizer_params.has_value()) {
    dispatched_speed_finder_params_ptr =
        std::make_unique<SpeedFinderParamsProto>(speed_finder_params);
    *dispatched_speed_finder_params_ptr->mutable_speed_optimizer_params() =
        std::move(*dispatched_speed_optimizer_params);
    new_speed_finder_params_ptr = dispatched_speed_finder_params_ptr.get();
  }
  const auto& new_speed_finder_params = *new_speed_finder_params_ptr;
  // Acc only have curvature speed limit.
  const auto reference_speed = GenerateReferenceSpeed(
      GenerateMinSpeedLimit(
          FindOrDie(speed_bound_map, SpeedLimitTypeProto::DEFAULT),
          FindOrDie(speed_bound_map, SpeedLimitTypeProto::CURVATURE),
          "min speed lim between default and curvature"),
      input.plan_start_v,
      new_speed_finder_params.speed_optimizer_params().ref_speed_bias(),
      new_speed_finder_params.speed_optimizer_params()
          .ref_speed_static_limit_bias(),
      motion_constraint_params.max_acceleration(),
      motion_constraint_params.max_deceleration(), plan_total_time,
      plan_time_interval);
  const auto predicted_av_speed = GeneratePredictedAvSpeed(
      sampling_dp_preliminary_speed, input.plan_start_v, input.plan_start_a,
      plan_total_time);
  const SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &predicted_av_speed, *input.traj_mgr,
      input.plan_start_v, plan_total_time, plan_time_interval,
      new_speed_finder_params);

  const auto start_a = input.plan_start_a;
  const auto start_v = input.plan_start_v;
  SpeedVector optimized_speed;
  RETURN_IF_ERROR(internal::OptimizeAccSpeed(
      input.base_name, start_v, start_a, plan_time_interval, opt_obj_mgr,
      input.path->length(), speed_bound_map, reference_speed,
      input.time_aligned_prev_traj, motion_constraint_params,
      new_speed_finder_params, &optimized_speed, &output.speed_finder_proto));
  constexpr double kSpeedOptimizerTimeLimitMs = 15.0;
  const auto speed_optimizer_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(2) << "Speed optimizer cost time(ms): " << speed_optimizer_time;
  if (speed_optimizer_time > kSpeedOptimizerTimeLimitMs) {
    QEVENT("pingshi", "speed_optimizer_timeout", [&](QEvent* qevent) {
      qevent->AddField("speed_optimizer_running_time[ms]", speed_optimizer_time)
          .AddField("time_limit[ms]", kSpeedOptimizerTimeLimitMs)
          .AddField("base_name", input.base_name);
    });
  }
  QCHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";
  timer.Mark("speed_optimization");
  /*
   *  Post-process the results of speed optimization.
   */
  if (new_speed_finder_params.enable_full_stop()) {
    PostProcessSpeedByFullStop(new_speed_finder_params, &optimized_speed);
  }

  /*
   *  Step 3: Integrate path and speed.
   *
   */
  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(optimized_speed.size());
  RETURN_IF_ERROR(CombinePathAndSpeed(*input.path, /*forward=*/true,
                                      optimized_speed,
                                      &output_trajectory_points));
  for (const auto& p : output_trajectory_points) {
    *output.speed_finder_proto.add_trajectory() = p;
  }
  output.trajectory_points = std::move(output_trajectory_points);

  if (FLAGS_planner_print_speed_finder_time_stats) {
    PrintMultiTimerReportStat(timer);
  }

  /*
   *  Step 4: Plot speed data to chart.
   *
   */
  PlotDataToChart(input, st_boundaries_with_decision,
                  sampling_dp_preliminary_speed, predicted_av_speed,
                  optimized_speed,
                  new_speed_finder_params.speed_optimizer_params()
                      .enable_comfort_brake_speed(),
                  &output);
  return output;
}

}  // namespace planner
}  // namespace qcraft
