#include "onboard/planner/speed/speed_finder.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/async/async_util.h"
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
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/path_speed_combiner.h"
#include "onboard/planner/speed/plot_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_bound.h"
#include "onboard/planner/speed/speed_decision.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_util.h"
#include "onboard/planner/speed/speed_optimizer.h"
#include "onboard/planner/speed/speed_optimizer_config_dispatcher.h"
#include "onboard/planner/speed/speed_optimizer_object_manager.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft {
namespace planner {

namespace {
using ChartSeriesDataProto = vis::vantage::ChartSeriesDataProto;

absl::Status OptimizeSpeed(
    std::string_view base_name, double init_v, double init_a, double delta_t,
    const SpeedOptimizerObjectManager& opt_obj_mgr,
    const SpeedBoundMapType& speed_bound_map, double path_length,
    const SpeedVector& reference_speed,
    const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  SCOPED_QTRACE("OptimizeSpeed");

  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);
  QCHECK_GT(path_length, 0.0);

  SpeedOptimizer speed_optimizer(
      base_name, init_v, init_a, &motion_constraint_params,
      &speed_finder_params, path_length,
      motion_constraint_params.default_speed_limit(), delta_t);
  const absl::Status status = speed_optimizer.Optimize(
      opt_obj_mgr, speed_bound_map, reference_speed, time_aligned_prev_traj,
      optimized_speed, speed_finder_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

std::optional<TrajectoryEndInfoProto> SetTrajectoryEndInfo(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    double speed_length) {
  std::optional<TrajectoryEndInfoProto> traj_end_info = std::nullopt;
  for (const StBoundaryWithDecision& st_boundary_with_decision :
       st_boundaries_with_decision) {
    const auto* st_boundary = st_boundary_with_decision.st_boundary();
    if (!st_boundary->is_stationary()) {
      continue;
    }

    const double min_s = st_boundary->min_s();
    const double object_upper_bound =
        min_s - st_boundary_with_decision.follow_standstill_distance();
    const double intrusion_value = speed_length - object_upper_bound;

    if (intrusion_value > 0.0) {
      if (!traj_end_info.has_value() || min_s < traj_end_info->end_s()) {
        TrajectoryEndInfoProto end_info_proto;
        end_info_proto.set_st_boundary_id(st_boundary->id());
        end_info_proto.set_end_s(min_s);
        end_info_proto.set_intrusion_value(intrusion_value);
        end_info_proto.set_type(st_boundary->source_type());
        traj_end_info = std::move(end_info_proto);
      }
    }
  }
  return traj_end_info;
}

ObjectsPredictionProto ConstructModifiedPrediction(
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>&
        processed_st_objects) {
  FUNC_QTRACE();
  // Use these elements to construct ObjectPrediction.
  struct PredElements {
    std::vector<prediction::PredictedTrajectory> trajs;
    const ObjectProto* object_proto = nullptr;
  };

  std::map<std::string, PredElements> processed_preds;
  for (const auto& [traj_id, st_obj] : processed_st_objects) {
    const auto object_id =
        SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(traj_id);
    if (processed_preds[object_id].object_proto == nullptr) {
      processed_preds[object_id].object_proto =
          &st_obj.planner_object().object_proto();
    }
    processed_preds[object_id].trajs.push_back(st_obj.trajectory());
  }

  ObjectsPredictionProto modified_prediction;
  for (auto& [_, pred_elements] : processed_preds) {
    // TODO(lidong): Set the correct prediction road status and intersection
    // status.
    const prediction::ObjectPrediction obj_pred(
        std::move(pred_elements.trajs), *pred_elements.object_proto,
        ObjectRoadStatus::ORS_NONE, ObjectIntersectionStatus::OIS_NONE);
    obj_pred.ToProto(modified_prediction.add_objects());
  }

  return modified_prediction;
}

void PlotSpeedDataToChart(
    const SpeedFinderInput& input,
    const SpeedDecisionOutput& speed_decision_output,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto& speed_finder_params,
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const std::vector<ApolloTrajectoryPointProto>& output_trajectory_points,
    const SpeedVector& preliminary_speed, const SpeedVector& optimized_speed,
    const SpeedVector& predicted_av_speed, int trajectory_steps,
    SpeedFinderOutput* output) {
  SCOPED_QTRACE("ExportCharts");
  // st_graph chart.
  speed::ExportStBoundaryToChart(input.base_name, trajectory_steps,
                                 st_boundaries_with_decision,
                                 output->speed_finder_proto,
                                 input.path->length(), &output->st_graph_chart);
  speed::ExportSpeedStDataToChart(
      preliminary_speed, "preliminary_speed", vis::Color::kOrange,
      ChartSeriesDataProto::DASHLINE, &output->st_graph_chart);
  speed::ExportSpeedStDataToChart(
      optimized_speed, "optimized_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::SOLIDLINE, &output->st_graph_chart);
  const auto& comfortable_brake_speed_proto =
      output->speed_finder_proto.speed_optimizer().comfortable_brake_speed();

  // You can judge whether to enter safe mode by whether
  // comfortable_brake_speed is displayed.
  if (speed_finder_params.speed_optimizer_params()
          .enable_comfort_brake_speed()) {
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

  const auto& max_brake_speed_proto =
      output->speed_finder_proto.speed_optimizer().max_brake_speed();
  SpeedVector max_brake_speed;
  max_brake_speed.reserve(max_brake_speed_proto.size());
  for (const auto& speed_pt : max_brake_speed_proto) {
    max_brake_speed.emplace_back().FromProto(speed_pt);
  }
  speed::ExportSpeedStDataToChart(
      max_brake_speed, "max_brake_speed", vis::Color::kLightGray,
      ChartSeriesDataProto::DASHLINE, &output->st_graph_chart);

  if (FLAGS_planner_send_interactive_speed_to_chart) {
    speed::ExportSamplingDpSpeedToChart(
        input.base_name, st_boundaries_with_decision, preliminary_speed,
        speed_decision_output.interactive_speed_debug,
        output->speed_finder_proto, &output->sampling_dp_chart);
    speed::ExportInteractiveSpeedToChart(
        input.base_name, st_boundaries_with_decision, preliminary_speed,
        speed_decision_output.interactive_speed_debug,
        output->speed_finder_proto, &output->interactive_speed_chart);
  }

  // vt_graph chart
  speed::ExportSpeedLimitToChart(input.base_name, output->speed_finder_proto,
                                 &output->vt_graph_chart);
  speed::ExportSpeedVtDataToChart(preliminary_speed, "preliminary_speed",
                                  vis::Color::kOrange, /*add_tips=*/true,
                                  &output->vt_graph_chart);
  speed::ExportSpeedVtDataToChart(optimized_speed, "optimized_speed",
                                  vis::Color::kDarkGreen, /*add_tips=*/true,
                                  &output->vt_graph_chart);
  speed::ExportSpeedVtDataToChart(predicted_av_speed, "predicted_av_speed",
                                  vis::Color::kDarkYellow, /*add_tips=*/true,
                                  &output->vt_graph_chart);
  const auto& ref_speed_proto =
      output->speed_finder_proto.speed_optimizer().ref_speed();
  SpeedVector ref_speeds;
  ref_speeds.reserve(ref_speed_proto.size());
  for (const auto& speed_pt : ref_speed_proto) {
    ref_speeds.emplace_back().FromProto(speed_pt);
  }
  speed::ExportSpeedVtDataToChart(ref_speeds, "ref_speed",
                                  vis::Color::kDarkMagenta,
                                  /*add_tips=*/false, &output->vt_graph_chart);

  // Prediction trajectories vt-chart.
  const auto& processed_st_objects = speed_decision_output.processed_st_objects;
  const auto st_trajs_map =
      GetAllOverlappedStObjectTrajs(speed_decision_output.overlap_trajs_info,
                                    processed_st_objects, *input.traj_mgr);
  speed::ExportPredVtDataToChart(st_trajs_map, input.base_name,
                                 &output->prediction_vt_chart);

  // Traj chart
  speed::ExportTrajToChart(input.base_name, output_trajectory_points,
                           &output->traj_chart);

  if (FLAGS_planner_send_speed_path_chart_data) {
    speed::ExportPathToChart(input.base_name, *input.path, &output->path_chart);
  }

  if (FLAGS_planner_draw_st_boundary_canvas) {
    speed::DrawStBoundaryOnCanvas(input.base_name, st_boundaries_with_decision,
                                  vehicle_geometry_params, *input.traj_mgr,
                                  *input.path);
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-function-size)
absl::StatusOr<SpeedFinderOutput> FindSpeed(
    const SpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE_ARG1("FindSpeed", "trajectory size",
                     input.traj_mgr->trajectories().size());
  QCHECK_NOTNULL(input.path);
  QCHECK_NOTNULL(input.st_path_points);
  if (input.path->size() < 2) {
    return absl::FailedPreconditionError(absl::StrFormat(
        "Path size is less than 2, which is %d.", input.path->size()));
  }
  if (input.st_path_points->empty()) {
    return absl::FailedPreconditionError("st_path_points is empty.");
  }
  ScopedMultiTimer timer("speed_finder");
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

  constexpr double kEpsilon = 1.0e-6;
  QCHECK_NEAR(input.path->front().x(), input.st_path_points->front().x(),
              kEpsilon);
  QCHECK_NEAR(input.path->front().y(), input.st_path_points->front().y(),
              kEpsilon);

  SpeedFinderOutput output;

  if (FLAGS_planner_send_path_data_to_debug) {
    for (const auto& p : *input.path) {
      *output.speed_finder_proto.add_path() = p;
    }
  }

  const int trajectory_steps = GetSpeedFinderTrajectorySteps(
      input.plan_start_v, speed_finder_params.max_traj_steps());

  // Generate path_approx.
  constexpr double kPathApproxTolerance = 0.05;  // m.
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(vehicle_geometry_params);
  const auto path_kd_tree = BuildPathKdTree(*input.path);
  const auto path_approx = BuildPathApprox(
      *input.path, vehicle_rect, kPathApproxTolerance, path_kd_tree.get());
  std::optional<PathApprox> path_approx_for_mirrors;
  auto start_time = absl::Now();
  const auto av_shapes = BuildAvShapes(vehicle_geometry_params, *input.path);
  PathApprox* path_approx_for_mirrors_ptr = nullptr;

  if (FLAGS_planner_use_path_approx_based_st_mapping) {
    path_approx_for_mirrors =
        BuildPathApproxForMirrors(path_approx, vehicle_geometry_params);
    path_approx_for_mirrors_ptr = path_approx_for_mirrors.has_value()
                                      ? &(*path_approx_for_mirrors)
                                      : nullptr;
  }

  /*
   *  Step 1: Generate st boundaries and make speed decision.
   *
   */
  auto st_graph = std::make_unique<StGraph>(
      input.path, trajectory_steps, input.plan_start_v,
      motion_constraint_params.max_deceleration(), &vehicle_geometry_params,
      &speed_finder_params.st_graph_params(), &av_shapes, path_kd_tree.get(),
      &path_approx, path_approx_for_mirrors_ptr);
  const double planner_speed_cap =
      Mph2Mps(motion_constraint_params.default_speed_limit());  // m/s.

  SpeedDecisionInput speed_decision_input{
      .base_name = input.base_name,
      .vehicle_geometry_params = &vehicle_geometry_params,
      .motion_constraint_params = &motion_constraint_params,
      .vehicle_drive_params = &vehicle_drive_params,
      .speed_finder_params = &speed_finder_params,
      .drive_passage = input.drive_passage,
      .path_sl_boundary = input.path_sl_boundary,
      .driving_map_topo = input.driving_map_topo,
      .psmm = input.psmm,
      .path = input.path,
      .st_path_points = input.st_path_points,
      .av_shapes = &av_shapes,
      .path_kd_tree = path_kd_tree.get(),
      .traj_mgr = input.traj_mgr,
      .constraint_mgr = input.constraint_mgr,
      .leading_trajs = input.leading_trajs,
      .follower_set = input.follower_set,
      .stalled_objects = input.stalled_objects,
      .planner_av_context = input.planner_av_context,
      .real_objects = input.real_objects,
      .virtual_objects = input.virtual_objects,
      .planner_model_pool = input.planner_model_pool,
      .run_act_net_speed_decision = input.run_act_net_speed_decision,
      .plan_time = input.plan_time,
      .plan_start_v = input.plan_start_v,
      .plan_start_a = input.plan_start_a,
      .planner_speed_cap = planner_speed_cap,
      .trajectory_steps = trajectory_steps};
  ASSIGN_OR_RETURN(auto speed_decision_output,
                   MapStBoundariesAndMakeSpeedDecision(
                       speed_decision_input, st_graph.get(), thread_pool));
  timer.Mark("Construct st graph and make non-interactive speed decision.");

  auto& st_boundaries_with_decision =
      speed_decision_output.st_boundaries_with_decision;
  const auto& speed_limit_provider = speed_decision_output.speed_limit_provider;
  auto& processed_st_objects = speed_decision_output.processed_st_objects;
  output.constraint_mgr = std::move(speed_decision_output.constraint_mgr);
  const auto& preliminary_speed = speed_decision_output.preliminary_speed;
  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.speed_finder_proto);

  /*
   *  Step 2: Speed optimization.
   *
   */
  start_time = absl::Now();

  const double plan_total_time = kTrajectoryTimeStep * trajectory_steps;
  const int knot_num = speed_finder_params.speed_optimizer_params().knot_num();
  QCHECK_GT(knot_num - 1, 0);
  const double plan_time_interval = plan_total_time / (knot_num - 1);

  // Assume the step length of the path is a fixed value.
  const double path_step_length = input.path->at(1).s() - input.path->at(0).s();
  QCHECK_GT(path_step_length, 0.0);

  const SpeedFinderParamsProto* new_speed_finder_params_ptr =
      &speed_finder_params;
  std::unique_ptr<SpeedFinderParamsProto> dispatched_speed_finder_params_ptr;
  auto dispatched_speed_optimizer_params = DispatchSpeedOptimizerConfig(
      st_boundaries_with_decision, *input.traj_mgr, input.drive_passage,
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

  const auto speed_bound_map = EstimateSpeedBound(
      speed_limit_provider, preliminary_speed, input.plan_start_v,
      planner_speed_cap, knot_num, plan_time_interval, input.base_name);
  const auto reference_speed = GenerateReferenceSpeed(
      GenerateMinSpeedLimit(
          FindOrDie(speed_bound_map, SpeedLimitTypeProto::LANE),
          FindOrDie(speed_bound_map, SpeedLimitTypeProto::CURVATURE),
          "min speed bound between lane and curvature"),
      input.plan_start_v,
      new_speed_finder_params.speed_optimizer_params().ref_speed_bias(),
      new_speed_finder_params.speed_optimizer_params()
          .ref_speed_static_limit_bias(),
      motion_constraint_params.max_acceleration(),
      motion_constraint_params.max_deceleration(), plan_total_time,
      plan_time_interval);
  SpeedVector optimized_speed;
  const auto predicted_av_speed =
      GeneratePredictedAvSpeed(preliminary_speed, input.plan_start_v,
                               input.plan_start_a, plan_total_time);
  const SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, &predicted_av_speed, *input.traj_mgr,
      input.plan_start_v, plan_total_time, plan_time_interval,
      speed_finder_params);
  RETURN_IF_ERROR(OptimizeSpeed(
      input.base_name, input.plan_start_v, input.plan_start_a,
      plan_time_interval, opt_obj_mgr, speed_bound_map, input.path->length(),
      reference_speed, input.time_aligned_prev_traj, motion_constraint_params,
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
  CutoffSpeedByTimeHorizon(&optimized_speed);

  // Set trajectory end info.
  output.trajectory_end_info = SetTrajectoryEndInfo(
      st_boundaries_with_decision, optimized_speed.TotalLength());

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

  if (FLAGS_planner_print_speed_finder_time_stats) {
    PrintMultiTimerReportStat(timer);
  }

  /*
   *  Step 4: Plot speed data to chart.
   *
   */
  PlotSpeedDataToChart(input, speed_decision_output, vehicle_geometry_params,
                       new_speed_finder_params, st_boundaries_with_decision,
                       output_trajectory_points, preliminary_speed,
                       optimized_speed, predicted_av_speed, trajectory_steps,
                       &output);

  if (!FLAGS_planner_send_speed_optimizer_debug) {
    output.speed_finder_proto.clear_speed_optimizer();
  }

  output.trajectory_points = std::move(output_trajectory_points);
  *output.speed_finder_proto.mutable_modified_prediction() =
      ConstructModifiedPrediction(processed_st_objects);

  output.considered_st_objects =
      GetConsideredStObjects(st_boundaries_with_decision, *input.traj_mgr,
                             std::move(processed_st_objects));

  output.speed_finder_proto.set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_time));
  DestroyContainerAsyncMarkSource(std::move(st_graph), (QCRAFT_LOC).ToString());

  return output;
}

}  // namespace planner
}  // namespace qcraft
