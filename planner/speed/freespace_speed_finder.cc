#include "onboard/planner/speed/freespace_speed_finder.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
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
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"
#include "onboard/planner/speed/path_speed_combiner.h"
#include "onboard/planner/speed/plot_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_bound.h"
#include "onboard/planner/speed/speed_decision_util.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_finder_util.h"
#include "onboard/planner/speed/speed_optimizer.h"
#include "onboard/planner/speed/speed_optimizer_object_manager.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/planner/speed/standstill_distance_decider.h"
#include "onboard/proto/charts.pb.h"
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

absl::Status OptimizeFreespaceSpeed(
    double init_v, double init_a, double /*total_time*/, double delta_t,
    const SpeedOptimizerObjectManager& opt_obj_mgr, double path_length,
    double allowed_max_speed,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    const SpeedBoundMapType& speed_bound_map,
    const SpeedVector& reference_speed, SpeedVector* optimized_speed,
    SpeedFinderDebugProto* speed_finder_debug_proto) {
  SCOPED_QTRACE("OptimizeFreespaceSpeed");

  QCHECK_NOTNULL(optimized_speed);
  QCHECK_NOTNULL(speed_finder_debug_proto);
  QCHECK_GT(path_length, 0.0);

  SpeedOptimizer speed_optimizer(
      "freespace", init_v, init_a, &motion_constraint_params,
      &speed_finder_params, path_length, allowed_max_speed, delta_t);

  const absl::Status status =
      speed_optimizer.Optimize(opt_obj_mgr, speed_bound_map, reference_speed,
                               /*time_aligned_prev_traj=*/nullptr,
                               optimized_speed, speed_finder_debug_proto);
  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Freespace speed optimizer failed: ", status.message()));
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<FreespaceSpeedFinderOutput> FindFreespaceSpeed(
    const FreespaceSpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& /*vehicle_drive_params*/,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("FreespacePlanner/FindSpeed");

  ScopedMultiTimer timer("speed_finder");
  const absl::Cleanup timeout_trigger = [start_time = absl::Now()]() {
    constexpr double kSpeedFinderTimeLimitMs = 30.0;
    const auto speed_total_time_ms =
        absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (speed_total_time_ms > kSpeedFinderTimeLimitMs) {
      QEVENT_EVERY_N_SECONDS("pingshi", "freespace_speed_finder_timeout", 10.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("time_ms", speed_total_time_ms);
                             });
    }
  };

  FreespaceSpeedFinderOutput output;

  const auto& input_path = *input.path;
  QCHECK(!input_path.empty());
  if (input_path.size() < 2) {
    return absl::FailedPreconditionError(
        absl::StrCat("Input path has only 1 path point: ",
                     input_path.front().DebugString()));
  }

  const double plan_start_v =
      input.forward ? input.plan_start_v : -input.plan_start_v;
  const double plan_start_a =
      input.forward ? input.plan_start_a : -input.plan_start_a;

  if (FLAGS_planner_send_path_data_to_debug) {
    for (const auto& p : input_path) {
      *output.speed_finder_proto.add_path() = p;
    }
  }

  /*
   *  Step 1: Map st boundaries of objects onto st graph.
   *
   */
  auto start_time = absl::Now();
  const auto av_shapes = BuildFreespaceAvShapes(
      vehicle_geometry_params, input_path, input.forward, vehicle_model_params);
  const auto path_kd_tree = BuildPathKdTree(input_path);
  auto st_graph = std::make_unique<StGraph>(
      &input_path, kTrajectorySteps, plan_start_v,
      motion_constraint_params.max_deceleration(), &vehicle_geometry_params,
      &speed_finder_params.st_graph_params(), &av_shapes, path_kd_tree.get(),
      /*path_approx=*/nullptr, /*path_approx_for_mirrors=*/nullptr);

  auto st_boundaries = st_graph->GetStBoundaries(
      *input.obj_mgr, /*leading_objs=*/{}, *input.constraint_mgr,
      input.planner_semantic_map_manager, /*drive_passage=*/nullptr,
      /*path_sl_boundary=*/nullptr, thread_pool);
  VLOG(2) << "Build st_graph cost time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(3) << "st_boundary size = " << st_boundaries.size();
  timer.Mark("map_st_boundaries");
  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));
  // Set follow/lead standstill distance for st-boundaries.
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = input.stalled_objects,
      .planner_semantic_map_manager = nullptr,
      .lane_path = nullptr,
      .st_traj_mgr = input.obj_mgr,
      .constraint_mgr = input.constraint_mgr,
      .extra_follow_standstill_for_large_vehicle =
          PiecewiseLinearFunctionFromProto(
              speed_finder_params
                  .extra_follow_standstill_distance_for_large_vehicle_plf())(
              plan_start_v)};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision);
  }
  // Only keep all zero-distance stationary st-boudnaries & the nearest
  // non-zero-distance stationary st-boundary.
  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);

  /*
   *  Step 2: Make some confident decisions in speed decider.
   *
   */
  // TODO(renjie): Implement freespace speed constraint decisions.
  output.constraint_mgr = *input.constraint_mgr;

  /*
   *  Step 3: Make pre-decision for st_boundary.
   *
   */
  MakeFreespacePreDecisionForStBoundaries(&st_boundaries_with_decision);
  timer.Mark("pre_decider");

  // Make follow decisions for all st-boundaries without prior decision.
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    if (st_boundary_with_decision.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundary_with_decision.set_decision_type(StBoundaryProto::FOLLOW);
      st_boundary_with_decision.set_decision_reason(StBoundaryProto::FREESPACE);
    }
  }

  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.speed_finder_proto);

  // Compute the minimum stop_s.
  const auto min_s_info = ComputeMinStopSInfo(st_boundaries_with_decision);

  /*
   *  Step 4: Speed optimization.
   *
   */
  start_time = absl::Now();

  const double plan_total_time = kTrajectoryTimeStep * kTrajectorySteps;
  const int knot_num = speed_finder_params.speed_optimizer_params().knot_num();
  QCHECK_GT(knot_num - 1, 0);
  const double plan_time_interval = plan_total_time / (knot_num - 1);
  const double allowed_max_speed_mph =
      input.forward ? motion_constraint_params.default_speed_limit()
                    : motion_constraint_params.default_reverse_speed_limit();
  const double allowed_max_speed = Mph2Mps(allowed_max_speed_mph);

  // Generate speed upper bound map with max speed.
  SpeedBoundMapType speed_bound_map;
  std::vector<SpeedBoundWithInfo> speed_bound(
      knot_num,
      {.bound = allowed_max_speed,
       .info = SpeedLimitTypeProto::Type_Name(SpeedLimitTypeProto::DEFAULT)});
  InsertIfNotPresent(&speed_bound_map, SpeedLimitTypeProto::DEFAULT,
                     speed_bound);
  InsertIfNotPresent(&speed_bound_map, SpeedLimitTypeProto::COMBINATION,
                     speed_bound);
  const auto reference_speed = GenerateReferenceSpeed(
      speed_bound, plan_start_v,
      speed_finder_params.speed_optimizer_params().ref_speed_bias(),
      speed_finder_params.speed_optimizer_params()
          .ref_speed_static_limit_bias(),
      motion_constraint_params.max_acceleration(),
      motion_constraint_params.max_deceleration(), plan_total_time,
      plan_time_interval);

  const SpeedOptimizerObjectManager opt_obj_mgr(
      st_boundaries_with_decision, /*preliminary_speed=*/nullptr,
      *input.obj_mgr, plan_start_v, plan_total_time, plan_time_interval,
      speed_finder_params);

  SpeedVector optimized_speed;
  RETURN_IF_ERROR(OptimizeFreespaceSpeed(
      plan_start_v, plan_start_a, plan_total_time, plan_time_interval,
      opt_obj_mgr, input_path.length(), allowed_max_speed_mph,
      motion_constraint_params, speed_finder_params, speed_bound_map,
      reference_speed, &optimized_speed, &output.speed_finder_proto));

  constexpr double kSpeedOptimizerTimeLimitMs = 15.0;
  const auto speed_optimizer_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(2) << "Speed optimizer cost time(ms): " << speed_optimizer_time;
  if (speed_optimizer_time > kSpeedOptimizerTimeLimitMs) {
    QEVENT("pingshi", "speed_optimizer_timeout", [&](QEvent* qevent) {
      qevent->AddField("speed_optimizer_running_time[ms]", speed_optimizer_time)
          .AddField("time_limit[ms]", kSpeedOptimizerTimeLimitMs);
    });
  }
  QCHECK(!optimized_speed.empty()) << "Optimized speed points is empty!";
  timer.Mark("speed_optimization");

  /*
   *  Post-process the results of speed optimization.
   */
  if (speed_finder_params.enable_full_stop()) {
    PostProcessSpeedByFullStop(speed_finder_params, &optimized_speed);
  }

  /*
   *  Step 5: Integrate path and speed.
   *
   */
  std::vector<ApolloTrajectoryPointProto> output_trajectory_points;
  output_trajectory_points.reserve(kTrajectorySteps);
  RETURN_IF_ERROR(CombinePathAndSpeed(
      input_path, input.forward, optimized_speed, &output_trajectory_points));
  for (const auto& p : output_trajectory_points) {
    *output.speed_finder_proto.add_trajectory() = p;
  }

  if (FLAGS_planner_print_speed_finder_time_stats) {
    PrintMultiTimerReportStat(timer);
  }

  /*
   *  Step 6: Plot speed data to chart.
   *
   */
  // st_graph chart
  speed::ExportStBoundaryToChart(
      /*base_name=*/"freespace", kTrajectorySteps, st_boundaries_with_decision,
      output.speed_finder_proto, input_path.length(), &output.st_graph_chart);
  speed::ExportSpeedStDataToChart(
      optimized_speed, "optimized_speed", vis::Color::kDarkGreen,
      ChartSeriesDataProto::SOLIDLINE, &output.st_graph_chart);

  // vt_graph chart
  speed::ExportSpeedLimitToChart(/*base_name=*/"freespace",
                                 output.speed_finder_proto,
                                 &output.vt_graph_chart);
  speed::ExportSpeedVtDataToChart(optimized_speed, "optimized_speed",
                                  vis::Color::kDarkGreen, /*add_tips=*/true,
                                  &output.vt_graph_chart);
  const auto& ref_speed_proto =
      output.speed_finder_proto.speed_optimizer().ref_speed();
  SpeedVector ref_speeds;
  ref_speeds.reserve(ref_speed_proto.size());
  for (const auto& speed_pt : ref_speed_proto) {
    ref_speeds.emplace_back().FromProto(speed_pt);
  }
  speed::ExportSpeedVtDataToChart(ref_speeds, "ref_speed",
                                  vis::Color::kDarkMagenta, /*add_tips=*/false,
                                  &output.vt_graph_chart);

  // traj chart
  speed::ExportTrajToChart(/*base_name=*/"freespace", output_trajectory_points,
                           &output.traj_chart);

  if (FLAGS_planner_send_speed_path_chart_data) {
    speed::ExportPathToChart(/*base_name=*/"freespace", input_path,
                             &output.path_chart);
  }

  if (FLAGS_planner_draw_st_boundary_canvas) {
    speed::DrawStBoundaryOnCanvas(
        /*base_name=*/"freespace", st_boundaries_with_decision,
        vehicle_geometry_params, *input.obj_mgr, input_path);
  }

  if (!FLAGS_planner_send_speed_optimizer_debug) {
    output.speed_finder_proto.clear_speed_optimizer();
  }

  output.trajectory_points = std::move(output_trajectory_points);
  output.considered_st_objects =
      GetConsideredStObjects(st_boundaries_with_decision, *input.obj_mgr,
                             /*processed_st_objects=*/{});

  output.stop_s = min_s_info.min_stop_s.has_value()
                      ? std::min(*min_s_info.min_stop_s, input.path->length())
                      : input.path->length();
  output.stationary_object_stop_s =
      min_s_info.min_stationary_object_stop_s.has_value()
          ? *min_s_info.min_stationary_object_stop_s
          : std::numeric_limits<double>::infinity();
  output.nearest_stationary_object_id =
      min_s_info.nearest_stationary_object_id.has_value()
          ? *min_s_info.nearest_stationary_object_id
          : "";
  output.speed_finder_proto.set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(input.plan_time));

  DestroyContainerAsyncMarkSource(std::move(st_graph), (QCRAFT_LOC).ToString());

  return output;
}

}  // namespace planner
}  // namespace qcraft
