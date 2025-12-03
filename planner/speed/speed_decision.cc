#include "onboard/planner/speed/speed_decision.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/ml/act_net_speed/act_net_speed_decider.h"
#include "onboard/planner/speed/close_trajectory_decider.h"
#include "onboard/planner/speed/constraint_generator.h"
#include "onboard/planner/speed/decider/pre_brake_decider.h"
#include "onboard/planner/speed/decider/pre_st_boundary_modifier.h"
#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"
#include "onboard/planner/speed/ignore_decider.h"
#include "onboard/planner/speed/interactive_speed_decision.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_decision_util.h"
#include "onboard/planner/speed/speed_finder_flags.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_limit_generator.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_overlap_analyzer.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/standstill_distance_decider.h"
#include "onboard/planner/speed/time_buffer_decider.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
StBoundaryWithDecision GenerateStBoundaryWithDecisionByStopLine(
    const StGraph& st_graph,
    const ConstraintProto::PathStopLineProto& path_stop_line) {
  StBoundaryWithDecision stb_wd(st_graph.MapPathStopLine(path_stop_line),
                                StBoundaryProto::FOLLOW,
                                StBoundaryProto::CONSTRAINT_GENERATOR);
  stb_wd.set_follow_standstill_distance(path_stop_line.standoff());
  return stb_wd;
}

void AnalyzePathSemanticsAndStOverlaps(
    const SpeedDecisionInput& input, std::vector<StBoundaryRef>* st_boundaries,
    std::vector<PathPointSemantic>* path_semantics, ThreadPool* thread_pool) {
  QCHECK_NOTNULL(st_boundaries);
  QCHECK_NOTNULL(path_semantics);
  // Analyze path semantics.
  int max_analyze_path_index = -1;
  for (const auto& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    for (const auto& overlap_info : st_boundary->overlap_infos()) {
      max_analyze_path_index =
          std::max(max_analyze_path_index, overlap_info.av_end_idx);
    }
  }
  if (max_analyze_path_index >= 0) {
    const auto start_time = absl::Now();
    auto path_semantics_or =
        AnalyzePathSemantics(*input.path, max_analyze_path_index, *input.psmm,
                             input.driving_map_topo, thread_pool);
    VLOG(2) << "Analyze path semantics cost time(ms): "
            << absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (path_semantics_or.ok() && VLOG_IS_ON(4)) {
      for (int i = 0; i < path_semantics_or->size(); ++i) {
        VLOG(4) << "Path point[" << i << "]: (" << (*input.path)[i].x() << ", "
                << (*input.path)[i].y() << "), closest lane point "
                << (*path_semantics_or)[i].closest_lane_point.DebugString()
                << ", lane path id history size "
                << (*path_semantics_or)[i].lane_path_id_history.size();
        for (int j = 0; j < (*path_semantics_or)[i].lane_path_id_history.size();
             ++j) {
          VLOG(4) << "Path point[" << i << "] lane path id [" << j
                  << "]: " << (*path_semantics_or)[i].lane_path_id_history[j];
        }
      }
    }

    if (path_semantics_or.ok()) {
      AnalyzeStOverlaps(*input.path, *path_semantics_or, *input.psmm,
                        *input.traj_mgr, input.drive_passage,
                        input.built_target_frenet_frame,
                        *QCHECK_NOTNULL(input.vehicle_geometry_params),
                        QCHECK_NOTNULL(input.speed_finder_params)
                            ->st_overlap_analyzer_params(),
                        *input.av_shapes, input.plan_start_v, st_boundaries);
      *path_semantics = std::move(*path_semantics_or);
      if (VLOG_IS_ON(4)) {
        for (const auto& st_boundary : *st_boundaries) {
          if (st_boundary->overlap_meta().has_value()) {
            const auto& overlap_meta = *st_boundary->overlap_meta();
            VLOG(4) << "St-boundary " << st_boundary->id()
                    << " overlap pattern: "
                    << StOverlapMetaProto::OverlapPattern_Name(
                           overlap_meta.pattern())
                    << ", source: "
                    << StOverlapMetaProto::OverlapSource_Name(
                           overlap_meta.source())
                    << ", priority: "
                    << StOverlapMetaProto::OverlapPriority_Name(
                           overlap_meta.priority())
                    << ", priority reason: " << overlap_meta.priority_reason()
                    << ", modification type: "
                    << StOverlapMetaProto::ModificationType_Name(
                           overlap_meta.modification_type());
          }
        }
      }
    } else {
      QLOG(WARNING)
          << "Path semantic analyzer fails, skip analyzing overlap meta: "
          << path_semantics_or.status().message();
    }
  }
}

void DecideTimeBuffers(
    const SpacetimeTrajectoryManager& traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    double plan_start_v,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  {
    SCOPED_QTRACE("TimeBufferDecider");
    absl::flat_hash_set<std::string> disable_pass_time_buffer_id_set;
    for (const auto& st_boundary_wd : *st_boundaries_with_decision) {
      if (!st_boundary_wd.raw_st_boundary()->is_protective()) {
        continue;
      }
      const auto& protected_st_boundary_id =
          st_boundary_wd.raw_st_boundary()->protected_st_boundary_id();
      if (!protected_st_boundary_id.has_value()) continue;
      switch (st_boundary_wd.raw_st_boundary()->protection_type()) {
        case StBoundaryProto::SMALL_ANGLE_CUT_IN:
        case StBoundaryProto::LANE_CHANGE_GAP: {
          disable_pass_time_buffer_id_set.insert(*protected_st_boundary_id);
          break;
        }
        case StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT:
        case StBoundaryProto::NON_PROTECTIVE:
          break;
      }
    }

    for (auto& st_boundary_with_decision : *st_boundaries_with_decision) {
      const bool disable_pass_time_buffer = ContainsKey(
          disable_pass_time_buffer_id_set, st_boundary_with_decision.id());
      DecideTimeBuffersForStBoundary(&st_boundary_with_decision, plan_start_v,
                                     vehicle_geometry_params, traj_mgr,
                                     disable_pass_time_buffer);
    }
  }
  if (VLOG_IS_ON(4)) {
    for (const auto& st_boundary_with_decision : *st_boundaries_with_decision) {
      VLOG(4) << "Set st-boundary " << st_boundary_with_decision.id()
              << " pass_time: " << st_boundary_with_decision.pass_time()
              << " yield_time: " << st_boundary_with_decision.yield_time();
    }
  }
}

void AddStationaryCloseObjectConstraint(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& traj_mgr,
    const DiscretizedPath& path, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary, double plan_start_v,
    ConstraintManager* constraint_mgr) {
  auto stationary_close_object_constraint =
      GenerateStationaryCloseObjectConstraints(
          st_boundaries_with_decision, st_graph, traj_mgr, path, drive_passage,
          path_sl_boundary, plan_start_v);
  for (auto& path_speed_region :
       stationary_close_object_constraint.path_speed_regions) {
    constraint_mgr->AddPathSpeedRegion(std::move(path_speed_region));
  }
  for (auto& path_stop_line :
       stationary_close_object_constraint.path_stop_lines) {
    constraint_mgr->AddPathStopLine(std::move(path_stop_line));
  }
}

void EraseSmallCutInProtectiveStBoundariesForMergingObjects(
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision) {
  // <id of the non-protective st-boundary, non-protective st-boundary
  // pointer>
  absl::flat_hash_map<std::string, StBoundaryWithDecision*> st_boundary_wd_map;
  for (auto& st_boundary_wd : *st_boundaries_with_decision) {
    if (st_boundary_wd.raw_st_boundary()->is_protective()) {
      continue;
    }
    st_boundary_wd_map.emplace(st_boundary_wd.id(), &st_boundary_wd);
  }

  st_boundaries_with_decision->erase(
      std::remove_if(
          st_boundaries_with_decision->begin(),
          st_boundaries_with_decision->end(),
          [&st_boundary_wd_map](
              const StBoundaryWithDecision& st_boundary_with_decision) {
            const auto& st_boundary =
                *st_boundary_with_decision.raw_st_boundary();
            if (st_boundary.protection_type() !=
                StBoundaryProto::SMALL_ANGLE_CUT_IN) {
              return false;
            }
            const auto& protected_st_boundary_id =
                st_boundary.protected_st_boundary_id();
            if (!protected_st_boundary_id.has_value()) return false;
            const auto original_st_boundary_wd =
                FindOrNull(st_boundary_wd_map, *protected_st_boundary_id);
            if (nullptr == original_st_boundary_wd) return false;
            const auto& original_st_boundary =
                *(*original_st_boundary_wd)->raw_st_boundary();
            return original_st_boundary.overlap_meta()->source() ==
                   StOverlapMetaProto::LANE_MERGE;
          }),
      st_boundaries_with_decision->end());
}
}  // namespace

absl::StatusOr<SpeedDecisionOutput> MapStBoundariesAndMakeSpeedDecision(
    const SpeedDecisionInput& input, StGraph* st_graph,
    ThreadPool* thread_pool) {
  const auto& vehicle_geometry_params =
      *QCHECK_NOTNULL(input.vehicle_geometry_params);
  const auto& motion_constraint_params =
      *QCHECK_NOTNULL(input.motion_constraint_params);
  QCHECK_NOTNULL(input.traj_mgr);
  QCHECK_NOTNULL(input.leading_trajs);
  QCHECK_NOTNULL(input.constraint_mgr);
  QCHECK_NOTNULL(input.path);
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.av_shapes);
  /*
   *  Step 1: Map st boundaries.
   *
   */
  ScopedMultiTimer timer("speed_decision");
  auto start_time = absl::Now();
  auto st_boundaries = st_graph->GetStBoundaries(
      *input.traj_mgr, *input.leading_trajs, *input.constraint_mgr, input.psmm,
      input.drive_passage, input.path_sl_boundary, thread_pool);
  VLOG(2) << "Build st_graph cost time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(3) << "st_boundary size = " << st_boundaries.size();
  timer.Mark("map_st_boundaries");

  /*
   *  Step 2: Analyze overlap.
   *
   */
  std::vector<PathPointSemantic> path_semantics;
  AnalyzePathSemanticsAndStOverlaps(input, &st_boundaries, &path_semantics,
                                    thread_pool);

  /*
   *  Step 3: Pre-decision.
   *
   */
  // Initialize st-boudaries with decision.
  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));

  // NOTE: Don't use input.constraint_mgr below this line.
  auto constraint_mgr = *input.constraint_mgr;
  const auto& speed_finder_params = *QCHECK_NOTNULL(input.speed_finder_params);
  if (speed_finder_params.enable_rule_based_stop_decider()) {
    // Generate stop line for dense traffic flow.
    auto dense_traffic_flow_constraint = GenerateDenseTrafficFlowConstraint(
        st_boundaries_with_decision, *input.traj_mgr, path_semantics,
        *input.path, input.plan_start_v, vehicle_geometry_params);
    // Only generate path stop line.
    for (auto& path_stop_line : dense_traffic_flow_constraint.path_stop_lines) {
      st_boundaries_with_decision.push_back(
          GenerateStBoundaryWithDecisionByStopLine(*st_graph, path_stop_line));
      constraint_mgr.AddPathStopLine(std::move(path_stop_line));
    }
  }

  // Set follow/lead standstill distance for st-boundaries.
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = input.stalled_objects,
      .congested_cutin_object_ids = input.congested_cutin_object_ids,
      .planner_semantic_map_manager = input.psmm,
      .lane_path = input.drive_passage == nullptr
                       ? nullptr
                       : &input.drive_passage->lane_path(),
      .st_traj_mgr = input.traj_mgr,
      .constraint_mgr = &constraint_mgr,
      .extra_follow_standstill_for_large_vehicle =
          PiecewiseLinearFunctionFromProto(
              speed_finder_params
                  .extra_follow_standstill_distance_for_large_vehicle_plf())(
              input.plan_start_v)};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision);
  }
  // Only keep all zero-distance stationary st-boudnaries & the nearest
  // non-zero-distance stationary st-boundary.
  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);

  // Merging behavior is not considered as cut-in.
  EraseSmallCutInProtectiveStBoundariesForMergingObjects(
      &st_boundaries_with_decision);

  const double planner_speed_cap = input.planner_speed_cap;  // m/s.
  // Make ignore decisions for st-boundaries.
  const auto ignore_decider_input = IgnoreDeciderInput(
      {.params = &speed_finder_params.ignore_decider_params(),
       .path = input.path,
       .path_semantics = &path_semantics,
       .psmm = input.psmm,
       .st_traj_mgr = input.traj_mgr,
       .drive_passage = input.drive_passage,
       .vehicle_geometry_params = &vehicle_geometry_params,
       .av_shapes = input.av_shapes,
       .path_kd_tree = input.path_kd_tree,
       .current_v = input.plan_start_v,
       .current_a = input.plan_start_a,
       .max_v = planner_speed_cap,
       .time_step = kSpeedLimitProviderTimeStep,
       .trajectory_steps = input.trajectory_steps});
  std::optional<VtSpeedLimit> ignore_speed_limit;
  MakeIgnoreAndPreBrakeDecisionForStBoundaries(
      ignore_decider_input, &st_boundaries_with_decision, &ignore_speed_limit);

  // Make pre-decisions for st-boundaries.
  std::optional<VtSpeedLimit> parallel_cut_in_speed_limit;
  if (speed_finder_params.enable_pre_decision()) {
    const auto pre_decider_input = PreDeciderInput({
        .params = &speed_finder_params.st_boundary_pre_decider_params(),
        .leading_trajs = input.leading_trajs,
        .follower_set = input.follower_set,
        .lane_change_gap = &constraint_mgr.TrafficGap(),
        .st_traj_mgr = input.traj_mgr,
        .path = input.path,
        .vehicle_geo_params = &vehicle_geometry_params,
        .drive_passage = QCHECK_NOTNULL(input.drive_passage),
        .current_v = input.plan_start_v,
        .max_v = planner_speed_cap,
        .time_step = kSpeedLimitProviderTimeStep,
        .trajectory_steps = input.trajectory_steps,
        .planner_model_pool = input.planner_model_pool,
        .planner_av_context = input.planner_av_context,
        .real_objects = input.real_objects,
        .virtual_objects = input.virtual_objects,
        .plan_time = input.plan_time,
        .run_act_net_speed_decision = input.run_act_net_speed_decision,
    });
    MakePreDecisionForStBoundaries(pre_decider_input,
                                   &st_boundaries_with_decision,
                                   &parallel_cut_in_speed_limit);
  }

  // Set pass/yield time buffers for st-boundaries.
  DecideTimeBuffers(*input.traj_mgr, vehicle_geometry_params,
                    input.plan_start_v, &st_boundaries_with_decision);

  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;
  const PreStboundaryModifierInput pre_st_boundary_modifier_input{
      .params = &speed_finder_params.st_boundary_pre_modifier_params(),
      .st_graph = st_graph,
      .st_traj_mgr = input.traj_mgr,
      .current_v = input.plan_start_v,
      .current_a = input.plan_start_a,
      .path = input.path};
  // NOTE(renjie): We may need to append more path semantics after this point
  // because there may have been newly generated st-boundaries.
  PreModifyStBoundaries(pre_st_boundary_modifier_input,
                        &st_boundaries_with_decision, &processed_st_objects);
  // Add some additional constraints (speed region, stop line, etc) to
  // constraint manager.
  // TODO(ping): Differentiate the speed regions generated by speed finder which
  // are based on the path and those generated by upstream modules which are
  // based on drive passage.
  if (speed_finder_params.close_object_params()
          .enable_stationary_close_object_slowdown()) {
    AddStationaryCloseObjectConstraint(
        st_boundaries_with_decision, *st_graph, *input.traj_mgr, *input.path,
        *QCHECK_NOTNULL(input.drive_passage), *input.path_sl_boundary,
        input.plan_start_v, &constraint_mgr);
    timer.Mark("stationary_close_object_decision");
  }

  /*
   *  Step 4: Calculate speed limit.
   *
   */

  auto speed_limit_map = GetSpeedLimitMap(
      *input.path, *input.st_path_points, planner_speed_cap, input.plan_start_v,
      vehicle_geometry_params, *QCHECK_NOTNULL(input.vehicle_drive_params),
      input.drive_passage, constraint_mgr,
      speed_finder_params.speed_limit_params(),
      st_graph->distance_info_to_impassable_boundaries());
  timer.Mark("get_speed_limit");

  // Moving close object speed limit.
  std::unordered_map<std::string, double> overlap_trajs_info;
  for (const auto& stb_wd : st_boundaries_with_decision) {
    if (const auto traj_id = stb_wd.traj_id(); traj_id.has_value()) {
      const auto& st_boundary = stb_wd.raw_st_boundary();
      overlap_trajs_info[*traj_id] = st_boundary->bottom_left_point().s();
    }
  }

  std::vector<std::optional<SpeedLimit>> close_traj_speed_limit;
  if (FLAGS_planner_enable_moving_close_traj_speed_limit) {
    close_traj_speed_limit = GetMovingCloseTrajSpeedLimits(
        st_graph->moving_close_trajs(), input.path->length(),
        std::max(0.0, input.plan_start_v), kSpeedLimitProviderTimeStep,
        speed_finder_params.speed_limit_params().moving_close_traj_max_time());
  }

  std::map<SpeedLimitTypeProto::Type, VtSpeedLimit> vt_speed_limit_map;
  vt_speed_limit_map[SpeedLimitTypeProto::EXTERNAL] = GetExternalVtSpeedLimit(
      constraint_mgr, input.trajectory_steps, kSpeedLimitProviderTimeStep);

  if (parallel_cut_in_speed_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto::NEAR_PARALLEL_VEHICLE] =
        *parallel_cut_in_speed_limit;
  }
  if (ignore_speed_limit.has_value()) {
    vt_speed_limit_map[SpeedLimitTypeProto::IGNORE_OBJECT] =
        *ignore_speed_limit;
  }

  // Construct speed limit provider.
  SpeedLimitProvider speed_limit_provider(
      std::move(speed_limit_map), std::move(close_traj_speed_limit),
      std::move(vt_speed_limit_map), kSpeedLimitProviderTimeStep);

  /*
   *  Step 5: Coarse speed planning given st-graph and speed limit, and make
   *          decisions on st-boundaries at the same time.
   */
  start_time = absl::Now();
  SpeedVector preliminary_speed;
  InteractiveSpeedDebugProto interactive_speed_debug;
  RETURN_IF_ERROR(MakeInteractiveSpeedDecision(
      input.base_name, vehicle_geometry_params, motion_constraint_params,
      *st_graph, *input.traj_mgr, *input.path, input.plan_start_v,
      input.plan_start_a, speed_finder_params, planner_speed_cap,
      input.trajectory_steps, &speed_limit_provider, &preliminary_speed,
      &st_boundaries_with_decision, &processed_st_objects,
      &interactive_speed_debug));
  timer.Mark("speed_search");
  // Record decision inconsistency.
  EvaluateActNetSpeedDecision(st_boundaries_with_decision);

  if (speed_finder_params.enable_pre_brake()) {
    const auto uncertain_vehicle_speed_limit =
        MakeUncertainVehiclePreBrakeDecision(
            *input.traj_mgr, *input.path, vehicle_geometry_params,
            input.plan_start_v, planner_speed_cap, kSpeedLimitProviderTimeStep,
            input.trajectory_steps, preliminary_speed,
            &st_boundaries_with_decision);

    if (uncertain_vehicle_speed_limit.has_value()) {
      speed_limit_provider.AddVtSpeedLimit(
          SpeedLimitTypeProto::UNCERTAIN_VEHICLE,
          *uncertain_vehicle_speed_limit);
    }
  }
  return SpeedDecisionOutput{
      .st_boundaries_with_decision = std::move(st_boundaries_with_decision),
      .speed_limit_provider = std::move(speed_limit_provider),
      .constraint_mgr = std::move(constraint_mgr),
      .processed_st_objects = std::move(processed_st_objects),
      .preliminary_speed = std::move(preliminary_speed),
      .overlap_trajs_info = std::move(overlap_trajs_info),
      .interactive_speed_debug = std::move(interactive_speed_debug)};
}
}  // namespace qcraft::planner
