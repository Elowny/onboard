#include "onboard/planner/initializer/search_motion.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include "Eigen/Core"

#include <algorithm>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/async/async_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/leading_object.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/astar_motion_searcher.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/dp_motion_searcher.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_graph_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph_cache.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/reference_line_searcher.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/initializer_models/initializer_feature_extractor.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {

constexpr double kHighSpeedThreshold = 14.0;              // m/s ~ 50 km/h.
constexpr double kMediumSpeedThreshold = 8.0;             // m/s ~ 30 km/h.
constexpr double kLowSpeedThreshold = 2.0;                // m/s ~ 7 km/h.
constexpr double kSpeedHysteresis = 1.0;                  // m/s.
constexpr double kStationaryObjectCollisionBuffer = 0.3;  // m on each side.
constexpr double kMovingObjectCollisionBuffer = 0.5;      // m on each side.
constexpr double kReduceStaticBufferThresholdS = 15.0;    // m.
constexpr double kMaxSamplingDistance = 220.0;            // meters.
constexpr double kMaxSamplingLookForwardTime = 15.0;      // seconds.
constexpr double kMinSamplingDistance = 60.0;
constexpr double kInitializerLKTimeConsumptionReportThreshold = 30;  // ms.
constexpr double kInitializerLCTimeConsumptionReportThreshold = 50;  // ms.
constexpr double kQeventSeconds = 5.0;                               // s.

GeometryGraphSamplingStrategy ParseStrategy(
    const InitializerConfig::InitializerSamplePattern& sample_pattern,
    bool is_lane_change) {
  GeometryGraphSamplingStrategy strategy;

  strategy.is_lane_change = is_lane_change;
  const int config_len = sample_pattern.config().range().size();
  strategy.range_list.reserve(config_len);
  strategy.layer_gap_list.reserve(config_len);
  strategy.lateral_resolution_list.reserve(config_len);
  strategy.cross_layer_connection_list.reserve(config_len);
  strategy.unit_length_lateral_span_list.reserve(config_len);

  for (const auto& val : sample_pattern.config().range()) {
    strategy.range_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().layer_gap()) {
    strategy.layer_gap_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().lateral_resolution()) {
    strategy.lateral_resolution_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().cross_layer_connection()) {
    strategy.cross_layer_connection_list.push_back(val);
  }
  for (const auto& val : sample_pattern.config().unit_length_lateral_span()) {
    strategy.unit_length_lateral_span_list.push_back(val);
  }
  return strategy;
}

GeometryGraphSamplingStrategy FindStrategy(
    InitializerSamplePatternConfig sample_pattern_config,
    const InitializerConfig& initializer_config, bool is_lane_change) {
  const auto scenario =
      is_lane_change
          ? InitializerConfig::InitializerSamplePattern::SCENARIO_LANE_CHANGE
          : InitializerConfig::InitializerSamplePattern::SCENARIO_LANE_KEEPING;

  for (const auto& sample_pattern : initializer_config.sample_patterns()) {
    if (sample_pattern.config_name() == sample_pattern_config &&
        sample_pattern.scenario() == scenario) {
      return ParseStrategy(sample_pattern, is_lane_change);
    }
  }
  // Cannot find a sample pattern, qcheck failed.
  QCHECK(false);
  return GeometryGraphSamplingStrategy();
}

InitializerSamplePatternConfig FindPattern(
    double cur_v, InitializerSamplePatternConfig prev_sample_config) {
  double high_speed_threshold = kHighSpeedThreshold;
  double medium_speed_threshold = kMediumSpeedThreshold;
  double low_speed_threshold = kLowSpeedThreshold;
  switch (prev_sample_config) {
    case InitializerSamplePatternConfig::ISC_NONE:
      break;
    case InitializerSamplePatternConfig::ISC_HIGH_SPEED:
      high_speed_threshold = high_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_MEDIUM_SPEED:
      medium_speed_threshold = medium_speed_threshold - kSpeedHysteresis;
      break;
    case InitializerSamplePatternConfig::ISC_LOW_SPEED:
      break;
    case InitializerSamplePatternConfig::ISC_CREEP_SPEED:
      // Hysteresis in the opposite direction here.
      low_speed_threshold = low_speed_threshold + kSpeedHysteresis;
      break;
  }
  if (cur_v > high_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_HIGH_SPEED;
  } else if (cur_v > medium_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_MEDIUM_SPEED;
  } else if (cur_v > low_speed_threshold) {
    return InitializerSamplePatternConfig::ISC_LOW_SPEED;
  } else {
    return InitializerSamplePatternConfig::ISC_CREEP_SPEED;
  }
}

// Get corresponding sample pattern based on current speed.
std::pair<InitializerSamplePatternConfig, GeometryGraphSamplingStrategy>
GetSamplingStrategy(const InitializerConfig& config, bool is_lane_change,
                    double cur_v,
                    InitializerSamplePatternConfig prev_sample_config) {
  InitializerSamplePatternConfig cur_pattern =
      FindPattern(cur_v, prev_sample_config);
  GeometryGraphSamplingStrategy strategy =
      FindStrategy(cur_pattern, config, is_lane_change);
  return std::make_pair(cur_pattern, std::move(strategy));
}

std::vector<double> ConvertStoplineToStopS(
    absl::Span<const ConstraintProto::StopLineProto> stoplines,
    double front_to_ra) {
  std::vector<double> stop_s;
  stop_s.reserve(stoplines.size());
  for (const auto& stop_line : stoplines) {
    stop_s.push_back(std::max(stop_line.s() - front_to_ra, 0.0));
  }
  return stop_s;
}

bool EnableInitializerLaneChangeTargetDecision(
    const LaneChangeStateProto& lc_state) {
  return (lc_state.stage() == LaneChangeStage::LCS_EXECUTING ||
          lc_state.stage() == LaneChangeStage::LCS_RETURN) &&
         !lc_state.entered_target_lane();
}

void PrintTimeOutQevent(bool is_lane_change, int multi_traj_size,
                        double perform_duration) {
  if (is_lane_change &&
      perform_duration > kInitializerLCTimeConsumptionReportThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "changqing", "initializer_LC_time_greater_than_threhold",
        kQeventSeconds, [&](QEvent* qevent) {
          qevent->AddField("num_trajs_considered", multi_traj_size)
              .AddField("time", perform_duration);
        });
  }

  if (!is_lane_change &&
      perform_duration > kInitializerLKTimeConsumptionReportThreshold) {
    QEVENT_EVERY_N_SECONDS(
        "changqing", "initializer_LK_time_greater_than_threhold",
        kQeventSeconds, [&](QEvent* qevent) {
          qevent->AddField("num_trajs_considered", multi_traj_size)
              .AddField("time", perform_duration);
        });
  }
}

}  // namespace

absl::StatusOr<MotionSearchOutput> SearchMotion(const MotionSearchInput& input,
                                                ThreadPool* thread_pool) {
  absl::StatusOr<MotionSearchOutput> output_or;
  switch (input.initializer_params->search_algorithm()) {
    case InitializerConfig::DP:
      output_or = DpSearchForRawTrajectory(input, thread_pool);
      break;
    case InitializerConfig::Astar:
      if (input.is_run_model_l4 &&
          input.captain_net_output->traj_points.empty()) {
        output_or = DpSearchForRawTrajectory(input, thread_pool);
      } else {
        output_or = AstarSearchForRawTrajectory(input, thread_pool);
      }
      break;
  }
  return output_or;
}

// NOLINTNEXTLINE(readability-function-size)
absl::StatusOr<InitializerOutput> RunInitializer(
    const InitializerInput& initializer_input,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    SchedulerOutput* scheduler_output, DeciderOutput* decider_output,
    InitializerDebugProto* debug_proto,
    LaneChangeSafetyDebugProto* lane_change_safety_debug_proto,
    vis::vantage::ChartDataBundleProto* charts, ThreadPool* thread_pool) {
  const auto plan_time = initializer_input.path_start_point_info->plan_time;
  const auto& lane_change_state = *initializer_input.lane_change_state;
  const auto& drive_passage = *initializer_input.drive_passage;
  const auto& st_traj_mgr = *initializer_input.st_traj_mgr;
  const auto& vehicle_geom =
      initializer_input.vehicle_params->vehicle_geometry_params();
  const auto& vehicle_drive =
      initializer_input.vehicle_params->vehicle_drive_params();
  const auto& path_sl_boundary = *initializer_input.sl_boundary;
  const auto& initializer_state = *initializer_input.prev_initializer_state;
  const auto& decision_constraint_config =
      *initializer_input.decision_constraint_config;
  const auto& initializer_params = *initializer_input.initializer_params;
  const auto& motion_constraint_params =
      *initializer_input.motion_constraint_params;
  const auto plan_id = initializer_input.plan_id;
  const auto& stalled_objects = *initializer_input.stalled_objects;
  const auto& st_planner_object_traj =
      *initializer_input.st_planner_object_traj;
  const auto& planner_semantic_map_manager =
      *initializer_input.planner_semantic_map_manager;
  const auto& av_frenet_box = *initializer_input.av_frenet_box;

  SCOPED_QTRACE("EstPlanner/Initializer");
  ScopedMultiTimer initializer_timer("initializer_debug");
  initializer_timer.Mark("initializer start");

  auto mutable_start_point =
      initializer_input.path_start_point_info->start_point;  // Copy
  mutable_start_point.mutable_path_point()->set_theta(NormalizeAngle(
      mutable_start_point.path_point().theta()));  // Fix heading angle.
  const auto& path_start_point = mutable_start_point;

  const bool is_lane_change =
      (lane_change_state.stage() == LaneChangeStage::LCS_EXECUTING ||
       lane_change_state.stage() == LaneChangeStage::LCS_RETURN ||
       lane_change_state.stage() == LaneChangeStage::LCS_PAUSE);

  // Create collision checker.
  std::unique_ptr<CollisionChecker> collision_checker =
      std::make_unique<BoxGroupCollisionChecker>(
          &st_planner_object_traj, &vehicle_geom,
          MotionForm::kConstTimeIntervalSampleStep,
          kStationaryObjectCollisionBuffer, kMovingObjectCollisionBuffer);

  const InitializerSamplePatternConfig prev_config =
      initializer_state.has_sample_pattern_config()
          ? initializer_state.sample_pattern_config()
          : InitializerSamplePatternConfig::ISC_NONE;
  const auto [sample_pattern, sample_strategy] = GetSamplingStrategy(
      initializer_params, is_lane_change, path_start_point.v(), prev_config);
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(path_start_point);
  ASSIGN_OR_RETURN(const auto ego_sl,
                   drive_passage.QueryFrenetCoordinateAt(ego_pos),
                   _ << "Failed to project ego position on drive passage.");

  debug_proto->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));

  // Get smooth reference line max length.
  const double sampling_dist_by_speed = std::max(
      path_start_point.v() * kMaxSamplingLookForwardTime, kMinSamplingDistance);
  const double max_sampling_acc_s = std::min(
      drive_passage.end_s(),
      std::min(kMaxSamplingDistance, sampling_dist_by_speed) + ego_sl.s);

  // Get s_from_start.
  double s_from_start = 0.0;
  if (!initializer_input.path_start_point_info->reset &&
      FLAGS_planner_async_low_freq_cycle_iterations == 0) {
    // On async mode, `prev_start_point` is not the actual prev pos, so skip.
    if (initializer_state.has_s_from_start() &&
        initializer_state.has_prev_start_point()) {
      const auto prev_pos = Vec2dFromApolloTrajectoryPointProto(
          initializer_state.prev_start_point());
      ASSIGN_OR_RETURN(const auto prev_sl,
                       drive_passage.QueryFrenetCoordinateAt(prev_pos),
                       _ << "Failed to project previous ego position ("
                         << prev_pos.transpose() << ") on drive passage.");
      s_from_start = initializer_state.s_from_start() - prev_sl.s + ego_sl.s;
    }
  }
  InitializerStateProto new_state;
  new_state.set_sample_pattern_config(sample_pattern);
  new_state.set_s_from_start(s_from_start);
  *new_state.mutable_prev_start_point() = path_start_point;

  XYGeometryGraph geom_graph;
  auto graph_cache = std::make_unique<GeometryGraphCache>();
  const double s_from_start_with_diff = s_from_start - ego_sl.s;
  // Not const, to be destroyed asynchronously.
  auto form_builder = std::make_unique<GeometryFormBuilder>(
      &drive_passage, max_sampling_acc_s, s_from_start_with_diff);

  // Leading decider
  const bool enable_lc_multi_traj =
      EnableInitializerLaneChangeTargetDecision(lane_change_state);
  std::vector<LeadingGroup> leading_groups;
  if (enable_lc_multi_traj) {
    // Generate multiple leading group candidates for initializer's
    // multiple trajectories.
    VLOG(3) << "lc_multiple_traj true: constructing leading object groups";
    if (initializer_input.is_run_model_l4 &&
        !initializer_input.captain_net_output->traj_points.empty() &&
        FLAGS_planner_use_ml_trajectory_to_derive_leading_objects) {
      // Only generate single leading group if using captainnet
      leading_groups = DeriveMultipleLeadingGroupsFromCaptainNetTrajectory(
          st_traj_mgr, drive_passage, stalled_objects, vehicle_geom,
          initializer_input.captain_net_output->traj_points);
    } else {
      leading_groups = FindMultipleLeadingGroups(
          drive_passage, path_sl_boundary, lane_change_state.lc_left(),
          st_traj_mgr, stalled_objects, path_start_point.path_point().theta(),
          av_frenet_box, vehicle_geom, path_start_point.v());

      leading_groups.push_back({});  // Take no trajectory as leading.
    }
  } else {
    // Do normal leading objects extraction and generate one single group.
    std::vector<ConstraintProto::LeadingObjectProto> leading_trajs;
    if (initializer_input.is_run_model_l4 &&
        !initializer_input.captain_net_output->traj_points.empty() &&
        FLAGS_planner_use_ml_trajectory_to_derive_leading_objects) {
      leading_trajs = DeriveLeadingObjectsFromCaptainNetTrajectory(
          st_traj_mgr, drive_passage, stalled_objects, vehicle_geom,
          initializer_input.captain_net_output->traj_points);
    } else {
      leading_trajs = FindLeadingObjects(
          planner_semantic_map_manager, drive_passage, path_sl_boundary,
          lane_change_state.stage(), *initializer_input.scene_reasoning,
          st_traj_mgr, stalled_objects, path_start_point, vehicle_geom,
          av_frenet_box, initializer_input.borrow_lane);
    }
    auto& traj_group = leading_groups.emplace_back();
    for (auto& leading_traj : leading_trajs) {
      traj_group.emplace(leading_traj.traj_id(), std::move(leading_traj));
    }
  }

  const auto stop_s_vec =
      ConvertStoplineToStopS(decider_output->constraint_manager.StopLine(),
                             vehicle_geom.front_edge_to_center());

  switch (initializer_params.search_algorithm()) {
    case InitializerConfig::Astar:
    case InitializerConfig::DP:
      const CurvyGeometryGraphBuilderInput geom_graph_builder_input = {
          .passage = &drive_passage,
          .sl_boundary = &path_sl_boundary,
          .stop_s_vec = &stop_s_vec,
          .leading_groups = &leading_groups,
          .st_traj_mgr = &st_traj_mgr,
          .plan_start_point = &path_start_point,
          .s_from_start = s_from_start,
          .vehicle_geom = &vehicle_geom,
          .collision_checker = collision_checker.get(),
          .sampling_params = &sample_strategy,
          .vehicle_drive = &vehicle_drive,
          .form_builder = form_builder.get(),
          .lc_multiple_traj = enable_lc_multi_traj};
      ASSIGN_OR_RETURN(
          geom_graph,
          BuildCurvyGeometryGraph(geom_graph_builder_input,
                                  /*retry_collision_checker=*/false,
                                  graph_cache.get(), thread_pool, debug_proto),
          MakeAebInitializerOutput(
              initializer_params.traj_steps(), std::move(mutable_start_point),
              std::move(new_state), _.JoinMessageToStatus().message(),
              debug_proto));

      // If the constructed graph is short & blocked by static obj, we try a
      // smaller buffer.
      const auto& geom_end_info = geom_graph.GetGeometryGraphEndInfo();
      if (geom_end_info.end_reason() == GeometryGraphProto::END_STATIC_OBJ &&
          geom_end_info.end_accumulated_s() - ego_sl.s <
              kReduceStaticBufferThresholdS) {
        VLOG(2) << "Failed to construct graph, try a smaller stationary object "
                   "buffer.";
        collision_checker->UpdateStationaryObjectBuffer(
            0.5 * kStationaryObjectCollisionBuffer);
        ASSIGN_OR_RETURN(
            geom_graph,
            BuildCurvyGeometryGraph(
                geom_graph_builder_input, /*retry_collision_checker=*/true,
                graph_cache.get(), thread_pool, debug_proto),
            MakeAebInitializerOutput(
                initializer_params.traj_steps(), std::move(mutable_start_point),
                std::move(new_state), _.JoinMessageToStatus().message(),
                debug_proto));
      }
      initializer_timer.Mark("build curvy xy geometry graph");
      break;
  }
  VLOG(1) << "Done build geometry graph";

  // Add blocking object stop line to constraint manager if the geometry graph
  // is blocked by some object.
  const auto& geom_end_info = geom_graph.GetGeometryGraphEndInfo();
  std::unique_ptr<ConstraintProto::LeadingObjectProto> blocking_static_traj =
      nullptr;
  if (geom_end_info.end_reason() == GeometryGraphProto::END_STATIC_OBJ &&
      !stalled_objects.contains(geom_end_info.object_id())) {
    const auto blocking_frenet_box_or = drive_passage.QueryFrenetBoxAt(
        st_traj_mgr.FindObjectByObjectId(geom_end_info.object_id())
            ->bounding_box());
    if (blocking_frenet_box_or.ok() &&
        blocking_frenet_box_or->s_min > av_frenet_box.s_max) {
      const auto trajs =
          st_traj_mgr.FindTrajectoriesByObjectId(geom_end_info.object_id());
      QCHECK_GT(trajs.size(), 0);
      blocking_static_traj =
          std::make_unique<ConstraintProto::LeadingObjectProto>(
              CreateLeadingObject(
                  *trajs[0], drive_passage,
                  ConstraintProto::LeadingObjectProto::BLOCKING_STATIC));
    }
  }

  auto* graph_proto = debug_proto->mutable_geom_graph();
  graph_proto->Clear();

  absl::StatusOr<ReferenceLineSearcherOutput> ref_line_output_or;
  const std::vector<Vec2d> empty_ref_line_points;
  if ((!initializer_input.is_run_model_l4 &&
       initializer_params.search_algorithm() == InitializerConfig::Astar &&
       FLAGS_planner_initializer_astar_inspired_by_reference_line) ||
      FLAGS_planner_initializer_only_activate_nodes_near_refline) {
    ReferenceLineSearcherInput ref_line_search_input{
        .geometry_graph = &geom_graph,
        .drive_passage = &drive_passage,
        .sl_boundary = &path_sl_boundary,
        .initializer_params = &initializer_params,
        .vehicle_geom = &vehicle_geom,
        .vehicle_drive = &vehicle_drive,
        .st_planner_object_traj = &st_planner_object_traj,
    };
    ref_line_output_or =
        SearchReferenceLine(ref_line_search_input, debug_proto, thread_pool);
    if (ref_line_output_or.ok() && FLAGS_planner_initializer_debug_level >= 1) {
      ParseReferenceLineResultToProto(*ref_line_output_or, graph_proto);
    }
    initializer_timer.Mark("search reference line");
  }

  if (FLAGS_planner_initializer_only_activate_nodes_near_capnet_traj) {
    RETURN_IF_ERROR(DeactivateFarGeometries(
        initializer_input.captain_net_output->traj_points, path_sl_boundary,
        &geom_graph));
  } else if (!is_lane_change &&
             FLAGS_planner_initializer_only_activate_nodes_near_refline &&
             ref_line_output_or.ok()) {
    RETURN_IF_ERROR(DeactivateFarGeometries(*ref_line_output_or,
                                            path_sl_boundary, &geom_graph));
  }

  if (FLAGS_planner_initializer_debug_level >= 1) {
    geom_graph.ToProto(graph_proto);
    form_builder->FillSmoothDrivePassage(graph_proto);
  }

  const bool eval_safety =
      !FLAGS_planner_est_scheduler_seperate_lc_pause && is_lane_change &&
      lane_change_state.stage() != LaneChangeStage::LCS_PAUSE &&
      !(lane_change_state.force_merge() ||
        lane_change_state.entered_target_lane());
  const ml::captain_net::CaptainNetOutput empty_captain_net_output;
  MotionSearchInput motion_search_input{
      .planner_semantic_map_manager = &planner_semantic_map_manager,
      .start_point = &path_start_point,
      .path_look_ahead_duration = initializer_input.path_look_ahead_duration,
      .plan_time = plan_time,
      .drive_passage = &drive_passage,
      .sl_boundary = &path_sl_boundary,
      .st_traj_mgr = &st_traj_mgr,
      .initializer_params = &initializer_params,
      .motion_constraint_params = &motion_constraint_params,
      .vehicle_geom = &vehicle_geom,
      .geom_graph = &geom_graph,
      .reference_line_points = ref_line_output_or.ok()
                                   ? &((*ref_line_output_or).ref_line_points)
                                   : &empty_ref_line_points,
      .form_builder = form_builder.get(),
      .collision_checker = collision_checker.get(),
      .stop_s_vec = &stop_s_vec,
      .leading_groups = &leading_groups,
      .blocking_static_traj = blocking_static_traj.get(),
      .captain_net_output =
          FLAGS_planner_use_ml_trajectory_as_initializer_ref_traj
              ? initializer_input.captain_net_output
              : &empty_captain_net_output,
      .is_lane_change = is_lane_change,
      .eval_safety = eval_safety,
      .lc_style = initializer_input.lane_change_style,
      .log_av_trajectory = initializer_input.log_av_trajectory,
      .is_run_model_l4 = initializer_input.is_run_model_l4};
  initializer_timer.Mark("create initializer input");

  ASSIGN_OR_RETURN(
      auto motion_output, SearchMotion(motion_search_input, thread_pool),
      MakeAebInitializerOutput(initializer_params.traj_steps(),
                               std::move(mutable_start_point),
                               std::move(new_state),
                               _.JoinMessageToStatus().message(), debug_proto));
  if (!motion_output.result_status.ok()) {
    return absl::CancelledError(
        absl::StrCat("Motion search output invalid: ",
                     motion_output.result_status.message()));
  }

  *lane_change_safety_debug_proto =
      std::move(motion_output.lane_change_safety_debug_proto);
  // Should only happen if lc safety check has been applied.
  if (motion_output.is_lc_pause) {
    *unsafe_object_ids = std::move(motion_output.unsafe_object_ids);
    // Modify scheduler output for lc pause.
    scheduler_output->lane_change_state.set_stage(LaneChangeStage::LCS_PAUSE);
    ASSIGN_OR_RETURN(
        scheduler_output->sl_boundary,
        BuildPathBoundaryFromPose(
            planner_semantic_map_manager, drive_passage,
            initializer_input.start_point_info->start_point, vehicle_geom,
            st_traj_mgr, scheduler_output->lane_change_state,
            *initializer_input.smooth_result_map, scheduler_output->borrow_lane,
            scheduler_output->should_smooth, *unsafe_object_ids),
        _ << "Rebuilding path boundary failed.");

    // Find the corresponding leading trajectories.
    std::vector<ConstraintProto::LeadingObjectProto> leading_trajs;
    if (initializer_input.is_run_model_l4 &&
        !initializer_input.captain_net_output->traj_points.empty() &&
        FLAGS_planner_use_ml_trajectory_to_derive_leading_objects) {
      leading_trajs = DeriveLeadingObjectsFromCaptainNetTrajectory(
          st_traj_mgr, drive_passage, stalled_objects, vehicle_geom,
          initializer_input.captain_net_output->traj_points);
    } else {
      leading_trajs =
          FindLeadingObjects(planner_semantic_map_manager, drive_passage,
                             scheduler_output->sl_boundary,
                             scheduler_output->lane_change_state.stage(),
                             *initializer_input.scene_reasoning, st_traj_mgr,
                             stalled_objects, path_start_point, vehicle_geom,
                             av_frenet_box, scheduler_output->borrow_lane);
    }
    for (auto& leading_traj : leading_trajs) {
      motion_output.leading_trajs.emplace(leading_traj.traj_id(),
                                          std::move(leading_traj));
    }

    // Rerun constraint builder for lc pause case.
    DeciderInput decider_input{
        .vehicle_geometry_params = &vehicle_geom,
        .motion_constraint_params = &motion_constraint_params,
        .config = &decision_constraint_config,
        .planner_semantic_map_manager = &planner_semantic_map_manager,
        .lc_state = &scheduler_output->lane_change_state,
        .plan_start_point = &initializer_input.start_point_info->start_point,
        .lane_path_before_lc = &scheduler_output->lane_path_before_lc,
        .passage = &drive_passage,
        .sl_boundary = &scheduler_output->sl_boundary,
        .ego_frenet_box = &av_frenet_box,
        .borrow_lane_boundary = scheduler_output->borrow_lane,
        .obj_mgr = initializer_input.obj_mgr,
        .st_traj_mgr = &st_traj_mgr,
        .tl_info_map = initializer_input.tl_info_map,
        .pre_decider_state = initializer_input.prev_decider_state,
        .parking_brake_release_time =
            initializer_input.parking_brake_release_time,
        .teleop_enable_traffic_light_stop =
            initializer_input.enable_traffic_light_stopping,
        .enable_pull_over = initializer_input.enable_pull_over,
        .brake_to_stop = initializer_input.brake_to_stop,
        .max_reach_length = scheduler_output->max_reach_length,
        .vehicle_model =
            initializer_input.vehicle_params->vehicle_params().model(),
        .plan_time = initializer_input.start_point_info->plan_time,
        .scene_reasoning = initializer_input.scene_reasoning,
        .enable_stop_polyline_stopping = false,
        .is_engage_steer_only = false,
        .enable_force_stop = initializer_input.enable_force_stop};
    ASSIGN_OR_RETURN(auto lcp_decider_output, BuildConstraints(decider_input),
                     _ << "Rebuilding decision constraints failed.");
    *decider_output = std::move(lcp_decider_output);

    const double target_l =
        scheduler_output->sl_boundary.QueryReferenceCenterL(ego_sl.s);
    ASSIGN_OR_RETURN(auto lcp_traj,
                     GenerateConstLateralAccelConstSpeedTraj(
                         drive_passage, vehicle_geom.front_edge_to_center(),
                         target_l, motion_output.leading_trajs,
                         decider_output->constraint_manager.StopLine(),
                         path_start_point, initializer_params.traj_steps()),
                     _ << "Generating lc pause trajectory failed.");
    motion_output.multi_traj_candidates.insert(
        motion_output.multi_traj_candidates.begin(),
        MotionSearchOutput::MultiTrajCandidate{.trajectory = lcp_traj});
    motion_output.traj_points = std::move(lcp_traj);
  }

  initializer_timer.Mark("run search motion");

  if (FLAGS_planner_initializer_debug_level >= 2) {
    PrintMultiTimerReportStat(initializer_timer);
  }
  const auto perform_duration =
      absl::ToDoubleMilliseconds(absl::Now() - initializer_timer.start());

  // Print Qevent if initializer timeout
  PrintTimeOutQevent(is_lane_change, motion_output.multi_traj_candidates.size(),
                     perform_duration);

  ParseMotionSearchOutputToInitializerResult(motion_output, debug_proto);
  ParseMotionSearchOutputToMultiTrajDebugProto(
      motion_output, debug_proto->mutable_multi_traj_debug());
  if (FLAGS_planner_initializer_debug_level >= 1) {
    if (!motion_output.is_lc_pause) {
      ParseMotionSearchOutputToMotionSearchDebugProto(
          motion_output, debug_proto->mutable_motion_search_debug());
    }
    if (FLAGS_planner_initializer_debug_level >= 2) {
      for (int i = 0; i < motion_output.multi_traj_candidates.size(); i++) {
        const auto& traj_info = motion_output.multi_traj_candidates.at(i);
        SendSingleTrajectoryToCanvas(traj_info, i, plan_id);
      }
      SendRefSpeedTableToCanvas(*motion_output.ref_speed_table, drive_passage);
    }
  }
  ExportMoionSpeedProfileToChart(motion_output, charts->add_charts());

  DestroyContainerAsyncMarkSource(std::move(form_builder), "form_builder");
  DestroyContainerAsyncMarkSource(std::move(graph_cache),
                                  "geometry_graph_cache");

  if (FLAGS_dumping_initializer_features) {
    ParseFeaturesDumpingProto(motion_output,
                              debug_proto->mutable_expert_evaluation(),
                              debug_proto->mutable_candidates_evaluation());
  }

  // Collect corresponding leading trajectory information to output.
  auto& leading_trajs = motion_output.leading_trajs;
  // Non-stalled blocking static should also be a leading trajectory.
  if (blocking_static_traj != nullptr &&
      leading_trajs.find(blocking_static_traj->traj_id()) ==
          leading_trajs.end()) {
    const auto obj_id = SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
        blocking_static_traj->traj_id());
    if (!stalled_objects.contains(obj_id)) {
      leading_trajs.emplace(blocking_static_traj->traj_id(),
                            *blocking_static_traj);
    }
  }
  for (const auto& [_, leading_proto] : leading_trajs) {
    *debug_proto->add_leading_objects() = leading_proto;
  }

  return InitializerOutput{
      .follower_set = std::move(motion_output.follower_set),
      .follower_max_decel = motion_output.follower_max_decel,
      .is_lc_pause = motion_output.is_lc_pause,
      .traj_points = std::move(motion_output.traj_points),
      .initializer_state = std::move(new_state),
      .leading_trajs = std::move(leading_trajs)};
}

}  // namespace qcraft::planner
