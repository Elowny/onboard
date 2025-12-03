#include "onboard/planner/plan/acc/acc_speed_decision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "onboard/async/async_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/speed/interactive_speed_decision.h"
#include "onboard/planner/speed/overlap_info.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_decision_util.h"
#include "onboard/planner/speed/speed_finder_util.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/planner/speed/st_overlap_analyzer.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/standstill_distance_decider.h"
#include "onboard/planner/speed/time_buffer_decider.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace internal {
constexpr double kCrossingThreshold = M_PI / 6.0;

StOverlapMetaProto AnalyzeStOverlapForACC(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  StOverlapMetaProto overlap_meta;

  const auto& overlap_infos = st_boundary->overlap_infos();
  QCHECK(!overlap_infos.empty());
  const auto& first_overlap_info = overlap_infos.front();
  const auto& av_idx =
      (first_overlap_info.av_start_idx + first_overlap_info.av_end_idx) / 2;
  const auto& obj_idx = first_overlap_info.obj_idx;
  const auto av_heading = path[av_idx].theta();
  const auto obj_heading = st_traj.states()[obj_idx].traj_point->theta();
  const double theta_diff = NormalizeAngle(obj_heading - av_heading);

  // Record theta diff.
  overlap_meta.set_theta_diff(theta_diff);
  // Record overlap pattern.
  overlap_meta.set_pattern(AnalyzeOverlapPattern(st_boundary, st_traj, path,
                                                 vehicle_geometry_params));

  // If the object is crossing, set to lane_cross, otherwise set to cutin.
  if (overlap_meta.pattern() == StOverlapMetaProto::CROSS &&
      theta_diff > kCrossingThreshold) {
    overlap_meta.set_source(StOverlapMetaProto::LANE_CROSS);
  } else {
    overlap_meta.set_source(StOverlapMetaProto::OBJECT_CUTIN);
  }

  // In ACC, we only consider objects that are really close, thus we assume
  // AV always have lower priority than object.
  overlap_meta.set_priority(StOverlapMetaProto::LOW);
  overlap_meta.set_priority_reason("ACC Cutin");

  // Decide if the st boundary is modifiable.
  if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
      st_boundary->object_type() == StBoundaryProto::CYCLIST) {
    overlap_meta.set_modification_type(StOverlapMetaProto::LON_MODIFIABLE);
  } else {
    overlap_meta.set_modification_type(StOverlapMetaProto::NON_MODIFIABLE);
  }

  // Do not set the following field:
  // optional double time_to_lc_complete = 6;
  QCHECK(!overlap_meta.has_time_to_lc_complete());
  // optional bool is_making_u_turn = 7;
  QCHECK(!overlap_meta.has_is_making_u_turn());
  // optional bool is_merging_straight_lane = 8;
  QCHECK(!overlap_meta.has_is_merging_straight_lane());
  // optional bool is_crossing_straight_lane = 9;
  QCHECK(!overlap_meta.has_is_crossing_straight_lane());
  // optional double front_most_projection_distance = 11;
  QCHECK(!overlap_meta.has_front_most_projection_distance());
  // optional double rear_most_projection_distance = 12;
  QCHECK(!overlap_meta.has_rear_most_projection_distance());
  VLOG(2) << "St boundary id " << st_boundary->id()
          << " overlap meta: " << overlap_meta.DebugString();
  return overlap_meta;
}

void AnalyzeStOverlapsForACC(
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::vector<StBoundaryRef>* st_boundaries) {
  for (auto& st_boundary : *st_boundaries) {
    if (!IsAnalyzableStBoundary(st_boundary)) continue;
    // All st-object-generated st-boundaries must have a traj_id.
    QCHECK(st_boundary->traj_id().has_value());
    const auto& traj_id = *st_boundary->traj_id();
    const auto* traj = QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    auto overlap_meta = AnalyzeStOverlapForACC(st_boundary, *traj, path,
                                               vehicle_geometry_params);
    st_boundary->set_overlap_meta(std::move(overlap_meta));
  }
}

// Filter StBoundaries by cutin time.
std::vector<StBoundaryRef> FilterStBoundariesByCutinTime(
    absl::Span<StBoundaryRef> boundaries,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  std::vector<StBoundaryRef> filtered_boundaries;
  filtered_boundaries.reserve(boundaries.size());
  absl::flat_hash_set<std::string> to_filtered;
  for (const auto& bound : boundaries) {
    const auto& overlap_infos = bound->overlap_infos();
    QCHECK(!overlap_infos.empty());
    const auto& first_overlap_info = overlap_infos.front();
    const auto& obj_idx = first_overlap_info.obj_idx;
    const auto& traj_id = *bound->traj_id();
    const auto* st_traj =
        QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    const auto obj_t = st_traj->states()[obj_idx].traj_point->t();
    double threshold = kCutinTimeThreshold;
    if (bound->object_type() == StBoundaryProto::PEDESTRIAN) {
      threshold = kPedestrianCutinTimeThreshold;
    }
    if (obj_t > threshold) {
      to_filtered.insert(bound->id());
    }
  }
  for (auto& bound : boundaries) {
    // Filter out those boundaries that cut in later than threshold.
    if (to_filtered.contains(bound->id())) {
      continue;
    }
    // As well as its potential small angle cutin overlap.
    if (bound->protected_st_boundary_id().has_value()) {
      if (to_filtered.contains(*bound->protected_st_boundary_id())) {
        continue;
      }
    }
    filtered_boundaries.push_back(std::move(bound));
  }

  return filtered_boundaries;
}

// Filter reverse driving st boundaries unless the object is already on path.
std::vector<StBoundaryRef> FilterOncomingStBoundaries(
    absl::Span<StBoundaryRef> boundaries,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DiscretizedPath& av_path) {
  std::vector<StBoundaryRef> filtered_boundaries;
  filtered_boundaries.reserve(boundaries.size());
  absl::flat_hash_set<std::string> to_filtered;
  constexpr double kPedestrianOncomingThreshold =
      0.5;  // Pedestrian ignore threshold for future oncoming.
  constexpr double kOncomingThreshold =
      0.2;  // Ignore threshold for future oncoming.
  constexpr double kOncomingAngleThreshold =
      M_PI / 2.0;  // Ignore threshold for future oncoming.
  for (const auto& bound : boundaries) {
    const auto& overlap_infos = bound->overlap_infos();
    QCHECK(!overlap_infos.empty());
    const auto& first_overlap_info = overlap_infos.front();
    const auto& obj_idx = first_overlap_info.obj_idx;
    const auto& av_idx = first_overlap_info.av_start_idx;
    const auto& traj_id = *bound->traj_id();
    const auto* st_traj =
        QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id));
    const auto& obj_traj_point = *st_traj->states()[obj_idx].traj_point;
    const auto& av_path_point = av_path[av_idx];
    // If oncoming object (heading > 90 degree)
    if (std::abs(
            NormalizeAngle(obj_traj_point.theta() - av_path_point.theta())) >=
        kOncomingAngleThreshold) {
      const auto obj_t = obj_traj_point.t();
      double threshold = kOncomingThreshold;
      if (bound->object_type() == StBoundaryProto::PEDESTRIAN) {
        threshold = kPedestrianOncomingThreshold;
      }
      // If arrival time later than threshold. Ignore it.
      if (obj_t > threshold) {
        to_filtered.insert(bound->id());
      }
    }
  }
  for (auto& bound : boundaries) {
    // Filter out those boundaries that cut in later than threshold.
    if (to_filtered.contains(bound->id())) {
      continue;
    }
    // As well as its potential small angle cutin overlap.
    if (bound->protected_st_boundary_id().has_value()) {
      if (to_filtered.contains(*bound->protected_st_boundary_id())) {
        continue;
      }
    }
    filtered_boundaries.push_back(std::move(bound));
  }

  return filtered_boundaries;
}

void ModifyStationaryObjectStBoundariesForCrowdedScene(
    const std::set<std::string>& crowded_side_objs,
    absl::Span<StBoundaryWithDecision> st_bounds_wd) {
  if (crowded_side_objs.empty()) return;
  constexpr double kEpsilon = 0.01;
  for (auto& st_bound_wd : st_bounds_wd) {
    const auto* raw_st_bound = st_bound_wd.raw_st_boundary();
    if (raw_st_bound->min_s() < kEpsilon) continue;
    if (!IsStBoundaryFromStationaryVehicle(*raw_st_bound)) {
      // Only consider to modify follow distance of stationary vehicles.
    }
    if (raw_st_bound->is_large_vehicle()) {
      continue;
    }
    if (IsStBoundaryFromObjects(*raw_st_bound, crowded_side_objs)) {
      // Do not decrease follow distance if it's the object trying to cut-in.
      continue;
    }
    const auto& original_follow_standstill_distance =
        st_bound_wd.follow_standstill_distance();
    auto new_follow_distance = original_follow_standstill_distance -
                               kCrowdedSceneFollowDistanceDecrease;
    const auto follow_s =
        std::max(kEpsilon, raw_st_bound->min_s() - new_follow_distance);
    new_follow_distance = raw_st_bound->min_s() - follow_s;
    st_bound_wd.set_follow_standstill_distance(new_follow_distance);
    const std::string info =
        absl::StrCat("%s. Modify follow distance: (CS), %.3f. ",
                     st_bound_wd.decision_info(), new_follow_distance);
    st_bound_wd.set_decision_info(info);
    VLOG(4) << absl::StrCat("Decrease follow distance for object ",
                            raw_st_bound->traj_id().value_or(""), info);
    QEVENT_EVERY_N_SECONDS("changqing", "acc_crowded_scene_avoid_cutin", 300.0,
                           [&](QEvent* qevent) {
                             qevent->AddField("crowded_side_objs num",
                                              crowded_side_objs.size());
                           });
  }
}

void PostProcessingReverseStBoundaries(absl::Span<StBoundaryRef> boundaries) {
  for (auto& bound : boundaries) {
    auto upper_points = bound->upper_points();
    auto lower_points = bound->lower_points();
    double first_overlap_s = lower_points.front().s();
    double allowed_ds =
        -std::max(first_overlap_s * kMaxOncomingRatio, kMinOncomingS);
    std::vector<std::pair<StPoint, StPoint>> point_pairs;
    point_pairs.reserve(lower_points.size());
    bool st_boundary_modified = false;
    // Limit lowest s to avoid strong brake of oncoming objects.
    const double allowed_s = first_overlap_s + allowed_ds;
    for (int i = 0; i < lower_points.size(); ++i) {
      auto& lower_pt = lower_points[i];
      auto& upper_pt = upper_points[i];
      if (lower_pt.s() <= allowed_s) {
        // If the oncoming st boundary surpasses allowed s, modify it.
        const double upper_lower_diff = upper_pt.s() - lower_pt.s();
        lower_pt.set_s(allowed_s);
        upper_pt.set_s(lower_pt.s() + upper_lower_diff);
        st_boundary_modified = true;
      }
      point_pairs.emplace_back(lower_pt, upper_pt);
    }
    if (st_boundary_modified) {
      bound->Init(point_pairs);
    }
  }
}

inline std::optional<std::string> GetNearestMovingStBoundaryObjectId(
    absl::Span<const StBoundaryRef> st_bounds) {
  double min_first_overlap_s = std::numeric_limits<double>::max();
  std::optional<std::string> nearest_moving_id = std::nullopt;
  for (const auto& st_bound : st_bounds) {
    const auto& lower_points = st_bound->lower_points();
    const double first_overlap_s = lower_points.front().s();
    if (first_overlap_s < min_first_overlap_s && !st_bound->is_stationary()) {
      // Closer && not stationary, update.
      min_first_overlap_s = first_overlap_s;
      nearest_moving_id = st_bound->object_id();
    }
  }
  return nearest_moving_id;
}

// Filter VEHICLE/UNKNOWN_MOVABLE/LARGE_VEHICLE st boundary generated by
// prediction safe guard traj if it's not the closest moving st bound.
std::vector<StBoundaryRef>
FilterStBoundariesBySafeGuardPredTrajIfNotNearestMoving(
    absl::Span<StBoundaryRef> st_bounds,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::string& nearest_moving_object_id) {
  absl::flat_hash_set<std::string> to_filter_ids;
  for (const auto& st_bound : st_bounds) {
    if (nearest_moving_object_id == *st_bound->object_id()) continue;
    const auto& traj_id_opt = st_bound->traj_id();
    if (!traj_id_opt.has_value()) continue;
    if (st_bound->object_type() != StBoundaryProto::UNKNOWN_OBJECT &&
        st_bound->object_type() != StBoundaryProto::VEHICLE) {
      // Only filter speed-considered vehicle and unknown object type.
      continue;
    }
    const auto* st_traj =
        QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id_opt));
    if (st_traj->trajectory().is_hard_braking()) {
      to_filter_ids.insert(st_bound->id());
      VLOG(1) << "filter " << st_bound->id()
              << " because of not nearest safe guard.";
    }
  }
  std::vector<StBoundaryRef> filtered_bounds;
  for (auto& bound : st_bounds) {
    if (to_filter_ids.contains(bound->id())) {
      continue;
    }
    if (bound->protected_st_boundary_id().has_value()) {
      // Also filter out its related protective st bound if it has any.
      if (to_filter_ids.contains(*bound->protected_st_boundary_id())) {
        continue;
      }
    }
    filtered_bounds.push_back(std::move(bound));
  }
  return filtered_bounds;
}

}  // namespace internal
absl::StatusOr<AccSpeedDecisionOutput> MakeAccSpeedDecision(
    const AccSpeedDecisionInput& input) {
  SCOPED_QTRACE("MakeAccSpeedDecision");

  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;

  /*
   *  Step 1: Map st boundaries of objects onto st graph.
   *
   */
  auto start_time = absl::Now();
  const auto av_shapes = BuildAvShapes(vehicle_geometry_params, *input.path);
  std::optional<PathApprox> path_approx_for_mirrors;
  PathApprox* path_approx_for_mirrors_ptr = nullptr;
  if (FLAGS_planner_use_path_approx_based_st_mapping) {
    path_approx_for_mirrors =
        BuildPathApproxForMirrors(*input.path_approx, vehicle_geometry_params);
    path_approx_for_mirrors_ptr = path_approx_for_mirrors.has_value()
                                      ? &(*path_approx_for_mirrors)
                                      : nullptr;
  }
  const auto& motion_constraint_params = *input.motion_constraint_params;
  const auto& speed_finder_params = *input.speed_finder_params;
  auto st_graph = std::make_unique<StGraph>(
      input.path, input.trajectory_steps, input.plan_start_v,
      motion_constraint_params.max_deceleration(), &vehicle_geometry_params,
      &speed_finder_params.st_graph_params(), &av_shapes, input.path_kd_tree,
      input.path_approx, path_approx_for_mirrors_ptr);
  ConstraintManager constraint_manager;

  auto st_boundaries = st_graph->GetStBoundaries(
      *input.traj_mgr, /*leading_objs=*/{}, constraint_manager,
      /*psman_mgr=*/nullptr, /*drive_passage=*/nullptr,
      /*path_sl_boundary=*/nullptr, /*thread_pool=*/nullptr);

  /*
   *  Step 2: Filter and process st boundaries of objects.
   *
   */
  // Filter out st boundaries by predicted cut-in time.
  st_boundaries = internal::FilterStBoundariesByCutinTime(
      absl::MakeSpan(st_boundaries), *input.traj_mgr);
  // Filter out oncoming st boundaries which are not already on path.
  st_boundaries = internal::FilterOncomingStBoundaries(
      absl::MakeSpan(st_boundaries), *input.traj_mgr, *input.path);
  // Filter hard braking (pred safe guard traj generated st bounds if it's not
  // the nearest moving st bound).
  const auto nearest_moving_id_opt =
      internal::GetNearestMovingStBoundaryObjectId(st_boundaries);
  if (nearest_moving_id_opt.has_value()) {
    st_boundaries =
        internal::FilterStBoundariesBySafeGuardPredTrajIfNotNearestMoving(
            absl::MakeSpan(st_boundaries), *input.traj_mgr,
            *nearest_moving_id_opt);
  }

  // Limit oncoming overlap range.
  internal::PostProcessingReverseStBoundaries(absl::MakeSpan(st_boundaries));

  VLOG(2) << "Build st_graph cost time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(3) << "st_boundary size = " << st_boundaries.size();
  internal::AnalyzeStOverlapsForACC(*input.path, *input.traj_mgr,
                                    vehicle_geometry_params, &st_boundaries);

  /*
   *  Step 3: Pre-decision.
   *
   */
  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));
  // Set follow/lead standstill distance for st-boundaries.
  absl::flat_hash_set<std::string> stalled_objects = {};
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = &stalled_objects,
      .planner_semantic_map_manager = nullptr,
      .lane_path = nullptr,
      .st_traj_mgr = input.traj_mgr,
      .constraint_mgr = &constraint_manager,
      .extra_follow_standstill_for_large_vehicle =
          PiecewiseLinearFunctionFromProto(
              speed_finder_params
                  .extra_follow_standstill_distance_for_large_vehicle_plf())(
              input.plan_start_v)};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision);
  }

  // Crowded scene modify nearest st boundary (from object).
  internal::ModifyStationaryObjectStBoundariesForCrowdedScene(
      *input.crowded_side_objs, absl::MakeSpan(st_boundaries_with_decision));

  // Only keep all zero-distance stationary st-boudnaries & the nearest
  // non-zero-distance stationary st-boundary.
  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);

  // Construct a simple speed limit provider. We only have curvature limit.
  std::map<SpeedLimitTypeProto::Type, SpeedLimit> static_speed_limit_map;
  auto acc_speed_limit = *input.speed_limit;  // Curvature speed limit.
  static_speed_limit_map.emplace(SpeedLimitTypeProto::CURVATURE,
                                 acc_speed_limit);
  static_speed_limit_map.emplace(SpeedLimitTypeProto::COMBINATION,
                                 std::move(acc_speed_limit));
  SpeedLimitProvider speed_limit_provider(
      std::move(static_speed_limit_map), /*dynamic_speed_limit=*/{},
      /*vt_speed_limit_map=*/{}, kSpeedLimitProviderTimeStep);

  // Set pass/yield time buffers for st-boundaries.
  {
    SCOPED_QTRACE("TimeBufferDecider");
    absl::flat_hash_set<std::string> disable_pass_time_buffer_id_set;
    for (const auto& st_boundary_wd : st_boundaries_with_decision) {
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

    for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
      const bool disable_pass_time_buffer = ContainsKey(
          disable_pass_time_buffer_id_set, st_boundary_with_decision.id());
      const auto* st_boundary = st_boundary_with_decision.raw_st_boundary();
      if (st_boundary->object_type() == StBoundaryProto::VEHICLE ||
          st_boundary->object_type() == StBoundaryProto::CYCLIST ||
          st_boundary->object_type() == StBoundaryProto::PEDESTRIAN) {
        DecideTimeBuffersForStBoundary(
            &st_boundary_with_decision, input.plan_start_v,
            vehicle_geometry_params, *input.traj_mgr, disable_pass_time_buffer);
      }
    }
  }

  if (VLOG_IS_ON(4)) {
    for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
      VLOG(4) << "Set st-boundary " << st_boundary_with_decision.id()
              << " pass_time: " << st_boundary_with_decision.pass_time()
              << " yield_time: " << st_boundary_with_decision.yield_time();
    }
  }
  /*
   *  Step 4: Coarse speed planning given st-graph and speed limit, and make
   *          decisions on st-boundaries at the same time.
   */
  start_time = absl::Now();
  SpeedVector sampling_dp_preliminary_speed;
  InteractiveSpeedDebugProto interactive_speed_debug;
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      processed_st_objects;
  RETURN_IF_ERROR(MakeInteractiveSpeedDecision(
      input.base_name, vehicle_geometry_params, motion_constraint_params,
      *st_graph, *input.traj_mgr, *input.path, input.plan_start_v,
      input.plan_start_a, speed_finder_params, input.user_speed_limit,
      input.trajectory_steps, &speed_limit_provider,
      &sampling_dp_preliminary_speed, &st_boundaries_with_decision,
      &processed_st_objects, &interactive_speed_debug));

  DestroyContainerAsyncMarkSource(std::move(st_graph), (QCRAFT_LOC).ToString());
  return AccSpeedDecisionOutput{
      .speed_limit_provider = std::move(speed_limit_provider),
      .st_boundaries_with_decision = std::move(st_boundaries_with_decision),
      .interactive_speed_debug = std::move(interactive_speed_debug),
      .sampling_dp_preliminary_speed = sampling_dp_preliminary_speed,
  };
}
}  // namespace planner
}  // namespace qcraft
