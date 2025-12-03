#include "onboard/planner/initializer/astar_motion_searcher.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "boost/intrusive/link_mode.hpp"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/astar_motion_searcher_util.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_graph_cache.h"
#include "onboard/planner/initializer/motion_search_util.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/multi_traj_selector.h"
#include "onboard/planner/initializer/proto/initializer_config.pb.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

using LeadingTrajs = std::vector<std::string>;

std::vector<LeadingTrajs> BuildLeadingConfigs(
    const std::vector<LeadingGroup>& leading_groups,
    const ConstraintProto::LeadingObjectProto* blocking_static_traj) {
  SCOPED_QTRACE("BuildLeadingConfigs");

  // Construct leading config according to leading groups.
  std::vector<LeadingTrajs> leading_configs;
  leading_configs.reserve(leading_groups.size());
  for (const auto& leading_group : leading_groups) {
    auto& leading_trajs = leading_configs.emplace_back();
    for (const auto& [traj_id, _] : leading_group) {
      leading_trajs.push_back(traj_id);
    }
  }

  if (blocking_static_traj != nullptr) {
    // Add non-stalled blocking static trajectory to all groups.
    for (auto& leading_config : leading_configs) {
      leading_config.push_back(blocking_static_traj->traj_id());
    }
  }

  return leading_configs;
}

std::vector<ApolloTrajectoryPointProto> BacktrackFinalTraj(
    const int traj_steps, const AstarNode& end_node, CloseMap* close_map) {
  std::vector<const MotionForm*> motions;
  auto current_node = end_node;
  while (current_node.motion != nullptr) {
    motions.emplace_back(current_node.motion.get());
    current_node = *((*close_map)[current_node.parent]);
  }
  std::reverse(motions.begin(), motions.end());
  return ResampleTrajectoryPoints(traj_steps, motions);
}

absl::StatusOr<double> FilterAndGetBoxMaxS(const DrivePassage& passage,
                                           const Box2d& box) {
  ASSIGN_OR_RETURN(
      const auto frenet_box, passage.QueryFrenetBoxAt(box),
      absl::NotFoundError("Cannot project the object box onto drive passage."));
  ASSIGN_OR_RETURN(
      const auto passage_tangent,
      passage.QueryTangentAtS(frenet_box.center_s()),
      absl::NotFoundError("Cannot find passage tangent near the object box."));

  if (std::abs(NormalizeAngle(passage_tangent.FastAngle() - box.heading())) >
      M_PI_2) {
    return absl::NotFoundError("Oncoming object ignored for leading object.");
  }

  return frenet_box.s_max;
}

double GetOvertakeLeadingTrajBehindMinS(
    double traj_horizon, const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::vector<std::string>>& leading_groups,
    const int leading_group_idx) {
  // Get max s of leading trajectories behind, also min s to overtake leading
  // trajectories behind for ego
  double traj_behind_max_s = std::numeric_limits<double>::lowest();
  for (int i = 0; i < leading_group_idx; ++i) {
    for (const auto& traj_id : leading_groups[i]) {
      const auto states =
          QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(traj_id))->states();
      const int traj_end_idx =
          std::min(static_cast<int>(states.size()),
                   CeilToInt(traj_horizon / prediction::kPredictionTimeStep)) -
          1;
      ASSIGN_OR_CONTINUE(
          const auto box_max_s,
          FilterAndGetBoxMaxS(drive_passage, states[traj_end_idx].box));
      traj_behind_max_s = std::max(traj_behind_max_s, box_max_s);
    }
  }
  return traj_behind_max_s;
}

absl::StatusOr<SingleTrajInfo> AstarSearchForSingleTrajectory(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl,
    const ApolloTrajectoryPointProto& start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<LeadingTrajs>& leading_groups,
    const int leading_group_idx, const InitializerConfig& initializer_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom,
    const GeometryGraph& geom_graph, const std::vector<Vec2d>& ref_line_points,
    const CollisionChecker& collision_checker,
    const ml::captain_net::CaptainNetOutput& captain_net_output,
    const std::vector<double>& stop_s_on_drive_passage, bool is_lane_change,
    const bool is_run_model_l4, AstarGraphCache* cost_cache,
    ThreadPool* thread_pool) {
  FUNC_QTRACE();

  VLOG(1) << "--------- Start of Astar Search-----------";

  const int traj_steps = initializer_params.traj_steps();
  const double traj_horizon = (traj_steps - 1) * kTrajectoryTimeStep;

  const double geom_graph_max_s = geom_graph.GetMaxAccumulatedS();
  const double speed_max_s =
      std::max(start_point.v(), kMinSpeedForFinalCost) * traj_horizon;

  const auto start_time = absl::Now();
  SingleTrajInfo traj_output;
  traj_output.leading_trajs = leading_groups[leading_group_idx];
  traj_output.ref_speed_table = std::make_unique<RefSpeedTable>(
      st_traj_mgr, leading_groups[leading_group_idx], drive_passage,
      stop_s_on_drive_passage);

  const double leading_obj_min_s = GetLeadingObjectsEndMinS(
      st_traj_mgr, drive_passage, leading_groups[leading_group_idx],
      vehicle_geom.front_edge_to_center());
  const double max_accumulated_s =
      Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);

  bool multi_search = true;
  if (leading_groups.size() == 1) multi_search = false;

  // Check if the ego is able to overtake leading groups behind
  bool able_to_overtake_leading_behind = false;
  if (multi_search) {
    const double min_followed_distance = 3.0;
    const double min_s_to_overtake =
        GetOvertakeLeadingTrajBehindMinS(traj_horizon, drive_passage,
                                         st_traj_mgr, leading_groups,
                                         leading_group_idx) +
        vehicle_geom.back_edge_to_center() + min_followed_distance;
    able_to_overtake_leading_behind =
        std::min(leading_obj_min_s, geom_graph_max_s) > min_s_to_overtake;
  }

  traj_output.cost_provider = static_cast<std::unique_ptr<CostProviderBase>>(
      std::make_unique<AstarCostProvider>(
          drive_passage, initializer_params, motion_constraint_params,
          stop_s_on_drive_passage, st_traj_mgr, leading_groups,
          leading_group_idx, able_to_overtake_leading_behind, vehicle_geom,
          &collision_checker, &path_sl, traj_output.ref_speed_table.get(),
          &captain_net_output, is_lane_change, max_accumulated_s));

  OpenPQ open_pq;
  OpenMap open_map;
  CloseMap close_map;

  // Prepare start node
  const auto& nodes_layers = geom_graph.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  MotionState sdc_motion = PrepareStartMotionNode(
      geom_graph, nodes_layers[0], start_point, &start_node_idx_on_first_layer);

  const int start_node_key =
      CreateNodeKey(sdc_motion.v, sdc_motion.t,
                    nodes_layers[0][start_node_idx_on_first_layer]);
  const auto start_node =
      CreateNode(sdc_motion, start_node_key,
                 nodes_layers[0][start_node_idx_on_first_layer]);
  const auto start_node_ptr = std::make_shared<AstarNode>(start_node);
  open_map.emplace(start_node_key, open_pq.emplace(start_node_ptr));

  bool search_success = false;
  AstarNode end_node;
  while (!open_pq.empty()) {
    const auto current_node = open_pq.top();
    open_pq.pop();
    open_map.erase(current_node->key);

    if (SatisfyEndCondition(traj_horizon, *current_node)) {
      search_success = true;
      end_node = *current_node;
      close_map.emplace(current_node->key, current_node);
      break;
    }

    if (current_node->state.t > traj_horizon) {
      // Not expand node having reached planning horizon
      close_map.emplace(current_node->key, current_node);
      continue;
    }

    bool sample_const_v = false;
    if (current_node->layer >= kConstVelSampleLayerSizeThreshold - 1) {
      // If current layer >= kConstVelSampleLayerSizeThreshold (currently
      // set at 5), only sample acc = 0 and a_min, a_max, a_stop motions.
      sample_const_v = true;
    }

    const auto& outgoing_edge_idxs =
        geom_graph.GetOutgoingEdges(current_node->geom_index);

    std::vector<std::vector<AstarCacheInfo>> new_caches_container;
    new_caches_container.resize(outgoing_edge_idxs.size());

    // Get and process candidate nodes, paralled
    absl::Mutex my_mutex;
    ParallelFor(0, outgoing_edge_idxs.size(), thread_pool, [&](int i) {
      const auto& geom_edge = geom_graph.GetEdge(outgoing_edge_idxs[i]);
      if (!geom_graph.IsActive(outgoing_edge_idxs[i])) {
        return;
      }
      const auto acc_samples =
          SampleMotionForms(traj_horizon, *current_node, geom_edge,
                            motion_constraint_params, sample_const_v);

      if (multi_search) {
        ProcessMotionFormsWithCache(
            traj_horizon, *current_node, geom_edge, acc_samples,
            *traj_output.cost_provider, ref_line_points,
            traj_output.ref_speed_table.get(), captain_net_output.traj_points,
            &open_pq, &open_map, close_map, *cost_cache,
            &new_caches_container[i], is_run_model_l4, &my_mutex);
      } else {
        ProcessMotionFormsWithoutCache(
            traj_horizon, *current_node, geom_edge, acc_samples,
            *traj_output.cost_provider, ref_line_points,
            traj_output.ref_speed_table.get(), captain_net_output.traj_points,
            &open_pq, &open_map, close_map, is_run_model_l4, &my_mutex);
      }
    });

    if (multi_search) {
      for (auto it = std::make_move_iterator(new_caches_container.begin());
           it != std::make_move_iterator(new_caches_container.end()); ++it) {
        cost_cache->BatchInsert(*it);
      }
    }

    close_map.emplace(current_node->key, current_node);
  }

  if (!search_success) {
    // Create a stationary motion.
    if (sdc_motion.v < kSearchFailedCanSetToZeroSpeed) {
      end_node = SearchFailTryStationaryMotion(
          traj_horizon, *traj_output.cost_provider, start_node, ref_line_points,
          traj_output.ref_speed_table.get(), captain_net_output.traj_points,
          is_run_model_l4);
      close_map.emplace(end_node.key, std::make_shared<AstarNode>(end_node));
    } else {
      return absl::NotFoundError("No trajectories found.");
    }
  }

  auto traj_points = BacktrackFinalTraj(traj_steps, end_node, &close_map);

  const auto& front_pt = traj_points.front();
  const auto& back_pt = traj_points.back();
  if (front_pt.v() < kCanSetToZeroSpeed && back_pt.v() < kCanSetToZeroSpeed &&
      back_pt.path_point().s() - front_pt.path_point().s() <
          kCanSetToZeroTrajLength) {
    traj_output.traj_points = ConstructStationaryTraj(traj_steps, sdc_motion);
  } else {
    traj_output.traj_points = std::move(traj_points);
  }

  traj_output.total_cost = end_node.cost_g;
  VLOG(1) << " Total cost: " << traj_output.total_cost;

  // Return ignorable trajectories.
  traj_output.ignored_trajs = end_node.ignored_trajs;

  traj_output.astar_end_node_index = end_node.key;

  if (FLAGS_planner_initializer_debug_level >= 1) {
    traj_output.debug_info.astar_open_pq = std::move(open_pq);
    traj_output.debug_info.astar_close_map = std::move(close_map);
  }

  VLOG(1) << "Astar motion search for Time spent: " << absl::Now() - start_time;
  VLOG(1) << "--------- End of Astar Search-------------";

  return traj_output;
}

}  // namespace

absl::StatusOr<MotionSearchOutput> AstarSearchForRawTrajectory(
    const MotionSearchInput& input, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  const DrivePassage& drive_passage = *input.drive_passage;
  const ApolloTrajectoryPointProto& start_point = *input.start_point;
  const SpacetimeTrajectoryManager& st_traj_mgr = *input.st_traj_mgr;
  const VehicleGeometryParamsProto& vehicle_geom = *input.vehicle_geom;
  const GeometryGraph& geom_graph = *input.geom_graph;

  const auto leading_traj_configs =
      BuildLeadingConfigs(*input.leading_groups, input.blocking_static_traj);
  if (leading_traj_configs.empty()) {
    return absl::NotFoundError("No leading group generated.");
  }

  // Prepare motion graph cache.
  AstarGraphCache cost_cache;

  std::vector<SingleTrajInfo> multi_trajs;
  multi_trajs.reserve(leading_traj_configs.size());

  for (int i = 0; i < leading_traj_configs.size(); ++i) {
    auto single_traj_result = AstarSearchForSingleTrajectory(
        drive_passage, *input.sl_boundary, start_point, st_traj_mgr,
        leading_traj_configs, i, *input.initializer_params,
        *input.motion_constraint_params, vehicle_geom, geom_graph,
        *input.reference_line_points, *input.collision_checker,
        *input.captain_net_output, *input.stop_s_vec, input.is_lane_change,
        input.is_run_model_l4, &cost_cache, thread_pool);
    if (single_traj_result.ok()) {
      multi_trajs.push_back(std::move(single_traj_result).value());
    }
  }

  if (multi_trajs.empty()) {
    return absl::NotFoundError("No Astar end node found.");
  }

  // Need to determine the final output.
  absl::flat_hash_set<std::string> follower_set;
  double follower_max_decel = 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  LaneChangeSafetyDebugProto lane_change_safety_debug_proto;
  const auto choice_or = EvaluateMultiTrajs(
      *input.planner_semantic_map_manager, drive_passage, st_traj_mgr,
      multi_trajs, vehicle_geom, input.eval_safety, input.lc_style,
      input.path_look_ahead_duration, &follower_set, &follower_max_decel,
      &unsafe_object_ids, &lane_change_safety_debug_proto, thread_pool);

  // Add all candidates to motion search output result, with the first one
  // always the selected.
  MotionSearchOutput output;
  output.multi_traj_candidates.reserve(multi_trajs.size());
  for (const auto& traj : multi_trajs) {
    MotionSearchOutput::MultiTrajCandidate traj_candidate;
    traj_candidate.leading_traj_ids = traj.leading_trajs;
    traj_candidate.trajectory = traj.traj_points;
    traj_candidate.total_cost = traj.total_cost;
    traj_candidate.ignored_trajs = traj.ignored_trajs;
    output.multi_traj_candidates.push_back(std::move(traj_candidate));
  }

  output.lane_change_safety_debug_proto =
      std::move(lane_change_safety_debug_proto);

  if (!choice_or.ok()) {
    QLOG(WARNING) << "No safe trajectory found for initializer, pausing "
                     "lane change: "
                  << choice_or.status().message();
    output.unsafe_object_ids = std::move(unsafe_object_ids);

    // No safe trajectory found, return a indicator to fake output outside.
    output.is_lc_pause = true;

    return output;
  }

  const int choice = *choice_or;
  if (choice != 0) {
    std::swap(output.multi_traj_candidates[0],
              output.multi_traj_candidates[choice]);
  }
  output.follower_set = std::move(follower_set);
  output.follower_max_decel = follower_max_decel;
  for (const auto& traj_id : multi_trajs[choice].leading_trajs) {
    for (const auto& leading_group : *input.leading_groups) {
      const auto it = leading_group.find(traj_id);
      if (it != leading_group.end()) {
        output.leading_trajs.emplace(traj_id, it->second);
        break;
      }
    }
  }
  output.traj_points = std::move(multi_trajs[choice].traj_points);
  output.min_cost = multi_trajs[choice].total_cost;
  output.motion_graph = nullptr;
  output.ref_speed_table = std::move(multi_trajs[choice].ref_speed_table);
  output.cost_provider = std::move(multi_trajs[choice].cost_provider);
  output.astar_end_node_index = multi_trajs[choice].astar_end_node_index;

  if (FLAGS_planner_initializer_debug_level >= 1) {
    auto& debug_info = multi_trajs[choice].debug_info;
    output.astar_open_pq = std::move(debug_info.astar_open_pq);
    output.astar_close_map = std::move(debug_info.astar_close_map);
  }

  output.search_algorithm = MotionSearchOutput::SearchAlgorithm::kASTAR;

  return output;
}

}  // namespace qcraft::planner
