#include "onboard/planner/initializer/dp_motion_searcher.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <iterator>
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
#include "absl/strings/str_join.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/dp_motion_searcher_util.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
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
#include "onboard/planner/ml/initializer_models/initializer_feature_extractor.h"
#include "onboard/planner/ml/initializer_models/initializer_post_evaluator.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
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

void InsertMotionNodes(
    double traj_horizon, const std::vector<GeometryNodeIndex>& nodes_layer,
    const MotionNodeIndex& sdc_node_idx,
    const GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>&
        opt_motion_samples,
    MotionGraph* motion_graph,
    std::vector<MotionEdgeIndex>* terminated_edge_idxes,
    GeometryNodeVector<std::vector<MotionEdgeIndex>>* motions_to_expand,
    MotionEdgeVector<MotionSearchOutput::SearchCost>* search_costs,
    MotionEdgeVector<IgnoreTrajMap>* ignored_trajs_vec) {
  SCOPED_QTRACE("InsertMotionNode");
  for (int i = 0, n = nodes_layer.size(); i < n; ++i) {
    const auto geom_node_idx = nodes_layer[i];
    // best motion so far of a given geometry node.
    const auto& best_motions = opt_motion_samples[geom_node_idx];
    for (const auto& [_, dp_motion_info] : best_motions) {
      MotionState end_motion_state =
          dp_motion_info.motion_form->GetEndMotionState();
      MotionEdgeIndex motion_edge_index;
      const auto prev_motion_edge_index = dp_motion_info.prev_motion_edge_index;
      if (prev_motion_edge_index ==
          MotionEdgeVector<MotionEdge>::kInvalidIndex) {
        // This edge has no previous edge since it connects directly to
        // the start motion node.
        const auto end_node_index =
            motion_graph->AddMotionNode(end_motion_state, geom_node_idx);
        motion_edge_index = motion_graph->AddMotionEdge(
            sdc_node_idx, end_node_index, dp_motion_info.motion_form,
            geom_node_idx, MotionEdgeVector<MotionEdge>::kInvalidIndex);

        search_costs->push_back(MotionSearchOutput::SearchCost{
            .feature_cost = dp_motion_info.costs,
            .cost_to_come = dp_motion_info.sum_cost});
        ignored_trajs_vec->push_back(dp_motion_info.ignored_trajs);
      } else {
        // This edge has previous edge.
        const auto& prev_motion_edge =
            motion_graph->GetMotionEdge(prev_motion_edge_index);
        end_motion_state.t +=
            motion_graph->GetMotionNode(prev_motion_edge.end).state.t;
        const auto end_node_index =
            motion_graph->AddMotionNode(end_motion_state, geom_node_idx);
        motion_edge_index = motion_graph->AddMotionEdge(
            prev_motion_edge.end, end_node_index, dp_motion_info.motion_form,
            geom_node_idx, prev_motion_edge_index);

        const auto& prev_cost = (*search_costs)[prev_motion_edge_index];
        search_costs->push_back(MotionSearchOutput::SearchCost{
            .feature_cost =
                AddCost(dp_motion_info.costs, prev_cost.feature_cost),
            .cost_to_come = dp_motion_info.sum_cost + prev_cost.cost_to_come});
        ignored_trajs_vec->push_back(dp_motion_info.ignored_trajs);
      }
      // Discard super slow state if it fails to reach the end of time
      // horizon.
      if (end_motion_state.v < 0.1 && end_motion_state.t < traj_horizon) {
        continue;
      }
      // We let the motion expansion stop if it reaches the end of time
      // horizon.
      if (end_motion_state.t >= traj_horizon) {
        terminated_edge_idxes->push_back(motion_edge_index);
      } else {
        // Otherwise we add it to the to-be-expanded motions.
        (*motions_to_expand)[geom_node_idx].push_back(motion_edge_index);
      }
    }
  }
}

int UpdateOptimalMotions(
    double traj_horizon,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    const MotionGraphCache& cost_cache,
    std::vector<std::vector<DpMotionInfo>> candidate_motions,
    GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>*
        opt_motion_samples) {
  int motion_count = 0;
  for (auto& candidate_motions_per_node : candidate_motions) {
    motion_count += candidate_motions_per_node.size();

    for (auto& candidate_motion : candidate_motions_per_node) {
      const auto end_geometry_node_index =
          candidate_motion.end_geometry_node_index;
      // best_motion_so_far is motion terminating at this geometry node (at
      // the velocity and time determined by this motion).
      auto& best_motion_so_far = (*opt_motion_samples)[end_geometry_node_index];
      ASSIGN_OR_CONTINUE(candidate_motion.motion_form,
                         cost_cache.GetMotionForm(candidate_motion.key));
      const auto end_state = candidate_motion.motion_form->GetEndMotionState();
      const DpMotionSample dp_motion_sample(end_state.v, end_state.t,
                                            traj_horizon);
      if (best_motion_so_far.find(dp_motion_sample) ==
          best_motion_so_far.end()) {
        // No motion is added yet to the hashmap, add it.
        // Only record some optimal edges costs to the cache.
        best_motion_so_far[dp_motion_sample] = std::move(candidate_motion);
      } else {
        // A motion is already in the hashmap, compare its cost with the
        // currently evaluating one and keep better.
        const auto& prev_best = best_motion_so_far[dp_motion_sample];
        auto prev_cost = prev_best.sum_cost;
        if (prev_best.prev_motion_edge_index !=
            MotionEdgeVector<MotionEdge>::kInvalidIndex) {
          prev_cost +=
              search_costs[prev_best.prev_motion_edge_index].cost_to_come;
        }
        auto cur_cost = candidate_motion.sum_cost;
        if (candidate_motion.prev_motion_edge_index !=
            MotionEdgeVector<MotionEdge>::kInvalidIndex) {
          cur_cost += search_costs[candidate_motion.prev_motion_edge_index]
                          .cost_to_come;
        }
        if (cur_cost < prev_cost) {
          best_motion_so_far[dp_motion_sample] = std::move(candidate_motion);
        }
      }
    }
  }
  return motion_count;
}

void FillTrajDebugInfo(
    int traj_steps, const MotionGraph& motion_graph,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    std::vector<MotionEdgeIndex> terminated_edge_idxes,
    SingleTrajDebugInfo* debug_info) {
  constexpr int kMaxTopTrajectoryNum = 50;
  const int k_top_traj_num = std::min(
      static_cast<int>(terminated_edge_idxes.size()), kMaxTopTrajectoryNum);
  const auto top_k_traj_info =
      TopKTrajectories(terminated_edge_idxes, search_costs, k_top_traj_num);

  auto& top_k_trajs = debug_info->top_k_trajs;
  top_k_trajs.reserve(top_k_traj_info.size());
  auto& top_k_total_costs = debug_info->top_k_total_costs;
  top_k_total_costs.reserve(top_k_traj_info.size());
  auto& top_k_edges = debug_info->top_k_edges;
  top_k_edges.reserve(top_k_traj_info.size());
  for (const auto& traj_info : top_k_traj_info) {
    top_k_total_costs.push_back(traj_info.total_cost);
    top_k_edges.push_back(traj_info.idx);
    top_k_trajs.push_back(
        ConstructTrajFromLastEdge(traj_steps, motion_graph, traj_info.idx));
  }
  debug_info->terminated_edge_idxes = std::move(terminated_edge_idxes);
}

absl::StatusOr<SingleTrajInfo> DpSearchForSingleTrajectory(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl,
    const ApolloTrajectoryPointProto& start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<LeadingTrajs>& leading_groups,
    const int leading_group_idx, const InitializerConfig& initializer_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom,
    const GeometryGraph& geom_graph, const GeometryFormBuilder& form_builder,
    const CollisionChecker& collision_checker,
    const ml::captain_net::CaptainNetOutput& captain_net_output,
    const std::vector<double>& stop_s_on_drive_passage, bool is_lane_change,
    MotionGraphCache* cost_cache, ThreadPool* thread_pool) {
  FUNC_QTRACE();

  VLOG(1) << "--------- Start of One Search-----------";

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
  traj_output.motion_graph = std::make_unique<XYTMotionGraph>(&geom_graph);

  const double leading_obj_min_s = GetLeadingObjectsEndMinS(
      st_traj_mgr, drive_passage, leading_groups[leading_group_idx],
      vehicle_geom.front_edge_to_center());
  const double max_accumulated_s =
      Min(leading_obj_min_s, geom_graph_max_s, speed_max_s);

  traj_output.cost_provider = static_cast<std::unique_ptr<CostProviderBase>>(
      std::make_unique<DpCostProvider>(
          drive_passage, initializer_params, motion_constraint_params,
          stop_s_on_drive_passage, st_traj_mgr, leading_groups,
          leading_group_idx, vehicle_geom, &collision_checker, &path_sl,
          traj_output.ref_speed_table.get(), &captain_net_output,
          is_lane_change, max_accumulated_s));
  // Keep track on finished motions (reached max trajectory time length).
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  // Keep track on best motions per GeometryNode.
  GeometryNodeVector<absl::flat_hash_map<DpMotionSample, DpMotionInfo>>
      opt_motion_samples;
  opt_motion_samples.resize(geom_graph.nodes().size());
  // Keep track on "expandable" edges (currently best edges) per GeometryNode.
  GeometryNodeVector<std::vector<MotionEdgeIndex>> motions_to_expand;
  motions_to_expand.resize(geom_graph.nodes().size());
  // Keep track on optimal costs.
  MotionEdgeVector<MotionSearchOutput::SearchCost> search_costs;
  MotionEdgeVector<IgnoreTrajMap> ignored_trajs_vector;

  // Prepare start node
  const auto& nodes_layers = geom_graph.nodes_layers();
  int start_node_idx_on_first_layer = 0;
  MotionState sdc_motion = PrepareStartMotionNode(
      geom_graph, nodes_layers[0], start_point, &start_node_idx_on_first_layer);
  const auto sdc_node_idx = traj_output.motion_graph->AddMotionNode(
      sdc_motion, nodes_layers[0][start_node_idx_on_first_layer]);
  // Main loop of DP search once a leading obj is specified.
  int motion_count = 0;
  for (int cur_layer_idx = 0, num_layer = nodes_layers.size();
       cur_layer_idx < num_layer; ++cur_layer_idx) {
    SCOPED_QTRACE("DpSearchOnLayer");

    const auto& nodes_layer = nodes_layers[cur_layer_idx];
    // 1. insert best so far motion nodes & edges of the current layer and
    // pick the motion edges to expand. The block is not executed for the
    // first layer since at the beginning we start to expand from the
    // planning_start_state.
    if (cur_layer_idx != 0) {
      InsertMotionNodes(traj_horizon, nodes_layer, sdc_node_idx,
                        opt_motion_samples, traj_output.motion_graph.get(),
                        &terminated_edge_idxes, &motions_to_expand,
                        &search_costs, &ignored_trajs_vector);
    }

    // Reached the last layer, stop the iteration.
    if (cur_layer_idx == num_layer - 1) break;

    // 2. Expand motions from nodes.
    // candidate_motions records the candidate motions for each node per layer.
    std::vector<std::vector<DpMotionInfo>> candidate_motions;
    candidate_motions.resize(nodes_layer.size());
    std::vector<std::vector<NewCacheInfo>> new_motion_forms_container;
    new_motion_forms_container.resize(nodes_layer.size());

    if (cur_layer_idx == 0) {
      candidate_motions[start_node_idx_on_first_layer] = ExpandStartMotionEdges(
          traj_horizon, nodes_layer[start_node_idx_on_first_layer],
          sdc_node_idx, geom_graph, *(traj_output.motion_graph),
          motion_constraint_params, *traj_output.cost_provider, *cost_cache,
          &new_motion_forms_container[start_node_idx_on_first_layer]);
    } else {
      // This routine is the most computational heavy one, can be paralleled.
      // Create container per node to hold new motion forms temporarily.
      bool sample_const_v = false;
      if (cur_layer_idx >= kConstVelSampleLayerSizeThreshold - 1) {
        // If current layer >= kConstVelSampleLayerSizeThreshold (currently
        // set at 5), only sample acc = 0 and a_min, a_max, a_stop motions.
        sample_const_v = true;
      }
      ParallelFor(0, nodes_layer.size(), thread_pool, [&](int i) {
        // Parallel to calculate candidate motions and their cost for each
        // node on this layer.
        const auto geom_node_idx = nodes_layer[i];
        const auto& motion_edge_idxs = motions_to_expand[geom_node_idx];
        auto& new_motion_forms = new_motion_forms_container[i];
        candidate_motions[i] = ExpandMotionEdges(
            traj_horizon, geom_node_idx, motion_edge_idxs, ignored_trajs_vector,
            geom_graph, *(traj_output.motion_graph), motion_constraint_params,
            *traj_output.cost_provider, sample_const_v, *cost_cache,
            &new_motion_forms);
      });
    }
    for (auto it = std::make_move_iterator(new_motion_forms_container.begin());
         it != std::make_move_iterator(new_motion_forms_container.end());
         ++it) {
      cost_cache->BatchInsert(*it);
    }
    // 3. Update optimal motions associated to each geometry node.
    motion_count +=
        UpdateOptimalMotions(traj_horizon, search_costs, *cost_cache,
                             std::move(candidate_motions), &opt_motion_samples);
  }

  if (FLAGS_planner_initializer_debug_level >= 2) {
    QLOG(INFO) << "Total evaluated motions of DP: " << motion_count;
    QLOG(INFO) << "Collected motion edge cache size: " << cost_cache->size();
  }

  // Find the best edge considering final cost.
  const auto best_edge_info =
      FindBestEdge(traj_horizon, *traj_output.cost_provider, sdc_motion,
                   sdc_node_idx, nodes_layers[0][start_node_idx_on_first_layer],
                   traj_output.motion_graph.get(), cost_cache, &search_costs,
                   &ignored_trajs_vector, &terminated_edge_idxes);
  const auto best_final_edge = best_edge_info.idx;
  if (best_final_edge == MotionEdgeVector<MotionEdge>::kInvalidIndex) {
    return absl::NotFoundError("No trajectories found.");
  }
  traj_output.last_edge_index = best_final_edge;
  traj_output.search_costs = search_costs;
  traj_output.total_cost = best_edge_info.total_cost;
  VLOG(1) << "best_final_edge: " << best_final_edge.value()
          << " Total cost: " << traj_output.total_cost;
  if (FLAGS_planner_initializer_enable_post_evaluation) {
    PostEvaluatorInput post_evaluator_input{
        .form_builder = &form_builder,
        .pre_best = &best_edge_info,
        .drive_passage = &drive_passage,
        .stop_s = &stop_s_on_drive_passage,
        .st_traj_mgr = &st_traj_mgr,
        .leading_groups = &leading_groups,
        .leading_group_idx = leading_group_idx,
        .vehicle_geom = &vehicle_geom,
        .collision_checker = &collision_checker,
        .path_sl = &path_sl,
        .initializer_params = &initializer_params,
        .motion_constraint_params = &motion_constraint_params,
        .ref_speed_table = traj_output.ref_speed_table.get(),
        .captain_net_output = &captain_net_output,
        .is_lane_change = is_lane_change,
        .max_accumulated_s = max_accumulated_s,
        .is_post_evaluation = true,
        .search_costs = &search_costs,
        .terminated_idxes = terminated_edge_idxes,
        .sdc_motion = &sdc_motion,
        .sdc_node_idx = sdc_node_idx,
        .sdc_geom_node = nodes_layers[0][start_node_idx_on_first_layer],
        .motion_graph = traj_output.motion_graph.get(),
    };
    PostEvaluateTrajs(post_evaluator_input, &traj_output, thread_pool);
  }
  // Return ignorable trajectories.
  traj_output.ignored_trajs = ignored_trajs_vector[traj_output.last_edge_index];
  auto traj_points = ConstructTrajFromLastEdge(
      traj_steps, *traj_output.motion_graph, traj_output.last_edge_index);
  const auto& front_pt = traj_points.front();
  const auto& back_pt = traj_points.back();
  if (front_pt.v() < kCanSetToZeroSpeed && back_pt.v() < kCanSetToZeroSpeed &&
      back_pt.path_point().s() - front_pt.path_point().s() <
          kCanSetToZeroTrajLength) {
    traj_output.traj_points = ConstructStationaryTraj(traj_steps, sdc_motion);
  } else {
    traj_output.traj_points = std::move(traj_points);
  }

  // For debug: show candidate trajectories
  if (FLAGS_planner_initializer_debug_level >= 1 ||
      FLAGS_dumping_initializer_features) {
    FillTrajDebugInfo(
        traj_steps, *traj_output.motion_graph, traj_output.search_costs,
        std::move(terminated_edge_idxes), &traj_output.debug_info);
  }

  VLOG(1) << "Dp motion search for Time spent: " << absl::Now() - start_time;
  VLOG(1) << "--------- End of One Search-------------";
  return traj_output;
}

}  // namespace

absl::StatusOr<MotionSearchOutput> DpSearchForRawTrajectory(
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
  MotionGraphCache cost_cache;

  std::vector<SingleTrajInfo> multi_trajs;
  multi_trajs.reserve(leading_traj_configs.size());

  // TODO(lidong): Change this loop into a parallel for loop.
  for (int i = 0; i < leading_traj_configs.size(); ++i) {
    auto single_traj_result = DpSearchForSingleTrajectory(
        drive_passage, *input.sl_boundary, start_point, st_traj_mgr,
        leading_traj_configs, i, *input.initializer_params,
        *input.motion_constraint_params, vehicle_geom, geom_graph,
        *input.form_builder, *input.collision_checker,
        *input.captain_net_output, *input.stop_s_vec, input.is_lane_change,
        &cost_cache, thread_pool);
    if (single_traj_result.ok()) {
      multi_trajs.push_back(std::move(single_traj_result).value());
    }
  }

  if (multi_trajs.empty()) {
    return absl::NotFoundError("No terminating motion edge found.");
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
    if (traj.last_edge_index.value() < traj.search_costs.size()) {
      traj_candidate.feature_costs =
          traj.search_costs[traj.last_edge_index].feature_cost;
    }
    traj_candidate.last_edge_index = traj.last_edge_index;
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
  output.search_costs = std::move(multi_trajs[choice].search_costs);
  output.best_last_edge_index = multi_trajs[choice].last_edge_index;
  output.min_cost = multi_trajs[choice].total_cost;
  output.motion_graph = std::move(multi_trajs[choice].motion_graph);
  output.ref_speed_table = std::move(multi_trajs[choice].ref_speed_table);
  output.cost_provider = std::move(multi_trajs[choice].cost_provider);
  VLOG(1) << absl::StrJoin(
      output.search_costs[output.best_last_edge_index].feature_cost, ", ");

  if (FLAGS_planner_initializer_debug_level >= 1 ||
      FLAGS_dumping_initializer_features) {
    auto& debug_info = multi_trajs[choice].debug_info;
    output.terminated_edge_idxes = std::move(debug_info.terminated_edge_idxes);
    output.top_k_trajs = std::move(debug_info.top_k_trajs);
    output.top_k_total_costs = std::move(debug_info.top_k_total_costs);
    output.top_k_edges = std::move(debug_info.top_k_edges);
  }

  // For data dumping only.
  if (FLAGS_dumping_initializer_features) {
    RETURN_IF_ERROR(ExpertDpMotionEvaluation(
        input.initializer_params->traj_steps(), input.plan_time,
        *input.form_builder, *input.log_av_trajectory, &output));
    SampledDpMotionEvaluation(input.initializer_params->traj_steps(),
                              output.search_costs, output.terminated_edge_idxes,
                              &output);
  }

  output.search_algorithm = MotionSearchOutput::SearchAlgorithm::kDP;

  return output;
}

}  // namespace qcraft::planner
