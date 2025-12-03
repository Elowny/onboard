#include "onboard/planner/initializer/dp_motion_searcher_util.h"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_graph_cache.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

std::vector<double> AddCost(absl::Span<const double> vec1,
                            absl::Span<const double> vec2) {
  const int vec1_size = vec1.size();
  std::vector<double> res(vec1_size);
  QCHECK_EQ(vec1_size, vec2.size());
  for (int i = 0; i < vec1_size; ++i) {
    res[i] = vec1[i] + vec2[i];
  }
  return res;
}

void ComputeCost(const CostProviderBase& cost_provider,
                 DpMotionInfo* ptr_motion_info) {
  auto& dp_motion_info = *ptr_motion_info;
  dp_motion_info.costs.resize(cost_provider.weights().size());
  dp_motion_info.ignored_trajs = cost_provider.ComputeInteractiveDpCost(
      dp_motion_info.start_t, dp_motion_info.motion_form,
      dp_motion_info.ignored_trajs, absl::MakeSpan(dp_motion_info.costs));
}

// Sample by acceleration.
void SampleDynamicMotions(
    double traj_horizon, MotionNodeIndex motion_node_idx,
    MotionEdgeIndex prev_motion_edge_idx, const MotionGraph& motion_graph,
    const GeometryEdge& geom_edge, const IgnoreTrajMap& ignored_trajs,
    const MotionConstraintParamsProto& motion_constraint_params,
    const CostProviderBase& cost_provider, const MotionGraphCache& cost_cache,
    bool sample_const_v, std::vector<DpMotionInfo>* ptr_motions,
    std::vector<NewCacheInfo>* new_motion_forms_cache) {
  const auto motion_state = motion_graph.GetMotionNode(motion_node_idx).state;
  const double v0 = motion_state.v;
  const double t0 = motion_state.t;
  auto& motions = *ptr_motions;
  // expand motion and get cost at the same time. if cannot get cost from cache,
  // record this idx and compute cost outside this function.

  // Create keys for the cost fetching.
  absl::flat_hash_set<MotionEdgeKey> unrepeated_keys;
  const auto expand_motion_by_a = [&unrepeated_keys, &geom_edge, t0](
                                      double v0, double a0) {
    unrepeated_keys.emplace(MotionEdgeKey(a0, v0, t0, geom_edge.index));
  };

  QCHECK_GE(v0, 0.0);
  const double a_max = motion_constraint_params.max_acceleration();
  const double a_min = motion_constraint_params.max_deceleration();
  QCHECK_LT(a_min, 0.0);
  QCHECK_GT(a_max, 0.0);
  const double v_limit =
      Mph2Mps(motion_constraint_params.default_speed_limit());     // m/s
  const double reciprocal_s = 1.0 / geom_edge.geometry->length();  // 1/m
  const double pos_a_limit = (Sqr(v_limit) - Sqr(v0)) * reciprocal_s * 0.5;
  const double stop_a = -Sqr(v0) * reciprocal_s * 0.5;

  double a_lower = std::max(a_min, stop_a);
  double a_upper = std::min(pos_a_limit, a_max);
  std::set<double> acc_samples;
  if (!sample_const_v) {
    if (FLAGS_planner_initializer_enable_clip) {
      const double a0 = motion_state.a;
      constexpr double kAccVariationRange = 1.0;  // m/s^2
      a_lower = std::max(a_lower, a0 - kAccVariationRange);
      a_upper = std::min(a_upper, a0 + kAccVariationRange);
    }
    const auto a_begin =
        std::lower_bound(kAccelerationSamplePoints.begin(),
                         kAccelerationSamplePoints.end(), a_lower);
    const auto a_end =
        std::lower_bound(a_begin, kAccelerationSamplePoints.end(), a_upper);
    // Sample the valid accelerations on the sampling grid.
    for (auto it = a_begin; it != a_end; ++it) {
      acc_samples.insert(*it);
    }
  }

  acc_samples.insert(a_lower);
  acc_samples.insert(a_upper);
  acc_samples.insert(a_min);
  acc_samples.insert(0.0);
  for (const double acc : acc_samples) {
    expand_motion_by_a(v0, acc);
  }

  // Collect costs from cache or calculate costs.

  // Prepare containers.
  std::vector<MotionEdgeKey> keys;
  keys.reserve(unrepeated_keys.size());
  for (auto& key : unrepeated_keys) {
    keys.push_back(key);
  }
  motions.reserve(keys.size());
  for (int i = 0; i < keys.size(); ++i) {
    motions.emplace_back(
        DpMotionInfo({.key = keys[i],
                      .sum_cost = 0.0,
                      .start_t = t0,
                      .prev_motion_edge_index = prev_motion_edge_idx,
                      .end_geometry_node_index = geom_edge.end,
                      .motion_form = nullptr,
                      .geometry_edge_index = geom_edge.index,
                      .ignored_trajs = ignored_trajs}));
  }
  std::vector<int> failed_idx;
  failed_idx.reserve(keys.size());
  // Fill in the calculated ones.
  cost_cache.BatchGetOrFail(keys, &motions, &failed_idx);

  // Calculated all costs and create new MotionForms for the failed_idx motion
  // edges given the cost_provider.
  std::vector<std::unique_ptr<MotionForm>> new_motion_forms;
  new_motion_forms.reserve(failed_idx.size());
  for (int i = 0; i < failed_idx.size(); ++i) {
    const int cur_failed_idx = failed_idx[i];
    const auto& key = keys[cur_failed_idx];
    new_motion_forms.emplace_back(std::make_unique<ConstAccelMotion>(
        traj_horizon, key.v0(), key.a0(), geom_edge.geometry));
    motions[cur_failed_idx].motion_form = new_motion_forms[i].get();
  }

  for (int i = 0; i < failed_idx.size(); ++i) {
    const int cur_failed_idx = failed_idx[i];
    ComputeCost(cost_provider, &motions[cur_failed_idx]);
    MotionEdgeCache new_cache(
        {.ptr_motion_form = std::move(new_motion_forms[i]),
         .costs = motions[cur_failed_idx].costs,
         .ignored_trajs = motions[cur_failed_idx].ignored_trajs});
    new_motion_forms_cache->push_back(NewCacheInfo{
        .key = keys[cur_failed_idx], .cache = std::move(new_cache)});
  }

  for (int i = 0; i < keys.size(); ++i) {
    if (std::find(failed_idx.begin(), failed_idx.end(), i) ==
        failed_idx.end()) {
      cost_provider.ComputeLeadingObjCost(motions[i].start_t,
                                          motions[i].motion_form,
                                          absl::MakeSpan(motions[i].costs));
    }
    motions[i].sum_cost = absl::c_accumulate(motions[i].costs, 0.0);
  }
}

// Special handling for motion expansion from start node (planning start state).
std::vector<DpMotionInfo> ExpandStartMotionEdges(
    double traj_horizon, GeometryNodeIndex geom_node_idx,
    MotionNodeIndex motion_node_idx, const GeometryGraph& geometry,
    const MotionGraph& motion_graph,
    const MotionConstraintParamsProto& motion_constraint_params,
    const CostProviderBase& cost_provider, const MotionGraphCache& cost_cache,
    std::vector<NewCacheInfo>* new_motion_forms) {
  SCOPED_QTRACE("ExpandStartMotionEdges");
  const auto& outgoing_edge_idxs = geometry.GetOutgoingEdges(geom_node_idx);
  std::vector<DpMotionInfo> motions;  // Candidate motions (to consider).
  motions.reserve((kAccelerationSamplePoints.size() + 3) *
                  outgoing_edge_idxs.size());
  for (const auto& outgoing_edge_idx : outgoing_edge_idxs) {
    const auto& geom_edge = geometry.GetEdge(outgoing_edge_idx);
    if (!geometry.IsActive(outgoing_edge_idx)) {
      continue;
    }
    // No previous edge index, set to invalid.
    // Sample start from this motion node, based on this geom_edge, sampling
    // by different accelerations.
    std::vector<DpMotionInfo> motions_per_edge;
    SampleDynamicMotions(traj_horizon, motion_node_idx,
                         MotionEdgeVector<MotionEdge>::kInvalidIndex,
                         motion_graph, geom_edge, /*ignored_trajs=*/{},
                         motion_constraint_params, cost_provider, cost_cache,
                         /*sample_const_v=*/false, &motions_per_edge,
                         new_motion_forms);
    for (int i = 0; i < motions_per_edge.size(); i++) {
      motions.emplace_back(std::move(motions_per_edge[i]));
    }
  }
  return motions;
}

// Expand motions from an intermediate geometry node.
std::vector<DpMotionInfo> ExpandMotionEdges(
    double traj_horizon, GeometryNodeIndex geom_node_idx,
    const std::vector<MotionEdgeIndex>& motion_edge_idxes,
    const MotionEdgeVector<IgnoreTrajMap>& ignored_trajs_vector,
    const GeometryGraph& geometry, const MotionGraph& motion_graph,
    const MotionConstraintParamsProto& motion_constraint_params,
    const CostProviderBase& cost_provider, bool sample_const_v,
    const MotionGraphCache& cost_cache,
    std::vector<NewCacheInfo>* new_motion_forms) {
  SCOPED_QTRACE("ExpandMotionEdges");

  const auto& outgoing_edge_idxs = geometry.GetOutgoingEdges(geom_node_idx);
  std::vector<DpMotionInfo> motions;
  motions.reserve((kAccelerationSamplePoints.size() + 2) *
                  outgoing_edge_idxs.size() * motion_edge_idxes.size());
  for (const auto& outgoing_edge_idx : outgoing_edge_idxs) {
    const auto& geom_edge = geometry.GetEdge(outgoing_edge_idx);
    if (!geometry.IsActive(outgoing_edge_idx)) {
      continue;
    }
    for (const auto& motion_edge_idx : motion_edge_idxes) {
      // Motion_edge_idexes are the motions to expand (decide by certain
      // conditions) which are terminating at this particular geometrty node.
      std::vector<DpMotionInfo> motions_per_edge;
      const auto motion_node_idx =
          motion_graph.GetMotionEdge(motion_edge_idx).end;
      const auto& ignored_trajs = ignored_trajs_vector[motion_edge_idx];
      SampleDynamicMotions(traj_horizon, motion_node_idx, motion_edge_idx,
                           motion_graph, geom_edge, ignored_trajs,
                           motion_constraint_params, cost_provider, cost_cache,
                           sample_const_v, &motions_per_edge, new_motion_forms);
      for (int i = 0; i < motions_per_edge.size(); i++) {
        motions.emplace_back(std::move(motions_per_edge[i]));
      }
    }
  }
  return motions;
}

std::vector<TrajInfo> TopKTrajectories(
    absl::Span<const MotionEdgeIndex> terminated_edge_idxes,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    int k_top_trajectories) {
  std::vector<TrajInfo> top_k_trajs;
  top_k_trajs.reserve(terminated_edge_idxes.size());
  for (const auto& term_idx : terminated_edge_idxes) {
    top_k_trajs.push_back(TrajInfo(
        {.idx = term_idx, .total_cost = search_costs[term_idx].cost_to_come}));
  }
  std::stable_sort(top_k_trajs.begin(), top_k_trajs.end(),
                   [](const TrajInfo& traj1, const TrajInfo& traj2) {
                     return traj1.total_cost < traj2.total_cost;
                   });
  if (k_top_trajectories < top_k_trajs.size()) {
    top_k_trajs.resize(k_top_trajectories);
  }

  return top_k_trajs;
}

BestEdgeInfo FindBestEdge(
    double traj_horizon, const CostProviderBase& cost_provider,
    const MotionState& sdc_motion, MotionNodeIndex sdc_node_index,
    const GeometryNodeIndex& sdc_geom_node, MotionGraph* mutable_motion_graph,
    MotionGraphCache* cost_cache,
    MotionEdgeVector<MotionSearchOutput::SearchCost>* ptr_mutable_search_costs,
    MotionEdgeVector<IgnoreTrajMap>* ptr_mutable_ignore_traj_map,
    std::vector<MotionEdgeIndex>* ptr_mutable_terminated_idxes) {
  SCOPED_QTRACE("FindBestEdge");
  auto& mutable_search_costs = *ptr_mutable_search_costs;
  auto& mutable_terminated_idxes = *ptr_mutable_terminated_idxes;
  MotionEdgeIndex best_final_edge = MotionEdgeVector<MotionEdge>::kInvalidIndex;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0, n = mutable_terminated_idxes.size(); i < n; ++i) {
    const auto terminated_idx = mutable_terminated_idxes[i];
    const auto total_cost = mutable_search_costs[terminated_idx].cost_to_come;
    if (total_cost < min_cost) {
      best_final_edge = terminated_idx;
      min_cost = total_cost;
    }
  }

  // If sdc is at low speed and we cannot find a reasonable motion, create a
  // stationary motion.
  bool is_created_stationary_motion = false;
  if (best_final_edge == MotionEdgeVector<MotionEdge>::kInvalidIndex &&
      sdc_motion.v < kSearchFailedCanSetToZeroSpeed) {
    is_created_stationary_motion = true;
    auto set2zero_sdc_motion = sdc_motion;
    set2zero_sdc_motion.v = 0.0;
    MotionEdgeKey stationary_motion_key = MotionEdgeKey(
        set2zero_sdc_motion.v, set2zero_sdc_motion.a, set2zero_sdc_motion.t,
        GeometryEdgeVector<GeometryEdge>::kInvalidIndex);
    // Feature cost for this stationary motion not initialized.
    const auto end_node_index =
        mutable_motion_graph->AddMotionNode(set2zero_sdc_motion, sdc_geom_node);
    const auto ptr_motion_form_or =
        cost_cache->GetMotionForm(stationary_motion_key);
    MotionForm* ptr_motion_form = nullptr;
    if (!ptr_motion_form_or.ok()) {
      // Stationary motion form not constructed yet.
      std::unique_ptr<MotionForm> stationary_motion =
          std::make_unique<StationaryMotion>(
              traj_horizon,
              GeometryState{.xy = sdc_motion.xy,
                            .h = sdc_motion.h,
                            .k = sdc_motion.k,
                            .accumulated_s = sdc_motion.accumulated_s,
                            .l = sdc_motion.l});
      ptr_motion_form = stationary_motion.get();
      DpMotionInfo temp_motion_info{.sum_cost = 0.0,
                                    .start_t = 0.0,
                                    .motion_form = stationary_motion.get()};
      ComputeCost(cost_provider, &temp_motion_info);
      cost_cache->Insert(stationary_motion_key, temp_motion_info.costs,
                         /*ignored_trajs=*/{}, std::move(stationary_motion));
      MotionSearchOutput::SearchCost new_search_cost{
          .feature_cost = temp_motion_info.costs,
          .cost_to_come = absl::c_accumulate(temp_motion_info.costs, 0.0)};
      mutable_search_costs.push_back(new_search_cost);
      min_cost = new_search_cost.cost_to_come;
    } else {
      ptr_motion_form = *ptr_motion_form_or;
      // Motion form and costs should exists in cache at the same time.
      const auto feature_costs =
          cost_cache->GetCosts(stationary_motion_key).value();
      MotionSearchOutput::SearchCost new_search_cost{
          .feature_cost = feature_costs,
          .cost_to_come = absl::c_accumulate(feature_costs, 0.0),
      };
      mutable_search_costs.push_back(new_search_cost);
      min_cost = new_search_cost.cost_to_come;
    }
    const auto motion_edge_index = mutable_motion_graph->AddMotionEdge(
        sdc_node_index, end_node_index, ptr_motion_form, sdc_geom_node,
        /*prev_edge=*/MotionEdgeVector<MotionEdge>::kInvalidIndex);
    // Push an empty IgnoreTrajMap.
    ptr_mutable_ignore_traj_map->push_back(/*ignored_traj_map=*/{});
    mutable_terminated_idxes.push_back(motion_edge_index);
    best_final_edge = motion_edge_index;
  }
  return {.idx = best_final_edge,
          .total_cost = min_cost,
          .is_created_stationary_motion = is_created_stationary_motion};
}

}  // namespace qcraft::planner
