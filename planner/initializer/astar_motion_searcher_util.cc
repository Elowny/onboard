#include "onboard/planner/initializer/astar_motion_searcher_util.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "boost/heap/fibonacci_heap.hpp"
#include "boost/intrusive/link_mode.hpp"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph_cache.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

constexpr double kCaptainNetHeuristicWeight = 7.0;
constexpr double kReferenceLineHeuristicWeightDist = 3.0;
constexpr double kReferenceLineHeuristicWeightVel = 10.0;

int CreateNodeKey(const double v, const double t,
                  const GeometryNodeIndex& geom_index) {
  constexpr double kAstarVelocityResolution = 3.0;
  constexpr double kAstarTimeResolution = 2.5;
  const int v_index = RoundToInt(v / kAstarVelocityResolution);
  const int t_index = RoundToInt(t / kAstarTimeResolution);
  const int v_dir = v_index >= 0 ? 1 : 0;
  const int index =
      geom_index.value() + (v_index << 10) + (t_index << 20) + (v_dir << 30);
  return index;
}

AstarNode CreateNode(const MotionState& state, const int key,
                     const GeometryNodeIndex& geom_index) {
  return AstarNode{.key = key,
                   .layer = 0,
                   .parent = 0,
                   .geom_index = geom_index,
                   .cost_g = 0.0,
                   .cost_h = 0.0,
                   .feature_costs = {},
                   .state = state,
                   .motion = nullptr,
                   .ignored_trajs = {}};
}

double ComputeCaptainNetInspiredHeuristicCost(
    const double traj_horizon, const AstarNode& node,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points) {
  // Euclidean_distance(node, the time-closest point in ref traj) +
  // Euclidean_distance(the chosen point in ref traj, last point in ref traj)
  if (captain_net_traj_points.empty()) {
    return 0.0;
  }

  const double t = node.state.t;
  auto it = std::lower_bound(
      captain_net_traj_points.begin(), captain_net_traj_points.end(), t,
      [](const auto& pt, double t) { return pt.relative_time() < t; });
  if (it == captain_net_traj_points.end()) {
    it = std::prev(it);
  }
  const auto closed_pt = Vec2dFromApolloTrajectoryPointProto(*it);
  const auto last_it = std::min_element(
      captain_net_traj_points.begin(), captain_net_traj_points.end(),
      [&traj_horizon](const auto& p1, const auto& p2) {
        return std::abs(traj_horizon - p1.relative_time()) <
               std::abs(traj_horizon - p2.relative_time());
      });
  const auto last_pt = Vec2dFromApolloTrajectoryPointProto(*last_it);
  const double dist_1 = node.state.xy.DistanceTo(closed_pt);
  const double dist_2 = closed_pt.DistanceTo(last_pt);
  const double h = kCaptainNetHeuristicWeight * (dist_1 + dist_2);
  return h;
}

double ComputeReferenceLineInspiredHeuristicCost(
    const AstarNode& node, const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table) {
  // h_dist + h_vel
  // where h_dist = k_dist * (Euclidean_distance(node, the closest point in
  // reference line) + Euclidean_distance(the point in ref line, the last point
  // in ref line)) h_vel = k_vel * (v_reference - v_node)
  if (reference_line_points.empty()) {
    return 0.0;
  }

  // heuristic for destination
  const auto node_xy = node.state.xy;
  const auto closest_pt = *std::min_element(
      reference_line_points.begin(), reference_line_points.end(),
      [&node_xy](const auto& p1, const auto& p2) {
        return node_xy.DistanceTo(p1) < node_xy.DistanceTo(p2);
      });
  const double dist_1 = node_xy.DistanceTo(closest_pt);
  const double dist_2 = closest_pt.DistanceTo(reference_line_points.back());
  const double h_dist = kReferenceLineHeuristicWeightDist * (dist_1 + dist_2);

  // heuristic for velocity
  const auto [v_limit, v_ref] = reference_speed_table->LookUpRefSpeed(
      node.state.t, node.state.accumulated_s);
  const double h_vel =
      kReferenceLineHeuristicWeightVel * (std::abs(v_ref - node.state.v));

  return h_dist + h_vel;
}

double ComputeHeuristicCost(
    const double traj_horizon, const AstarNode& node,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    const bool is_run_model_l4) {
  if (is_run_model_l4) {
    // if L4, use captain net trajectory for heuristic
    return ComputeCaptainNetInspiredHeuristicCost(traj_horizon, node,
                                                  captain_net_traj_points);
  } else {
    // if not L4, use rule-base generated reference line and reference speed
    // table for heuristic
    return ComputeReferenceLineInspiredHeuristicCost(
        node, reference_line_points, reference_speed_table);
  }
}

std::set<double> SampleMotionForms(
    const double /*traj_horizon*/, const AstarNode& astar_node,
    const GeometryEdge& geom_edge,
    const MotionConstraintParamsProto& motion_constraint_params,
    const bool sample_const_v) {
  const auto motion_state = astar_node.state;
  const double v0 = motion_state.v;
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

  return acc_samples;
}

void ProcessMotionFormsWithCache(
    const double traj_horizon, const AstarNode& parent_node,
    const GeometryEdge& geom_edge, const std::set<double>& acc_samples,
    const CostProviderBase& cost_provider,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    OpenPQ* open_pq, OpenMap* open_map, const CloseMap& close_map,
    const AstarGraphCache& cost_cache, std::vector<AstarCacheInfo>* new_cache,
    const bool is_run_model_l4, absl::Mutex* my_mutex) {
  SCOPED_QTRACE("ProcessMotionForms");
  // Collect motions and costs from cache or create it.
  // Create keys for the cost fetching.
  const double t0 = parent_node.state.t;
  const double v0 = parent_node.state.v;
  absl::flat_hash_set<MotionEdgeKey> unrepeated_keys;
  const auto expand_motion_by_a = [&unrepeated_keys, &geom_edge, t0](
                                      double v0, double a0) {
    unrepeated_keys.emplace(MotionEdgeKey(a0, v0, t0, geom_edge.index));
  };
  for (const double acc : acc_samples) {
    expand_motion_by_a(v0, acc);
  }

  // Prepare containers.
  std::vector<MotionEdgeKey> keys;
  keys.reserve(unrepeated_keys.size());
  for (auto& key : unrepeated_keys) {
    keys.push_back(key);
  }

  std::vector<AstarCacheInfo> infos;
  infos.reserve(keys.size());
  for (int i = 0; i < keys.size(); ++i) {
    infos.emplace_back(AstarCacheInfo({.key = keys[i], .node = nullptr}));
  }
  std::vector<int> failed_idx;
  failed_idx.reserve(keys.size());
  // Fill in the calculated ones.
  cost_cache.BatchGetOrFail(keys, &infos, &failed_idx);

  // Calculated all costs and create new MotionForms for the failed_idx motion
  // edges given the cost_provider.
  std::vector<int> closed_idx;
  for (int i = 0, n = failed_idx.size(); i < n; ++i) {
    const int cur_failed_idx = failed_idx[i];
    const auto& key = keys[cur_failed_idx];
    const auto new_motion_form = std::make_shared<ConstAccelMotion>(
        traj_horizon, key.v0(), key.a0(), geom_edge.geometry);
    auto motion_state = new_motion_form->GetEndMotionState();
    motion_state.s += parent_node.state.s;
    motion_state.t += parent_node.state.t;
    int new_key = CreateNodeKey(motion_state.v, motion_state.t, geom_edge.end);
    if (close_map.find(new_key) != close_map.end()) {
      closed_idx.push_back(cur_failed_idx);
      continue;
    }
    auto new_node = CreateNode(motion_state, new_key, geom_edge.end);
    std::vector<double> edge_cost;
    edge_cost.resize(cost_provider.weights().size());
    new_node.ignored_trajs = cost_provider.ComputeInteractiveAstarCost(
        parent_node.state.t, new_motion_form.get(), parent_node.ignored_trajs,
        absl::MakeSpan(edge_cost));
    new_node.cost_g = absl::c_accumulate(edge_cost, 0.0) + parent_node.cost_g;
    new_node.cost_h = ComputeHeuristicCost(
        traj_horizon, new_node, reference_line_points, reference_speed_table,
        captain_net_traj_points, is_run_model_l4);
    new_node.feature_costs = edge_cost;
    new_node.parent = parent_node.key;
    new_node.layer = parent_node.layer + 1;
    new_node.motion = new_motion_form;
    const auto new_node_ptr = std::make_shared<AstarNode>(new_node);
    infos[cur_failed_idx].node = new_node_ptr;
    new_cache->push_back(AstarCacheInfo{.key = key, .node = new_node_ptr});
  }

  for (int i = 0, n = keys.size(); i < n; ++i) {
    if (std::find(failed_idx.begin(), failed_idx.end(), i) ==
        failed_idx.end()) {
      cost_provider.ComputeLeadingObjCost(
          t0, infos[i].node->motion.get(),
          absl::MakeSpan(infos[i].node->feature_costs));
      infos[i].node->cost_g =
          absl::c_accumulate(infos[i].node->feature_costs, 0.0) +
          parent_node.cost_g;
      infos[i].node->parent = parent_node.key;
      infos[i].node->layer = parent_node.layer + 1;
    }
  }

  for (int i = 0; i < infos.size(); ++i) {
    if (std::find(closed_idx.begin(), closed_idx.end(), i) !=
        closed_idx.end()) {
      continue;
    }
    const auto current_node = std::move(infos[i].node);
    const int current_key = current_node->key;
    const auto current_state = current_node->state;
    if (close_map.find(current_key) != close_map.end()) {
      // The key is in CLOSE, skip
      continue;
    }

    {
      // Mutex lock region
      absl::MutexLock lock(my_mutex);
      auto it = open_map->find(current_key);
      if (it == open_map->end()) {
        // Not in OPEN
        if (current_state.v < 0.1 && current_state.t < traj_horizon) {
          // Discard super slow state if it fails to reach the end of time
          // horizon.
          continue;
        }

        open_map->emplace(current_key, open_pq->emplace(current_node));
      } else {
        // update node information already in OPEN
        if (current_node->cost_g < (*it->second)->cost_g) {
          open_pq->update(it->second, current_node);
        }
      }
    }  // Mutex lock region
  }
}

void ProcessMotionFormsWithoutCache(
    const double traj_horizon, const AstarNode& parent_node,
    const GeometryEdge& geom_edge, const std::set<double>& acc_samples,
    const CostProviderBase& cost_provider,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    OpenPQ* open_pq, OpenMap* open_map, const CloseMap& close_map,
    const bool is_run_model_l4, absl::Mutex* my_mutex) {
  SCOPED_QTRACE("ProcessMotionForms");
  // Create motion forms from acc samples
  std::vector<std::shared_ptr<MotionForm>> motion_form_candidates;
  motion_form_candidates.reserve(acc_samples.size());
  const double v0 = parent_node.state.v;
  for (const double acc : acc_samples) {
    motion_form_candidates.emplace_back(std::make_shared<ConstAccelMotion>(
        traj_horizon, v0, acc, geom_edge.geometry));
  }

  for (const auto& motion_form : motion_form_candidates) {
    auto motion_state = motion_form->GetEndMotionState();
    motion_state.s += parent_node.state.s;
    motion_state.t += parent_node.state.t;
    int new_key = CreateNodeKey(motion_state.v, motion_state.t, geom_edge.end);

    if (close_map.find(new_key) != close_map.end()) {
      // The key is in CLOSE, skip
      continue;
    }

    // Create new node
    auto new_node = CreateNode(motion_state, new_key, geom_edge.end);
    std::vector<double> edge_cost;
    edge_cost.resize(cost_provider.weights().size());
    new_node.ignored_trajs = cost_provider.ComputeInteractiveAstarCost(
        parent_node.state.t, motion_form.get(), parent_node.ignored_trajs,
        absl::MakeSpan(edge_cost));
    new_node.cost_g = absl::c_accumulate(edge_cost, 0.0) + parent_node.cost_g;
    new_node.cost_h = ComputeHeuristicCost(
        traj_horizon, new_node, reference_line_points, reference_speed_table,
        captain_net_traj_points, is_run_model_l4);
    new_node.feature_costs = edge_cost;
    new_node.parent = parent_node.key;
    new_node.layer = parent_node.layer + 1;
    new_node.motion = motion_form;
    const auto new_node_ptr = std::make_shared<AstarNode>(new_node);

    {
      // Mutex lock region
      absl::MutexLock lock(my_mutex);
      auto it = open_map->find(new_key);
      if (it == open_map->end()) {
        // Not in OPEN
        if (motion_state.v < 0.1 && motion_state.t < traj_horizon) {
          // Discard super slow state if it fails to reach the end of time
          // horizon.
          continue;
        }

        open_map->emplace(new_key, open_pq->emplace(new_node_ptr));
      } else {
        // update node information already in OPEN
        if (new_node.cost_g < (*it->second)->cost_g) {
          open_pq->update(it->second, new_node_ptr);
        }
      }
    }  // Mutex lock region
  }
}

AstarNode SearchFailTryStationaryMotion(
    const double traj_horizon, const CostProviderBase& cost_provider,
    const AstarNode& parent_node,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    const bool is_run_model_l4) {
  SCOPED_QTRACE("SearchFailTryStationaryMotion");
  // If sdc is at low speed and we cannot find a reasonable motion, create a
  // stationary motion.
  const auto stationary_state = parent_node.state;
  std::shared_ptr<MotionForm> stationary_motion =
      std::make_shared<StationaryMotion>(
          traj_horizon,
          GeometryState{.xy = stationary_state.xy,
                        .h = stationary_state.h,
                        .k = stationary_state.k,
                        .accumulated_s = stationary_state.accumulated_s,
                        .l = stationary_state.l});
  auto motion_state = stationary_motion->GetEndMotionState();
  auto stationary_node = CreateNode(motion_state, /*key=*/0, {});

  std::vector<double> edge_cost;
  edge_cost.resize(cost_provider.weights().size());
  stationary_node.ignored_trajs = cost_provider.ComputeInteractiveAstarCost(
      parent_node.state.t, stationary_motion.get(), /*ignored_trajs=*/{},
      absl::MakeSpan(edge_cost));

  stationary_node.cost_g = absl::c_accumulate(edge_cost, 0.0);
  stationary_node.cost_h = ComputeHeuristicCost(
      traj_horizon, stationary_node, reference_line_points,
      reference_speed_table, captain_net_traj_points, is_run_model_l4);
  stationary_node.feature_costs = edge_cost;
  stationary_node.parent = parent_node.key;
  stationary_node.layer = parent_node.layer + 1;
  stationary_node.motion = std::move(stationary_motion);

  return stationary_node;
}

bool SatisfyEndCondition(const double traj_horizon, const AstarNode& node) {
  bool should_end = false;
  if (node.state.t >= traj_horizon) {
    should_end = true;
  }
  return should_end;
}

}  // namespace qcraft::planner
