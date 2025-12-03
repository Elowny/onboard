#include "onboard/prediction/post_process/object_svt_graph.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <iterator>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/prediction/post_process/conflict_resolver_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace {

constexpr double kZeroVelocityEpsilon = 0.1;  // m/s.
constexpr int kMaxSSegSize = 6;
constexpr int kMinSSegSize = 3;
constexpr double kAssumeAccResponseTime = 0.2;  // s.

std::vector<double> StandardDiscretizdSByInitSpeedPoint(
    const planner::SpeedPoint& point, double total_length) {
  std::vector<double> s;
  s.push_back(0.0);
  double ds = point.v() * kAssumeAccResponseTime;
  const int size =
      std::clamp(CeilToInt(total_length / ds), kMinSSegSize, kMaxSSegSize);
  ds = total_length / size;
  for (int i = 0; i < size - 1; ++i) {
    s.push_back(ds + s.back());
  }
  s.push_back(total_length);
  return s;
}

void ExpandCandidateEdgesFromNode(
    const SvtEdgeIndex& prev_edge_index, const SvtNodeIndex& cur_node_index,
    const std::vector<double>& a_samples, double length,
    std::vector<DpEdgeInfo>* candidates_container) {
  // Given a previous edge, current node, expand according to acc.
  candidates_container->reserve(a_samples.size());
  for (const auto& a : a_samples) {
    candidates_container->push_back(DpEdgeInfo({
        .start_index = cur_node_index,
        .prev_edge_index = prev_edge_index,
        .a = a,
        .length = length,
    }));
  }
}

// Fill in final_t, final_v, SvtEdgeCost in DpEdgeInfo.
void ParallelComputeCost(const SvtCostProvider* cost_provider,
                         const std::vector<SvtNode>& nodes,
                         const SvtEdgeVector<SvtEdgeCost>& search_costs,
                         double sampling_ds,
                         std::vector<DpEdgeInfo>* ptr_candidate_edges,
                         ThreadPool* thread_pool) {
  FUNC_QTRACE();
  auto& candidate_edges = *ptr_candidate_edges;  // mutable.
  ParallelFor(0, candidate_edges.size(), thread_pool, [&](int i) {
    auto& candidate_dp_info = candidate_edges[i];
    const auto& prev_edge_index = candidate_dp_info.prev_edge_index;
    double prev_edge_sum_cost =
        prev_edge_index == SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex
            ? 0.0
            : search_costs[prev_edge_index].sum_cost;
    cost_provider->ComputeCost(nodes, prev_edge_sum_cost, sampling_ds,
                               &candidate_dp_info);
  });
}

void UpdateVtOptimalMapFromCandidates(
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    VtOptimal* ptr_vt_optimal_layer,
    std::vector<DpEdgeInfo>* ptr_movable_candidates) {
  FUNC_QTRACE();
  auto& candidates = *ptr_movable_candidates;
  auto& vt_optimal_layer = *ptr_vt_optimal_layer;
  for (auto& candidate : candidates) {
    const VtGrid key(candidate.final_v, candidate.final_t,
                     general_config.v_discrete(), general_config.t_discrete());
    auto* iter = FindOrNull(vt_optimal_layer, key);
    if (iter == nullptr) {
      vt_optimal_layer.emplace(std::make_pair(key, std::move(candidate)));
    } else {
      const auto& candidate_cost = candidate.cost;
      const auto& in_map_cost = iter->cost;
      if (candidate_cost.sum_cost < in_map_cost.sum_cost) {
        *iter = std::move(candidate);
      }
    }
  }
}

SvtEdgeIndex FindBestEdge(const std::vector<SvtEdge>& edges,
                          const SvtEdgeVector<SvtEdgeCost>& search_costs,
                          const std::vector<SvtEdgeIndex>& final_candidates) {
  FUNC_QTRACE();
  VLOG(3) << "Find final candidates in " << final_candidates.size()
          << " edges.";
  double min_cost = std::numeric_limits<double>::infinity();
  SvtEdgeIndex min_cost_edge_idx = SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex;
  for (const auto& idx : final_candidates) {
    const auto& edge = edges[idx.value()];
    const auto& edge_cost = search_costs[idx];
    const double inv_t = 1.0 / edge.final_t;
    if (edge_cost.sum_cost * inv_t < min_cost) {
      min_cost = edge_cost.sum_cost * inv_t;
      min_cost_edge_idx = idx;
    }
  }
  return min_cost_edge_idx;
}

void SampleAndAddSpeedPoint(const std::vector<SvtEdge>& edges,
                            const std::vector<SvtNode>& nodes,
                            SvtEdgeIndex edge_idx, double ds,
                            std::vector<planner::SpeedPoint>* speed_points) {
  const auto& edge = edges[edge_idx.value()];
  const auto states = SampleEdge(nodes, edge, ds);
  for (const auto& state : states) {
    const auto a = state.v == 0.0 ? 0.0 : edge.a;
    speed_points->push_back(SvtStateToSpeedPoint(state, a));
  }
}

std::unique_ptr<planner::SpeedVector> ConstructSpeedVector(
    const std::vector<SvtEdge>& edges, const std::vector<SvtNode>& nodes,
    SvtEdgeIndex best_edge_idx, double ds,
    std::vector<SvtEdgeIndex>* ptr_edges_from_start) {
  FUNC_QTRACE();
  auto& edges_from_start = *ptr_edges_from_start;
  SvtEdgeIndex cur_idx = best_edge_idx;
  while (cur_idx != SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex) {
    edges_from_start.push_back(cur_idx);
    cur_idx = edges[cur_idx.value()].prev_edge_index;
  }
  std::reverse(edges_from_start.begin(), edges_from_start.end());
  std::vector<planner::SpeedPoint> speed_points;
  for (const auto& edge_idx : edges_from_start) {
    SampleAndAddSpeedPoint(edges, nodes, edge_idx, ds, &speed_points);
  }
  return std::make_unique<planner::SpeedVector>(std::move(speed_points));
}

}  // namespace

ObjectSvtGraph::ObjectSvtGraph(const ObjectSvtGraphInput& input,
                               ThreadPool* thread_pool) {
  QCHECK_NOTNULL(input.object_proto);
  QCHECK_NOTNULL(input.stoplines);
  QCHECK_NOTNULL(input.stationary_objects);
  QCHECK_NOTNULL(input.ref_speed);
  QCHECK_NOTNULL(input.cost_provider);
  QCHECK_NOTNULL(input.general_config);
  QCHECK_NOTNULL(input.object_config);

  // 0. Fill in.
  general_config_ = input.general_config;
  object_config_ = input.object_config;
  thread_pool_ = thread_pool;
  object_proto_ = input.object_proto;
  cost_provider_ = input.cost_provider;
  ref_speed_ = input.ref_speed;
  traj_id_ = input.traj_id;

  // 1. collect st boundaries and decide s sampling resolution.
  min_unit_s_ = general_config_->min_s_seg_length();
  s_knots_ = StandardDiscretizdSByInitSpeedPoint(ref_speed_->front(),
                                                 ref_speed_->TotalLength());
  QCHECK_GE(s_knots_.size(), 1);
  VLOG(3) << "Discretizing in s dimension: " << absl::StrJoin(s_knots_, ",");
  vt_optimal_layers_.resize(s_knots_.size() - 1);
  // First layer doesn't need to record optimal edges.
  nodes_layers_.resize(s_knots_.size());

  // 2. get acceleration range from params.
  for (const auto& a : object_config_->acc_samples()) {
    a_range_.push_back(a);
  }
}

SvtNodeIndex ObjectSvtGraph::AddNode(double s, double v, double t,
                                     int s_layer_index) {
  SvtNodeIndex index(nodes_.size());
  nodes_.push_back(SvtNode({
      .s = s,
      .v = v,
      .t = t,
      .index = index,
      .s_index = s_layer_index,
  }));
  nodes_layers_[s_layer_index].push_back(index);
  return index;
}

SvtEdgeIndex ObjectSvtGraph::AddEdge(const SvtNodeIndex& start_idx,
                                     const SvtNodeIndex& end_idx, double a,
                                     const SvtEdgeIndex& prev_edge_index,
                                     const SvtEdgeCost& edge_cost,
                                     const std::vector<SvtState>* states,
                                     double final_t, double final_v) {
  SvtEdgeIndex index(edges_.size());
  edges_.push_back(SvtEdge({
      .start_index = start_idx,
      .end_index = end_idx,
      .a = a,
      .index = index,
      .prev_edge_index = prev_edge_index,
      .final_t = final_t,
      .final_v = final_v,
      .states = states,
      .edge_cost = &edge_cost,
  }));
  // Start SvtNode of this edge should already have been built.
  search_costs_.push_back(edge_cost);
  return index;
}

absl::StatusOr<planner::SpeedVector> ObjectSvtGraph::Search(
    const bool visual_to_debug) {
  SCOPED_QTRACE("ConflictResolver::ObjectSvtGraph::Search");

  // 1. Prepare Start layer node (one node with v_0, a_0 from ref speed
  // vector).
  const auto& ref_speed = *ref_speed_;
  const auto& start_speed_point = ref_speed.front();
  const auto start_node_index = AddNode(
      start_speed_point.s(), start_speed_point.v(), start_speed_point.t(),
      /*s_layer_index=*/0);

  // 2. Update layer by layer. Use function ExpandFromSvtNode() to get candidate
  // edges and compute the cost.
  std::vector<std::vector<SvtEdgeIndex>> edges_to_expand_layers;
  edges_to_expand_layers.resize(s_knots_.size());

  // After expansion, collect all candidate edges and compute them in parallel.
  std::vector<std::vector<DpEdgeInfo>> candidate_edges_layers;
  const absl::Cleanup cleanup_candidate_edges_layers =
      [&candidate_edges_layers] {
        DestroyContainerAsyncMarkSource(std::move(candidate_edges_layers),
                                        (QCRAFT_LOC).ToString());
      };
  candidate_edges_layers.resize(s_knots_.size());

  // Final candidate pool.
  std::vector<SvtEdgeIndex> final_candidates;
  for (int layer_index = 0; layer_index < s_knots_.size(); ++layer_index) {
    // For each layer, collect optimal edges to expand. (except for first
    // layer && last layer).
    if (layer_index != 0) {
      SCOPED_QTRACE("AddNodesAndEdges");
      // Process updated vt optimal map for this layer.
      const auto& vt_optimal = vt_optimal_layers_[layer_index - 1];
      for (const auto& pair : vt_optimal) {
        // Since updated, dp_edge_info in vt_optimal for this layer should
        // contain all the potential svt edges to be expanded to next layer.
        const auto& dp_edge_info = pair.second;
        const auto new_end_node_index =
            AddNode(s_knots_[layer_index], dp_edge_info.final_v,
                    dp_edge_info.final_t, layer_index);
        const auto new_edge_index =
            AddEdge(dp_edge_info.start_index, new_end_node_index,
                    dp_edge_info.a, dp_edge_info.prev_edge_index,
                    dp_edge_info.cost, dp_edge_info.states.get(),
                    dp_edge_info.final_t, dp_edge_info.final_v);

        if (layer_index == s_knots_.size() - 1) {
          // Add all edges terminating at final layer to final speed vector
          // pool.
          final_candidates.push_back(new_edge_index);
        } else {
          if (dp_edge_info.final_v <= kZeroVelocityEpsilon) {
            final_candidates.push_back(new_edge_index);
          } else {
            // Continue sampling till the final layer (reach final s).
            edges_to_expand_layers[layer_index].push_back(new_edge_index);
          }
        }
      }
    }

    if (layer_index == s_knots_.size() - 1) {
      break;
    }

    // Expand optimal edges (build candidate dp edge infos, compute cost).
    // (except for last layer).
    const int next_layer_index =
        std::min<int>(layer_index + 1, s_knots_.size() - 1);
    // Max length of the candidates edges.
    const double length = s_knots_[next_layer_index] - s_knots_[layer_index];

    if (layer_index == 0) {
      // Start layer, expand from start_node at different a_range_.
      SCOPED_QTRACE("ExpandCandidateEdgesFromStartNode");
      ExpandCandidateEdgesFromNode(SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex,
                                   start_node_index, a_range_, length,
                                   &candidate_edges_layers[layer_index]);
    } else {
      // TODO(changqing): ParallelFor each svt edges to expand for this layer.
      // Candidate SvtEdge container. (Just for current layer)
      std::vector<std::vector<DpEdgeInfo>> candidates_per_optimal_edge;
      const absl::Cleanup cleanup_candidates_per_optimal_edge =
          [&candidates_per_optimal_edge] {
            DestroyContainerAsyncMarkSource(
                std::move(candidates_per_optimal_edge),
                (QCRAFT_LOC).ToString());
          };
      candidates_per_optimal_edge.resize(
          edges_to_expand_layers[layer_index].size());
      const auto& edge_idxes_to_expand = edges_to_expand_layers[layer_index];

      ParallelFor(0, edge_idxes_to_expand.size(), thread_pool_, [&](int i) {
        const auto& edge_idx = edge_idxes_to_expand[i];
        const auto& edge = edges_[edge_idx.value()];
        auto& candidates_container = candidates_per_optimal_edge[i];
        SCOPED_QTRACE("ExpandCandidateEdgesFromLayerNode");
        ExpandCandidateEdgesFromNode(edge_idx, edge.end_index, a_range_, length,
                                     &candidates_container);
      });

      // Collect candidates_per_optimal_edge to candidate_edges_layers
      // container so we can parallel compute cost.
      for (auto& candidates : candidates_per_optimal_edge) {
        for (auto it = std::make_move_iterator(candidates.begin());
             it != std::make_move_iterator(candidates.end()); ++it) {
          candidate_edges_layers[layer_index].push_back(*it);
        }
      }
    }
    ParallelComputeCost(cost_provider_, nodes_, search_costs_,
                        general_config_->sampling_s_step_dp(),
                        &candidate_edges_layers[layer_index], thread_pool_);

    // Process candidate edges with computed cost.
    // Iterate through all candidate edges for current layer (except for last
    // layer). Update optimal svt edges(as DpEdgeInfo) for next layer.
    // vt_optimal_layers doesn't have map for the first layer.
    UpdateVtOptimalMapFromCandidates(*general_config_,
                                     &vt_optimal_layers_[layer_index],
                                     &candidate_edges_layers[layer_index]);
  }

  if (final_candidates.empty()) {
    // TODO(changqing): add clearer error message. (use object svt graph debug
    // string).
    return absl::InternalError("Cannot find any speed vector! Will abort!");
  }

  // Find Best Edge from final candidates.
  const auto best_edge_idx =
      FindBestEdge(edges_, search_costs_, final_candidates);
  if (best_edge_idx == SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex) {
    return absl::InternalError("Cannot find best edge idx in FindBestEdge!");
  }
  final_speed_vector_ = ConstructSpeedVector(
      edges_, nodes_, best_edge_idx, general_config_->sampling_s_step_final(),
      &edges_from_start_);
  if (visual_to_debug) {
    candidate_speed_vectors_.reserve(final_candidates.size());
    candidate_edges_from_start_.reserve(final_candidates.size());
    for (const auto& candidate_final_edge_idx : final_candidates) {
      std::vector<SvtEdgeIndex> temp_edge_container;
      candidate_speed_vectors_.push_back(ConstructSpeedVector(
          edges_, nodes_, candidate_final_edge_idx,
          general_config_->sampling_s_step_final(), &temp_edge_container));
      candidate_edges_from_start_.push_back(std::move(temp_edge_container));
    }
  }
  return *final_speed_vector_;
}

void ObjectSvtGraph::ToProto(ConflictResolverDebugProto* debug_proto) const {
  auto* graph_proto = debug_proto->add_svt_graphs();
  graph_proto->set_object_traj_id(traj_id_);
  const auto cost_names = cost_provider_->cost_names();
  for (const auto& node : nodes_) {
    *graph_proto->add_nodes() = node.ToProto();
  }
  for (const auto& edge : edges_) {
    *graph_proto->add_edges() = edge.ToProto(cost_names);
  }
  for (const auto& s : s_knots_) {
    graph_proto->add_s_knots(s);
  }
  *graph_proto->mutable_final_speed_profile() =
      SpeedVectorToSpeedProfile(*final_speed_vector_, edges_from_start_);
  QCHECK_EQ(candidate_speed_vectors_.size(),
            candidate_edges_from_start_.size());
  graph_proto->mutable_candidate_speed_profiles()->Reserve(
      candidate_speed_vectors_.size());
  for (int i = 0; i < candidate_edges_from_start_.size(); ++i) {
    *graph_proto->add_candidate_speed_profiles() =
        EdgeConnectionToSimpleSpeedProfile(cost_provider_->cost_names(), edges_,
                                           search_costs_,
                                           candidate_edges_from_start_[i]);
  }
  std::string nodes_layer;
  for (int i = 0; i < nodes_layers_.size(); ++i) {
    nodes_layer += absl::StrFormat("Layer index: %d, %d nodes.", i,
                                   nodes_layers_[i].size());
  }
  graph_proto->set_node_size_on_each_layer(nodes_layer);
}
}  // namespace prediction
}  // namespace qcraft
