#ifndef ONBOARD_PREDICTION_POST_PROCESS_OBJECT_SVT_SAMPLE_H_
#define ONBOARD_PREDICTION_POST_PROCESS_OBJECT_SVT_SAMPLE_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/container/strong_vector.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"

namespace qcraft {
namespace prediction {

using SvtStateProto = ConflictResolverDebugProto::SvtStateProto;
using SvtNodeProto = ConflictResolverDebugProto::SvtNodeProto;
using SvtEdgeProto = ConflictResolverDebugProto::SvtEdgeProto;
using SvtEdgeCostProto = ConflictResolverDebugProto::SvtEdgeCostProto;
using FeatureCostProto = ConflictResolverDebugProto::FeatureCostProto;

DECLARE_STRONG_VECTOR(SvtNode);
DECLARE_STRONG_VECTOR(SvtEdge);

struct SvtState {
  double s = 0.0;
  double v = 0.0;
  double t = 0.0;

  SvtStateProto ToProto() const {
    SvtStateProto proto;
    proto.set_s(s);
    proto.set_v(v);
    proto.set_t(t);
    return proto;
  }

  std::string DebugString() const {
    return absl::StrFormat("SvtState: s: %f, v: %f, t: %f.", s, v, t);
  }
};

struct SvtEdgeCost {
  std::vector<double> feature_costs;  // Computed by svt cost provider.
  double sum_cost = 0.0;  // prev_edge sum_cost + current edge summed feature
                          // costs. (weighted)

  SvtEdgeCostProto ToProto(absl::Span<const std::string> names) const {
    QCHECK_EQ(names.size(), feature_costs.size());
    SvtEdgeCostProto edge_cost_proto;
    for (int i = 0; i < names.size(); ++i) {
      auto* feature_cost_proto = edge_cost_proto.add_feature_costs();
      FeatureCostProto new_feature_cost;
      new_feature_cost.set_name(names[i]);
      new_feature_cost.set_cost(feature_costs[i]);
      *feature_cost_proto = std::move(new_feature_cost);
    }
    edge_cost_proto.set_sum_cost(sum_cost);
    return edge_cost_proto;
  }

  std::string DebugString() const {
    return absl::StrFormat("Sum_cost: %f, feature costs: %s.", sum_cost,
                           absl::StrJoin(feature_costs, ", "));
  }
};

// Dp search structure. Maybe should move to namespace.
struct DpEdgeInfo {
  // Infos to sample states and compute cost.
  SvtNodeIndex start_index = SvtNodeVector<SvtNodeIndex>::kInvalidIndex;
  SvtEdgeIndex prev_edge_index = SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex;
  double a = 0.0;
  double length = 0.0;  // Length between layers.

  // To be filled when sampling & compute cost.
  double final_v = 0.0;
  double final_t = 0.0;
  SvtEdgeCost cost;

  std::unique_ptr<std::vector<SvtState>> states;

  std::string DebugString() const {
    return absl::StrFormat(
        "Start Svt Node index: %d, prev_edge_index: %d, acc: %f, "
        "feature_costs: %s, sum_cost: %f",
        start_index.value(), prev_edge_index.value(), a,
        absl::StrJoin(cost.feature_costs, ", "), cost.sum_cost);
  }
};

struct SvtNode {
  double s = 0.0;
  double v = 0.0;
  double t = 0.0;
  SvtNodeIndex index = SvtNodeVector<SvtNodeIndex>::kInvalidIndex;

  int s_index;  // layer.

  SvtNodeProto ToProto() const {
    SvtNodeProto proto;
    SvtStateProto state_proto;
    state_proto.set_s(s);
    state_proto.set_v(v);
    state_proto.set_t(t);
    *proto.mutable_state() = std::move(state_proto);
    proto.set_s_index(s_index);
    proto.set_index(index.value());
    return proto;
  }

  std::string DebugString() const {
    return absl::StrFormat(
        "SvtNode index: %d, s: %f, v: %f, t: %f, s(layer) index: %d.",
        index.value(), s, v, t, s_index);
  }
};

struct SvtEdge {
  SvtNodeIndex start_index = SvtNodeVector<SvtNodeIndex>::kInvalidIndex;
  SvtNodeIndex end_index = SvtNodeVector<SvtNodeIndex>::kInvalidIndex;
  double a = 0.0;
  SvtEdgeIndex index = SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex;
  SvtEdgeIndex prev_edge_index = SvtEdgeVector<SvtEdgeIndex>::kInvalidIndex;

  double final_t = 0.0;
  double final_v = 0.0;

  const std::vector<SvtState>* states = nullptr;
  const SvtEdgeCost* edge_cost = nullptr;

  SvtEdgeProto ToProto(absl::Span<const std::string> cost_names) const {
    SvtEdgeProto proto;
    proto.set_index(index.value());
    proto.set_prev_edge_index(prev_edge_index.value());
    proto.set_start_node_index(start_index.value());
    proto.set_end_node_index(end_index.value());
    proto.set_a(a);
    proto.set_final_t(final_t);
    proto.set_final_v(final_v);
    if (edge_cost != nullptr) {
      *proto.mutable_edge_cost() = edge_cost->ToProto(cost_names);
    }
    if (states != nullptr) {
      for (const auto& state : *states) {
        *proto.add_states() = state.ToProto();
      }
    }
    return proto;
  }

  ConflictResolverDebugProto::SimpleSpeedProfile::Edge
  ToSimpleSpeedProfileProto(absl::Span<const std::string> cost_names) const {
    ConflictResolverDebugProto::SimpleSpeedProfile::Edge proto;
    proto.set_index(index.value());
    proto.set_a(a);
    if (states != nullptr && !states->empty()) {
      proto.set_length(states->back().s - states->front().s);
    }
    if (edge_cost != nullptr) {
      *proto.mutable_edge_cost() = edge_cost->ToProto(cost_names);
    }
    return proto;
  }

  std::string DebugString() const {
    return absl::StrFormat(
        "SvtEdge index: %d, acc: %f, start SvtNode index: %d, end SvtNode "
        "index: %d, prev_edge_index: %d.",
        index.value(), a, start_index.value(), end_index.value(),
        prev_edge_index.value());
  }
};

struct VtGrid {
  int v_discrete;  // discrete speed step
  int t_discrete;  // discrete time step

  VtGrid(double v, double t, double v_grid_size, double t_grid_size)
      : v_discrete(std::max(0, RoundToInt(v / v_grid_size))),
        t_discrete(std::clamp(
            RoundToInt(t / t_grid_size), 0,
            CeilToInt(planner::kTrajectoryTimeHorizon / t_grid_size))) {}

  friend bool operator==(const VtGrid& lhs, const VtGrid& rhs) {
    return lhs.v_discrete == rhs.v_discrete && lhs.t_discrete == rhs.t_discrete;
  }

  template <typename H>
  friend H AbslHashValue(H h, const VtGrid& vtg) {
    return H::combine(std::move(h), vtg.v_discrete, vtg.t_discrete);
  }
};

// Used during dp sampling.
std::vector<SvtState> SampleDp(absl::Span<const SvtNode> nodes,
                               const DpEdgeInfo& edge_info, double ds);

// Used during reconstruct trajectory.
std::vector<SvtState> SampleEdge(absl::Span<const SvtNode> nodes,
                                 const SvtEdge& edge, double ds);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_OBJECT_SVT_SAMPLE_H_
