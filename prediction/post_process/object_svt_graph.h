#ifndef ONBOARD_PREDICTION_POST_PROCESS_OBJECT_SVT_GRAPH_H_
#define ONBOARD_PREDICTION_POST_PROCESS_OBJECT_SVT_GRAPH_H_

// IWYU pragma: no_include "absl/hash/hash.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/post_process/object_svt_sample.h"
#include "onboard/prediction/post_process/svt_cost_provider.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

namespace prediction {

using VtOptimal = absl::flat_hash_map<VtGrid, DpEdgeInfo>;

// TODO(changqing): Move to input file.
struct ObjectSvtGraphInput {
  std::string traj_id;
  const ObjectProto* object_proto = nullptr;
  const std::vector<double>* stoplines = nullptr;
  const std::vector<double>* stationary_objects = nullptr;
  const planner::SpeedVector* ref_speed = nullptr;
  const SvtCostProvider* cost_provider = nullptr;
  const ConflictResolutionConfigProto::ConflictResolverConfig* general_config =
      nullptr;
  const ConflictResolutionConfigProto::ObjectConflictResolverConfig*
      object_config = nullptr;
};

class ObjectSvtGraph {
 public:
  explicit ObjectSvtGraph(const ObjectSvtGraphInput& input,
                          ThreadPool* thread_pool);

  absl::StatusOr<planner::SpeedVector> Search(bool visual_to_debug);

  inline int layers_size() const { return s_knots_.size(); }
  void ToProto(ConflictResolverDebugProto* debug_proto) const;

  // TODO(changqing): Create DebugString().

 private:
  // AddNode
  SvtNodeIndex AddNode(double s, double v, double t, int s_layer_index);

  // AddEdge
  SvtEdgeIndex AddEdge(const SvtNodeIndex& start_idx,
                       const SvtNodeIndex& end_idx, double a,
                       const SvtEdgeIndex& prev_edge_index,
                       const SvtEdgeCost& edge_cost,
                       const std::vector<SvtState>* states, double final_t,
                       double final_v);
  ThreadPool* thread_pool_;
  const ObjectProto* object_proto_ = nullptr;
  const SvtCostProvider* cost_provider_ = nullptr;
  const planner::SpeedVector* ref_speed_ = nullptr;
  const ConflictResolutionConfigProto::ConflictResolverConfig* general_config_;
  const ConflictResolutionConfigProto::ObjectConflictResolverConfig*
      object_config_;

  std::string traj_id_;
  std::unique_ptr<planner::SpeedVector> final_speed_vector_ = nullptr;
  std::vector<SvtEdgeIndex> edges_from_start_;

  // Set virtual discretization grid.
  double min_unit_s_;
  std::vector<double> s_knots_;
  std::vector<double> a_range_;  // Object acceleration sampling range.

  std::vector<VtOptimal>
      vt_optimal_layers_;  // To be built and updated layer by layer.

  // Debug: record edges/related nodes if they have been considered as an
  // optimal edge/node.
  std::vector<SvtNode> nodes_;
  std::vector<SvtEdge> edges_;
  SvtEdgeVector<SvtEdgeCost> search_costs_;  // Each edge's cost_so_far.
  std::vector<std::vector<SvtNodeIndex>> nodes_layers_;
  std::vector<std::unique_ptr<planner::SpeedVector>> candidate_speed_vectors_;
  std::vector<std::vector<SvtEdgeIndex>> candidate_edges_from_start_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_OBJECT_SVT_GRAPH_H_
