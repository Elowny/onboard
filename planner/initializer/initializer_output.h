#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
namespace qcraft::planner {

struct MotionSearchOutput {
  absl::Status result_status = absl::OkStatus();

  enum class SearchAlgorithm {
    kDP = 0,
    kASTAR = 1,
  };
  SearchAlgorithm search_algorithm;

  // Only has value on performing lane change and safety confirmed.
  absl::flat_hash_set<std::string> follower_set;
  double follower_max_decel = 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids;

  bool is_lc_pause = false;
  // TODO(lidong): Add returning a list of motion forms and Edge indices.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  MotionEdgeIndex best_last_edge_index;
  int astar_end_node_index;

  struct SearchCost {
    std::vector<double> feature_cost;  // Accumulated feature cost
    double cost_to_come = 0.0;
  };

  MotionEdgeVector<SearchCost> search_costs;
  double min_cost;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;

  std::unique_ptr<MotionGraph> motion_graph;

  std::unique_ptr<RefSpeedTable> ref_speed_table;

  std::unique_ptr<CostProviderBase> cost_provider;

  // Debug only (FLAGS_planner_initializer_debug_level >= 1)
  // Optimal trajectory (among all the leading objs)
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
  std::vector<double> top_k_total_costs;
  std::vector<MotionEdgeIndex> top_k_edges;
  OpenPQ astar_open_pq;
  CloseMap astar_close_map;

  // Lane change safety debug.
  LaneChangeSafetyDebugProto lane_change_safety_debug_proto;
  // Trajectories with multiple leading objects.
  struct MultiTrajCandidate {
    std::vector<ApolloTrajectoryPointProto> trajectory;
    std::vector<std::string> leading_traj_ids;
    double total_cost;
    MotionEdgeIndex last_edge_index;
    std::vector<double> feature_costs;
    double final_cost;
    IgnoreTrajMap ignored_trajs;
  };
  std::vector<MultiTrajCandidate> multi_traj_candidates;

  // Data Dumping only
  struct IsFilteredReasons {
    bool is_out_of_bound = false;
    bool is_violating_stop_constraint = false;
    bool is_dynamic_collision = false;
    bool is_violating_leading_objects = false;
  };
  struct TrajectoryEvaluationDumping {
    double weighted_total_cost;  // weighted by onboard initializer's weights
    std::vector<double> dumped_weights;  // onboard initializer's weights
    std::vector<double> feature_costs;   // unweighted
    std::vector<ApolloTrajectoryPointProto> traj;
    IsFilteredReasons is_filtered_reasons;
  };
  TrajectoryEvaluationDumping expert_evaluation;
  std::vector<TrajectoryEvaluationDumping> candidates_evaluation;
};

struct InitializerOutput {
  absl::flat_hash_set<std::string> follower_set;
  double follower_max_decel = 0.0;
  bool is_lc_pause = false;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  InitializerStateProto initializer_state;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs;
};

struct ReferenceLineSearcherOutput {
  struct NodeInfo {
    double min_cost = std::numeric_limits<double>::infinity();
    GeometryEdgeIndex outgoing_edge_idx =
        GeometryEdgeVector<GeometryEdge>::kInvalidIndex;
    GeometryNodeIndex prev_node_idx =
        GeometryNodeVector<GeometryNode>::kInvalidIndex;
  };

  struct EdgeInfo {
    std::vector<double> feature_costs;
    double sum_cost = 0.0;
  };

  std::vector<GeometryNodeIndex> nodes_list;
  std::vector<GeometryEdgeIndex> edges_list;
  std::vector<Vec2d> ref_line_points;
  double total_cost = 0.0;
  std::vector<double> feature_costs;

  std::unique_ptr<RefLineCostProvider> ptr_cost_provider;
  GeometryEdgeVector<EdgeInfo> cost_edges;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_
