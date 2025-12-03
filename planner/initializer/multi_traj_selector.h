#ifndef ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_
#define ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct SingleTrajDebugInfo {
  std::vector<MotionEdgeIndex> terminated_edge_idxes;
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs;
  std::vector<double> top_k_total_costs;
  std::vector<MotionEdgeIndex> top_k_edges;
  OpenPQ astar_open_pq;
  CloseMap astar_close_map;
};

struct SingleTrajInfo {
  // Different trajectory should be determined by different grouping of leading
  // trajectories based on lane change state. Need logic to choose leading
  // trajectories while changing lanes.
  std::vector<std::string> leading_trajs;  // Leading objects' traj_ids.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  MotionEdgeIndex last_edge_index;
  MotionEdgeVector<MotionSearchOutput::SearchCost> search_costs;
  double total_cost;
  std::unique_ptr<MotionGraph> motion_graph;
  std::unique_ptr<RefSpeedTable> ref_speed_table;
  std::unique_ptr<CostProviderBase> cost_provider;
  IgnoreTrajMap ignored_trajs;
  SingleTrajDebugInfo debug_info;
  int astar_end_node_index;
};

absl::StatusOr<int> EvaluateMultiTrajs(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<SingleTrajInfo>& multi_trajs,
    const VehicleGeometryParamsProto& vehicle_geom, bool eval_safety,
    LaneChangeStyle lc_style, absl::Duration path_look_ahead_duration,
    absl::flat_hash_set<std::string>* follower_set, double* follower_max_decel,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    LaneChangeSafetyDebugProto* lane_change_safety_debug_proto,
    ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_MULTI_TRAJ_SELECTOR_H_
