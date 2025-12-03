#ifndef ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_UTIL_H_

#include <set>
#include <vector>

#include "absl/hash/hash.h"  // IWYU pragma: keep
#include "absl/synchronization/mutex.h"

#include "onboard/math/vec.h"
#include "onboard/planner/initializer/astar_motion_searcher_defs.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_graph_cache.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

int CreateNodeKey(const double v, const double t,
                  const GeometryNodeIndex& geom_index);

AstarNode CreateNode(const MotionState& state, const int key,
                     const GeometryNodeIndex& geom_index);

double ComputeCaptainNetInspiredHeuristicCost(
    const double traj_horizon, const AstarNode& node,
    const std::vector<ApolloTrajectoryPointProto>&
        captain_net_traj_points_traj_points);

double ComputeReferenceLineInspiredHeuristicCost(
    const AstarNode& node, const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table);

double ComputeHeuristicCost(
    const double traj_horizon, const AstarNode& node,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    bool is_run_model_l4);

std::set<double> SampleMotionForms(
    const double traj_horizon, const AstarNode& astar_node,
    const GeometryEdge& geom_edge,
    const MotionConstraintParamsProto& motion_constraint_params,
    const bool sample_const_v);

void ProcessMotionFormsWithCache(
    const double traj_horizon, const AstarNode& parent_node,
    const GeometryEdge& geom_edge, const std::set<double>& acc_samples,
    const CostProviderBase& cost_provider,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* ref_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    OpenPQ* open_pq, OpenMap* open_map, const CloseMap& close_map,
    const AstarGraphCache& cost_cache, std::vector<AstarCacheInfo>* new_cache,
    const bool is_run_model_l4, absl::Mutex* my_mutex);

void ProcessMotionFormsWithoutCache(
    const double traj_horizon, const AstarNode& parent_node,
    const GeometryEdge& geom_edge, const std::set<double>& acc_samples,
    const CostProviderBase& cost_provider,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* ref_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    OpenPQ* open_pq, OpenMap* open_map, const CloseMap& close_map,
    const bool is_run_model_l4, absl::Mutex* my_mutex);

AstarNode SearchFailTryStationaryMotion(
    const double traj_horizon, const CostProviderBase& cost_provider,
    const AstarNode& parent_node,
    const std::vector<Vec2d>& reference_line_points,
    const RefSpeedTable* reference_speed_table,
    const std::vector<ApolloTrajectoryPointProto>& captain_net_traj_points,
    const bool is_run_model_l4);

bool SatisfyEndCondition(const double traj_horizon, const AstarNode& node);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_UTIL_H_
