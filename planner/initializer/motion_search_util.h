#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_

#include <string>
#include <vector>

#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

inline constexpr double kEpsilon = 1e-6;

ApolloTrajectoryPointProto MotionState2TrajPoint(
    const MotionState& motion_state, double s, double current_t);

std::vector<ApolloTrajectoryPointProto> ResampleTrajectoryPoints(
    int traj_steps, const std::vector<const MotionForm*>& motions);

std::vector<ApolloTrajectoryPointProto> ConstructStationaryTraj(
    int traj_steps, const MotionState& sdc_motion);

std::vector<ApolloTrajectoryPointProto> ConstructTrajFromLastEdge(
    int traj_steps, const MotionGraph& motion_graph,
    MotionEdgeIndex last_edge_index);

MotionState PrepareStartMotionNode(
    const GeometryGraph& geometry,
    const std::vector<GeometryNodeIndex>& first_layer,
    const ApolloTrajectoryPointProto& start_point,
    int* start_node_idx_on_first_layer);

double GetLeadingObjectsEndMinS(const SpacetimeTrajectoryManager& st_mgr,
                                const DrivePassage& drive_passage,
                                const std::vector<std::string>& leading_objs,
                                double sdc_length);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_
