#include "onboard/planner/initializer/motion_search_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <limits>
#include <string>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

ApolloTrajectoryPointProto MotionState2TrajPoint(
    const MotionState& motion_state, double s, double current_t) {
  PathPoint point;
  point.set_x(motion_state.xy.x());
  point.set_y(motion_state.xy.y());
  point.set_z(0);
  point.set_theta(motion_state.h);
  point.set_kappa(motion_state.k);
  point.set_s(s);

  ApolloTrajectoryPointProto traj_point;
  *(traj_point.mutable_path_point()) = point;
  traj_point.set_v(motion_state.v);
  traj_point.set_a(motion_state.a);
  traj_point.set_relative_time(current_t);

  return traj_point;
}

std::vector<ApolloTrajectoryPointProto> ResampleTrajectoryPoints(
    int traj_steps, const std::vector<const MotionForm*>& motions) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(traj_steps);

  double s = 0.0;
  auto motion_it = motions.begin();
  double current_motion_start_time = 0,
         current_motion_end_time = (*motion_it)->duration();
  traj_points.emplace_back(MotionState2TrajPoint(
      (*motion_it)->GetStartMotionState(), /*s=*/0, /*current_t=*/0));
  for (int i = 1; i < traj_steps; ++i) {
    const double current_t = i * kTrajectoryTimeStep;
    if (current_t > current_motion_end_time) {
      ++motion_it;
      // If we reach the end of the trajectory.
      if (motion_it == motions.end()) {
        break;
      }
      current_motion_start_time = current_motion_end_time;
      current_motion_end_time += (*motion_it)->duration();
    }
    const auto motion_state =
        (*motion_it)->State(current_t - current_motion_start_time);

    s += motion_state.xy.DistanceTo(Vec2d(traj_points[i - 1].path_point().x(),
                                          traj_points[i - 1].path_point().y()));
    traj_points.emplace_back(MotionState2TrajPoint(motion_state, s, current_t));
  }

  return traj_points;
}

std::vector<ApolloTrajectoryPointProto> ConstructStationaryTraj(
    int traj_steps, const MotionState& sdc_motion) {
  const MotionState state{.xy = sdc_motion.xy,
                          .h = sdc_motion.h,
                          .k = sdc_motion.k,
                          .v = 0.0,
                          .a = 0.0};

  std::vector<ApolloTrajectoryPointProto> stationary_traj;
  stationary_traj.reserve(traj_steps);
  for (int i = 0; i < traj_steps; ++i) {
    const double current_t = i * kTrajectoryTimeStep;
    stationary_traj.emplace_back(
        MotionState2TrajPoint(state, /*s=*/0.0, current_t));
  }

  return stationary_traj;
}

std::vector<ApolloTrajectoryPointProto> ConstructTrajFromLastEdge(
    int traj_steps, const MotionGraph& motion_graph,
    MotionEdgeIndex last_edge_index) {
  std::vector<const MotionForm*> motions;
  QCHECK_GE(last_edge_index.value(), 0);
  while (last_edge_index != MotionEdgeVector<MotionEdge>::kInvalidIndex) {
    const auto& motion_edge = motion_graph.GetMotionEdge(last_edge_index);
    motions.emplace_back(motion_edge.motion);
    last_edge_index = motion_edge.prev_edge;
  }
  std::reverse(motions.begin(), motions.end());
  return ResampleTrajectoryPoints(traj_steps, motions);
}

MotionState PrepareStartMotionNode(
    const GeometryGraph& geometry,
    const std::vector<GeometryNodeIndex>& first_layer,
    const ApolloTrajectoryPointProto& start_point,
    int* start_node_idx_on_first_layer) {
  for (int i = first_layer.size() - 1; i >= 0; --i) {
    // Reverse look up because the first node is always reachable.
    const auto& node = geometry.GetNode(first_layer[i]);
    if (node.reachable) {
      *start_node_idx_on_first_layer = i;
      return MotionState{.xy = node.xy,
                         .h = start_point.path_point().theta(),
                         .k = node.k,
                         .t = 0.0,
                         .v = start_point.v(),
                         .a = start_point.a()};
    }
  }
  return MotionState();  // Should not reach here.
}

double GetLeadingObjectsEndMinS(const SpacetimeTrajectoryManager& st_mgr,
                                const DrivePassage& drive_passage,
                                const std::vector<std::string>& leading_objs,
                                double ego_front_to_ra) {
  double min_s = std::numeric_limits<double>::max();
  for (const auto& lead_obj : leading_objs) {
    const auto states = st_mgr.FindTrajectoryById(lead_obj)->states();
    const auto fbox = drive_passage.QueryFrenetBoxAt(states.back().box);
    if (!fbox.ok()) {
      continue;
    }
    min_s = std::min(min_s, fbox->s_min);
  }
  return min_s - ego_front_to_ra;
}

}  // namespace qcraft::planner
