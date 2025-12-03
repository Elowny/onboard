#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/decision/traffic_gap_result.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"

namespace qcraft::planner {

struct TrafficGap {
  absl::Span<const SpacetimeObjectTrajectory* const> leader_trajectories;
  absl::Span<const SpacetimeObjectTrajectory* const> follower_trajectories;
  double s_start;
  double s_end;
};

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, const FrenetBox& ego_frenet_box,
    const SpacetimeTrajectoryManager& st_traj_mgr);

absl::StatusOr<TrafficGapResult> EvaluateAndTakeBestTrafficGap(
    absl::Span<const TrafficGap> candidate_gaps,
    const FrenetBox& ego_frenet_box, double ego_init_v);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_FINDER_H_
