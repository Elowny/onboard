#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_V2_TRAFFIC_GAP_FINDER_IMPL_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_V2_TRAFFIC_GAP_FINDER_IMPL_H_

#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"

namespace qcraft::planner {

struct TrafficGap {
  const PlannerObject* leader_object;
  const PlannerObject* follower_object;
  double lon_offset;
  double target_v;
  double align_time;
  double gap_length;
  double gap_cap_v;
};

struct GapAlignInfo {
  double align_time;
  double align_s;
  double align_v;
};

absl::StatusOr<GapAlignInfo> ComputeGapAlignInfo(double lon_offset,
                                                 double target_v, double cap_v,
                                                 double ego_v, bool gap_behind,
                                                 bool gap_aligned);

std::pair<double, double> FindNearestLeadingObject(
    const FrenetFrame& frenet_frame, const FrenetBox& ego_frenet_box,
    double ego_v, const SpacetimeTrajectoryManager& st_traj_mgr);

std::vector<TrafficGap> FindCandidateTrafficGapsOnLanePath(
    const FrenetFrame& target_frenet_frame, double cap_v, double cap_offset,
    const FrenetBox& ego_frenet_box, double ego_v, double ego_a,
    const SpacetimeTrajectoryManager& st_traj_mgr, double max_reach_length,
    double speed_limit);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_V2_TRAFFIC_GAP_FINDER_IMPL_H_
