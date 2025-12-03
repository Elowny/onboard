#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_V2_TRAFFIC_GAP_FINDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_V2_TRAFFIC_GAP_FINDER_H_

#include <string>

#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/decision/traffic_gap_result.h"
#include "onboard/planner/decision/traffic_gap_v2/proto/traffic_gap.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"

namespace qcraft::planner {

TrafficGapResult FindBestTrafficGapOnRouteTarget(
    const FrenetFrame& cur_frenet_frame, const FrenetBox& cur_ego_fbox,
    const SpacetimeTrajectoryManager& cur_st_traj_mgr,
    const FrenetFrame& target_frenet_frame, const FrenetBox& target_ego_fbox,
    const SpacetimeTrajectoryManager& target_st_traj_mgr, double ego_v,
    double ego_a, double max_reach_length, double speed_limit,
    const std::string& prev_leader, const std::string& prev_follower,
    TrafficGapDebugProto* debug_info);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_GAP_V2_TRAFFIC_GAP_FINDER_H_
