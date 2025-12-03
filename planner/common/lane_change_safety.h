#ifndef ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_
#define ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/time/time.h"

#include "onboard/math/frenet_frame.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

// TODO(jiayu): Move unsafe_object_id to lane change safety debug proto.
absl::Status CheckLaneChangeSafety(
    const std::vector<ApolloTrajectoryPointProto>& ego_traj_pts,
    const FrenetFrame& target_frenet_frame, double speed_limit,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geom, LaneChangeStyle lc_style,
    absl::Duration path_look_ahead_duration,
    absl::flat_hash_set<std::string>* follower_set, double* follower_max_decel,
    std::string* unsafe_object_id, LaneChangeSafetyDebugProto* debug_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_H_
