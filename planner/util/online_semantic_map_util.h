#ifndef ONBOARD_PLANNER_UTIL_ONLINE_SEMANTIC_MAP_UTIL_H_
#define ONBOARD_PLANNER_UTIL_ONLINE_SEMANTIC_MAP_UTIL_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

absl::StatusOr<std::vector<Vec2d>> FindClosestLanePathPoints(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map,
    const FrenetFrame& frenet_frame, const Vec2d& ego_xy,
    double valid_lane_length, double max_lat_offset_thres,
    double avg_lat_offset_thres);

std::vector<mapping::ElementId> ComputeStartLanesByPos(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_xy,
    double proj_thres, double max_backward_thres);

absl::StatusOr<std::vector<mapping::ElementId>> FindCloseOnlineLaneIds(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map,
    const FrenetFrame& frenet_frame, double valid_lane_length,
    double max_check_length, double max_lat_offset_thres,
    double avg_lat_offset_thres);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_ONLINE_SEMANTIC_MAP_UTIL_H_
