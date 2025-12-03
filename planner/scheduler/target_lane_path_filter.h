#ifndef ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_
#define ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_

#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const RouteNaviInfo& route_navi_info,
    const mapping::LanePath& last_target_lane_path,
    const ApolloTrajectoryPointProto& plan_start_point,
    const mapping::LanePath& preferred_lane_path,
    const mapping::LanePath& lc_preview_lane_path,
    std::vector<LanePathInfo>* mutable_lp_infos);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_
