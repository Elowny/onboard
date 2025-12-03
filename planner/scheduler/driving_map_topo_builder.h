#ifndef ONBOARD_PLANNER_SCHEDULER_DRIVING_MAP_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_DRIVING_MAP_BUILDER_H_

#include "absl/status/statusor.h"

#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft::planner {

absl::StatusOr<DrivingMapTopo> BuildDrivingMapByRouteOnOfflineMap(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& sections_from_start);

absl::StatusOr<DrivingMapTopo> BuildDrivingMapByOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_pos);

absl::StatusOr<DrivingMapTopo>
BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
    const PlannerSemanticMapManager& psmm, const Vec2d& pos, double heading,
    double radius, double max_forward_length, double max_heading_diff,
    bool allow_virtual);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_DRIVING_MAP_BUILDER_H_
