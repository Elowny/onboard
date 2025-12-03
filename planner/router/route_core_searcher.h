#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_CORE_SEARCHER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_CORE_SEARCHER_H_

#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/spatial_search_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft::planner::route {
absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchForRouteCorePathFromGlobalPose(
    const RoutingRequestContext& route_context,
    const CoordinateConverter& coordinate_converter,
    const RoutingRequestProto& routing_request, const Vec2d& global_pose,
    const RouteCorePrecondition& route_pre,
    /*global theta*/ double heading, RouteCore* route_core);

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchForRouteCorePathToRoutingRequest(
    const RoutingRequestContext& route_context,
    const RoutingRequestProto& routing_request,
    const std::vector<mapping::PointToLane>& origin,
    const RouteCorePrecondition& route_pre, RouteCore* route_core);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_CORE_SEARCHER_H_
