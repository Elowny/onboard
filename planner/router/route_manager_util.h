#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_UTIL_H_

#include <functional>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {
absl::StatusOr<int> FindNextDestinationIndexViaLanePoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MultipleStopsRequest& multi_stops_request,
    const std::function<bool(const RouteSections&)>& point_on_route_func,
    route::RouteCore* route_core);

absl::StatusOr<mapping::LanePoint> FindLanePointFromPose(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex& map_index,
    const CoordinateConverter& coordinate_converter,
    const RouteParamProto& route_param_proto, const PoseProto& pose);

absl::StatusOr<MultipleStopsRequest> RecoverRoutingRequest(
    const RoutingRequestProto& log_routing_request,
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index);

absl::StatusOr<RoutingRequestProto>
GenerateAndCheckRoutingRequestProtoToNextStop(
    const mapping::v2::SemanticMapManager& smm,
    const route::MapIndex* map_index, const RouteParamProto& route_param_proto,
    const MultipleStopsRequest& multi_stops, const Vec2d& car_global,
    double heading, int next_destination_index);

absl::StatusOr<RouteProto> TrackAlternateRoute(
    const mapping::v2::SemanticMapManager& smm, const RouteProto& last_route,
    mapping::LanePoint cur_match_point, double travel_dist);

absl::StatusOr<std::pair<RouteSections, RouteSections>>
BackWardExtendSectionsFromCurrentAlongRoute(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& global_sections,
    mapping::LanePoint start_point, int start_sec_idx, double extend_length);

absl::StatusOr<RouteNaviInfo> CalcCurRouteNaviInfo(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& global_sections,
    const RouteSectionSequenceProto& cur_sections,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    mapping::LanePoint start_lp, double extend_length, double preview_dist);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_MANAGER_UTIL_H_
