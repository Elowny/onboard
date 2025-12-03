#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_GENERATOR_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_GENERATOR_H_

#include "absl/status/statusor.h"

#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

absl::StatusOr<RoutingResultProto> InitRoute(
    const route::RoutingRequestContext& context, const CoordinateConverter& cc,
    const route::RouteRestrictDistrict& route_restrict_district,
    const PlannerRoutingRequestProto::InitRouteRequest& request, bool is_bus,
    route::RouteCore* route_core);

absl::StatusOr<RoutingResultProto> GenerateRoutingResultProto(
    const route::RoutingRequestContext& context, const CoordinateConverter& cc,
    const route::RouteRestrictDistrict& route_restrict_district,
    const PlannerRoutingRequestProto* request, bool is_bus,
    route::RouteCore* route_core);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_GENERATOR_H_
