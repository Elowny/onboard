#ifndef ONBOARD_PLANNER_ROUTER_ALTERNATE_ALTERNATIVE_ROUTE_H_
#define ONBOARD_PLANNER_ROUTER_ALTERNATE_ALTERNATIVE_ROUTE_H_

#include "absl/status/statusor.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/lane_point.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {

struct AlternateRouteInput {
  mapping::LanePoint origin;
  bool is_bus = false;
  double look_ahead_dist = 0.0;
  double cost_per_meter = 0.0;
  const RouteSectionSequenceProto* primary_sections = nullptr;
  RoutingRequestContext route_context;
  const RoutingRequestProto* routing_request = nullptr;
  const MultipleStopsRequest* multi_stops = nullptr;
};

struct AlternateRouteOutput {
  RoutingResultProto routing_result_proto;
  RouteProto route;
  RouteProto route_from_current;
};

absl::StatusOr<AlternateRouteOutput> FindAlternateRoute(
    const AlternateRouteInput& alter_input, RouteCore* route_core);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ALTERNATE_ALTERNATIVE_ROUTE_H_
