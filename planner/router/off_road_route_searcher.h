#ifndef ONBOARD_PLANNER_ROUTER_OFF_ROAD_ROUTE_SEARCHER_H_
#define ONBOARD_PLANNER_ROUTER_OFF_ROAD_ROUTE_SEARCHER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

std::vector<mapping::LanePoint> FindCrossingLanePointsFromPose(
    const mapping::v2::SemanticMapManager& smm, const route::MapIndex& index,
    const CoordinateConverter& cc, const PoseProto& pose, double radius);

struct OffRoadRouteSearcherOutput {
  mapping::LanePoint link_lane_point;
  RouteSections route_sections;
  CompositeLanePath on_road_route_lane_path;
  int next_stop_index;
};

absl::StatusOr<OffRoadRouteSearcherOutput> SearchForRouteFromOffRoad(
    const route::RoutingRequestContext& context, const CoordinateConverter& cc,
    const PoseProto& pose, const MultipleStopsRequest& route_request,
    double search_radius, route::RouteCore* route_core);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_OFF_ROAD_ROUTE_SEARCHER_H_
