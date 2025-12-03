#ifndef ONBOARD_PLANNER_ROUTER_REQUEST_ROUTE_REQUEST_CONTEXT_H_
#define ONBOARD_PLANNER_ROUTER_REQUEST_ROUTE_REQUEST_CONTEXT_H_
#include <optional>

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
struct RoutingRequestContext {
  const mapping::v2::SemanticMapManager* semantic_map_manager = nullptr;
  const route::MapIndex* map_index = nullptr;
  const RouteParamProto* route_param_proto = nullptr;
  int64_t request_id = -1;
  std::optional<Vec2d> pred_trajectory_point;  // smooth
};

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_REQUEST_ROUTE_REQUEST_CONTEXT_H_
