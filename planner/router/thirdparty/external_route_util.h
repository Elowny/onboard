#ifndef ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_UTIL_H_

#include <optional>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/math/vec.h"
#include "onboard/planner/router/thirdparty/external_route_request.h"
namespace qcraft::planner::route {

// All coordinates are raw GPS. NOT tencent.
absl::StatusOr<ExternalRouteResult> GetTencentRouteFromMultiStops(
    const std::optional<Vec2d>& pos,
    const MultipleStopsRequestProto& multi_stops);

absl::StatusOr<ExternalRouteResult> GetTencentRouteFromDestinations(
    const std::optional<Vec2d>& pos,
    const google::protobuf::RepeatedPtrField<RoutingDestinationProto>&
        destinations);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_UTIL_H_
