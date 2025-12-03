#ifndef ONBOARD_PLANNER_ROUTER_NOA_SD_ROUTE_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_NOA_SD_ROUTE_UTIL_H_
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route::noa {

absl::Status FillRoutingResultBySdRoute(const SDRouteProto& sd_route,
                                        RoutingResultProto* routing_result);

absl::Status InitSdRouteMatchResult(
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto,
    SdRouteMatchState* /*In&Out*/ sd_route_match_state);

absl::StatusOr<SdRouteMatchState> BuildSdRouteMatchState(
    const std::shared_ptr<const SDRouteProto>& sd_route,
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto);

absl::Status FillRouteManagerState(
    const std::shared_ptr<const SDRouteProto>& sd_route,
    const std::shared_ptr<const SdRouteMatchResultProto>&
        sd_route_match_result_proto,
    RouteManagerState* rms);

double DistanceToSdMatchPoint(const CoordinateConverter& cc,
                              const SdRouteMatchState& state,
                              const SdRouteMatchPoint& sd_match_point);

}  // namespace qcraft::planner::route::noa

#endif  // ONBOARD_PLANNER_ROUTER_NOA_SD_ROUTE_UTIL_H_
