#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_PORTAL_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_PORTAL_UTIL_H_

#include <string>

#include "absl/status/statusor.h"
#include "common/proto/drive_mission.pb.h"
#include "onboard/proto/q_command.pb.h"

/// Used for external project. It is not used in routing module internal.
namespace qcraft::planner {
absl::StatusOr<QCommand::DriveRoute> ConvertToDriverRoute(
    const std::string& pnc_arg);

absl::StatusOr<RoutingRequestProto> ConverterToRoutingRequestProto(
    const QCommand::DriveRoute& driver_route);

absl::StatusOr<RoutingRequestProto> ConverterToSimSnippet(
    const RoutingRequestProto& routing_request_proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_PORTAL_UTIL_H_
