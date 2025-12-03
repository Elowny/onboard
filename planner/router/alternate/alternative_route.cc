#include "onboard/planner/router/alternate/alternative_route.h"

#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/alternate/alternative_route_util.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_core_searcher.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner::route {
namespace {
absl::Status CheckAlternateRoutePrecondition(
    const AlternateRouteInput& alter_input) {
  if (alter_input.route_context.semantic_map_manager == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot find alternate route, semantic_map_manager cannot be null.");
  }
  if (alter_input.routing_request == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot find alternate route, routing_request cannot be null.");
  }
  if (alter_input.primary_sections == nullptr) {
    return absl::FailedPreconditionError(
        "Cannot find alternate route, primary route cannot be null.");
  }
  return absl::OkStatus();
}

absl::StatusOr<AlternateRouteOutput> GenerateAlternateRoute(
    const mapping::v2::SemanticMapManager& /*smm*/,
    const MultipleStopsRequest& multi_stops, RoutingResultProto response) {
  RouteProto route;
  *route.mutable_lane_path() = response.lane_path();
  *route.mutable_route_section_sequence() = response.route_section_sequence();
  *route.mutable_routing_request() = response.routing_request();
  route.mutable_avoid_lanes()->Reserve(multi_stops.avoid_lanes().size());
  route.set_update_id(response.update_id());
  for (const auto& avoid_lane : multi_stops.avoid_lanes()) {
    route.add_avoid_lanes(avoid_lane.value());
  }
  return AlternateRouteOutput{.routing_result_proto = std::move(response),
                              .route = route,
                              .route_from_current = route};
}
}  // namespace

absl::StatusOr<AlternateRouteOutput> FindAlternateRoute(
    const AlternateRouteInput& alter_input, RouteCore* route_core) {
  RETURN_IF_ERROR(CheckAlternateRoutePrecondition(alter_input));

  const auto& smm = *alter_input.route_context.semantic_map_manager;

  SMM_LANE_PROTO_OR_RETURN(
      lane_proto, smm, alter_input.origin.lane_id(),
      absl::NotFoundError(absl::StrFormat("Cannot find element id: %d",
                                          alter_input.origin.lane_id())));

  const mapping::PointToLane origin_ptl = {
      .lane_proto = lane_proto,
      .dist = 0.0,
      .fraction = alter_input.origin.fraction()};

  absl::flat_hash_set<mapping::ElementId> blacklist(
      alter_input.routing_request->avoid_lanes().begin(),
      alter_input.routing_request->avoid_lanes().end());

  RouteCorePrecondition route_pre = {
      .is_bus = alter_input.is_bus,
      .use_time = true,
      .route_restrict_district = route::RouteRestrictDistrict{
          .avoid_lanes = std::move(blacklist),
          .extra_sections_cost =
              (alter_input.primary_sections != nullptr)
                  ? GenerateDecreasingCostSeq(
                        smm, *alter_input.primary_sections,
                        alter_input.look_ahead_dist, alter_input.cost_per_meter)
                  : absl::flat_hash_map<mapping::SectionId, double>()}};

  ASSIGN_OR_RETURN(const auto sections_lanes_pair,
                   SearchForRouteCorePathToRoutingRequest(
                       alter_input.route_context, *alter_input.routing_request,
                       {origin_ptl}, route_pre, route_core));

  RoutingResultProto routing_result_proto;

  sections_lanes_pair.second.ToProto(routing_result_proto.mutable_lane_path());
  sections_lanes_pair.first.ToProto(
      routing_result_proto.mutable_route_section_sequence());
  routing_result_proto.set_success(true);
  routing_result_proto.set_update_id(alter_input.route_context.request_id);
  *routing_result_proto.mutable_routing_request() =
      *alter_input.routing_request;
  return GenerateAlternateRoute(*alter_input.route_context.semantic_map_manager,
                                *alter_input.multi_stops,
                                std::move(routing_result_proto));
}
}  // namespace qcraft::planner::route
