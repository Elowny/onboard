#ifndef ONBOARD_PLANNER_ROUTER_INTERFACE_GENERATE_ROUTE_H_
#define ONBOARD_PLANNER_ROUTER_INTERFACE_GENERATE_ROUTE_H_

#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
//<< -----------support v1 legacy medhod begin------------
absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchRouteResultsBetweenLanePoints(
    const mapping::SemanticMapManager& smm, const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus);

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenLanePoints(
    const mapping::SemanticMapManager& smm, const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus);

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenVec3ds(
    const mapping::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const Vec3d& origin, const Vec3d& destination,
    bool is_bus);
// -----------support v1 legacy medhod end------------>

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchRouteResultsBetweenLanePoints(
    const mapping::v2::SemanticMapManager& smm,
    const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus);

absl::StatusOr<std::pair<RouteSections, CompositeLanePath>>
SearchRouteResultsBetweenDestinations(
    const mapping::v2::SemanticMapManager& smm, const MapIndex* map_index,
    const google::protobuf::RepeatedPtrField<RoutingDestinationProto>&
        destinations,
    const RouteRestrictDistrict& route_restrict, bool is_bus);

// -------------- Search for CompositeLanePath --------------
absl::StatusOr<std::vector<CompositeLanePath>> GenerateRoutePathFromStops(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    absl::Span<const mapping::LanePoint> destinations,
    absl::Span<const int> stops_index, bool infinite_loop, bool is_bus);

absl::StatusOr<std::vector<CompositeLanePath>>
GenerateRoutePathByRoutingRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const RoutingRequestProto& routing_request_proto,
    bool is_bus);

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenLanePoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const mapping::LanePoint& origin,
    absl::Span<const mapping::LanePoint> destinations, bool is_bus);

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenVec3ds(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const Vec3d& origin, const Vec3d& destination,
    bool is_bus);

absl::StatusOr<CompositeLanePath> SearchLanePathBetweenGeoPoints(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, const mapping::GeoPointProto& origin,
    const mapping::GeoPointProto& destination, bool is_bus);

// Only search routing_request destinations
absl::StatusOr<CompositeLanePath> SearchRoutingRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const MapIndex& map_index,
    const RoutingRequestProto& routing_request, const PoseProto& pose,
    bool is_bus = false, bool use_time = true);

// -------------- Search for RouteSections --------------
absl::StatusOr<std::vector<RouteSections>> GenerateRouteSectionsFromStops(
    const mapping::v2::SemanticMapManager& smm,
    absl::Span<const mapping::LanePoint> destinations,
    absl::Span<const int> stops_index, bool infinite_loop, bool is_bus);

absl::StatusOr<RouteSections> GenerateTotalRouteSections(
    absl::Span<const RouteSections> vec_route_sections);

// --------------------- Utils --------------------------------
absl::StatusOr<std::pair<std::vector<mapping::LanePoint>, std::vector<int>>>
GetDestinations(const mapping::v2::SemanticMapManager& semantic_map_manager,
                const MapIndex* map_index,
                const MultipleStopsRequest& multiple_stops);

/// @brief UNIT = rad
absl::StatusOr<mapping::LanePoint> GetNearestLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const MapIndex* map_index, double x, double y, double z);

absl::StatusOr<PathStatInfoProto> StatPathInfo(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CompositeLanePath& composite_lane_path);

RouteProto CompositeLanePathToRouteProto(const CompositeLanePath& route_path);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_INTERFACE_GENERATE_ROUTE_H_
