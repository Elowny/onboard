#ifndef ONBOARD_PLANNER_TEST_UTIL_ROUTE_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_ROUTE_BUILDER_H_

#include <string>

#include "absl/status/statusor.h"

#include "common/proto/drive_mission.pb.h"  // for RoutingRequestProto

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
RouteParamProto CreateDefaultRouteParam();
CompositeLanePath RoutingToNameSpot(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const PoseProto& pose,
    std::string name_spot);

CompositeLanePath RoutingToNameSpot(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const route::MapIndex& map_index,
    const PoseProto& pose, std::string name_spot);

absl::StatusOr<CompositeLanePath> RoutingToLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const PoseProto& pose,
    const CompositeLanePath::LanePoint& lane_point);

absl::StatusOr<CompositeLanePath> RoutingToLanePoint(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const route::MapIndex& map_index,
    const PoseProto& pose, const CompositeLanePath::LanePoint& lane_point);

absl::StatusOr<CompositeLanePath> CalcRoutingRequest(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const CoordinateConverter& cc, const route::MapIndex& map_index,
    const RoutingRequestProto& routing_request, const PoseProto& pose);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_ROUTE_BUILDER_H_
