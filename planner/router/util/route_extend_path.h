#ifndef ONBOARD_PLANNER_ROUTER_UTIL_ROUTE_EXTEND_PATH_H_
#define ONBOARD_PLANNER_ROUTER_UTIL_ROUTE_EXTEND_PATH_H_
#include "absl/status/statusor.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/v2/semantic_map_manager.h"
namespace qcraft::planner::route {

absl::StatusOr<mapping::LanePathProto> FindConnectedLanePathWithin(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const mapping::LanePoint& origin, const mapping::LanePoint& dest,
    double allow_distance = 100.0);

}  // namespace qcraft::planner::route
#endif  // ONBOARD_PLANNER_ROUTER_UTIL_ROUTE_EXTEND_PATH_H_
