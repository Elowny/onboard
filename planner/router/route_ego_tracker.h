#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_EGO_TRACKER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_EGO_TRACKER_H_

#include <stdint.h>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/level_id.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

// The result of a point(vehicle) project to a route.
struct PointToCompositeLanePath {
  bool point_on_route_lane = false;
  // An map matched index of the route composite lane path
  CompositeLanePath::CompositeIndex composite_index = {0, 0};
  // The lane id which on the route other than the vehicle is on
  mapping::ElementId route_lane_id;
};

absl::StatusOr<PointToCompositeLanePath> FindLaneFromLastCompositeIndex(
    const CompositeLanePathProto& composite_lane_path,
    const CompositeLanePath::CompositeIndex& last_composite_index,
    absl::Span<const int64_t> section_lane_ids,
    mapping::ElementId to_find_lane_id);

RouteTrackerDebugProto GetTrackerInfoOnRoute(
    const mapping::v2::SemanticMapManager& smm,
    const route::MapIndex* map_index, mapping::LevelId level_id,
    const RouteManagerState& rm_state, const Vec2d& global, double heading,
    double av_speed_x);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_EGO_TRACKER_H_
