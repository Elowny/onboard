#ifndef ONBOARD_PLANNER_ROUTER_NOA_ROUTE_MPP_H_
#define ONBOARD_PLANNER_ROUTER_NOA_ROUTE_MPP_H_

#include <cstdint>
#include <optional>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "common/proto/drive_mission.pb.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/road_constraints.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route::noa {
struct NoaRouteMpp {
  RouteProto route;
  bool is_noa_end;
  int noa_length;  // meters
};

NoaRouteMpp MppToNoaRoute(const mapping::v2::SemanticMapManager& v2smm,
                          absl::Span<const std::int64_t> sections,
                          int update_id, bool route_trunc_mpp_by_nav);

AvoidLaneSegmentMap ParseDynamicOddAlongMpp(
    const mapping::v2::SemanticMapManager& v2smm,
    absl::Span<const int64_t> sections_id);

AvoidLaneSegmentMap MergeDynamicOddWithAvoidLanes(
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    AvoidLaneSegmentMap* mutable_lane_segments_map);

/**
 *@brief Provide navigation information for `Decision` from the given map and
 *MPP sections with pose.
 *@param smm_index A SemanticMapSpatialIndex object representing the map.
 *@param mpp A MppSectionsProto object containing MPP sections.
 *@param global A Vec2d object representing global coordinates.
 *@param heading the AV heading, used for filter near lanes if provided.
 *@param update_id An integer representing for route's update
 *@return An absl::StatusOr<RouteManagerOutputProto> object containing
 *navigation information.
 */
absl::StatusOr<RouteManagerOutputProto> CalcNavInfoFromMpp(
    const mapping::v2::SemanticMapSpatialIndex& smm_index,
    const MppSectionsProto& mpp, const Vec2d& global,
    const std::optional<double>& heading, int64_t update_id,
    const std::optional<double>& av_speed,
    const std::optional<double>& look_ahead_time, bool route_trunc_mpp_by_nav,
    const RestrictProto& restrict_proto);

}  // namespace qcraft::planner::route::noa
#endif  // ONBOARD_PLANNER_ROUTER_NOA_ROUTE_MPP_H_
