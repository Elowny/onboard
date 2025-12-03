
#ifndef ONBOARD_PREDICTION_UTIL_LANE_PATH_FINDER_H_
#define ONBOARD_PREDICTION_UTIL_LANE_PATH_FINDER_H_

#include <optional>
#include <set>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft {
namespace prediction {
// This function returns the nearest lane id (if exists) if the query point is
// inside the lane boundary and satisfies the max_heading_diff condition,
// otherwise returns nullopt.
std::optional<mapping::ElementId>
FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
    const planner::PlannerSemanticMapManager& semantic_map_mgr, const Vec2d& pt,
    double heading, double boundary_distance_limit, double max_heading_diff);

// This function returns the nearest lane id (if exists) if the query point is
// inside the lane boundary with certain threshold, otherwise returns nullopt.
std::optional<mapping::ElementId> FindNearestLaneIdWithBoundaryDistanceLimit(
    const planner::PlannerSemanticMapManager& semantic_map_mgr, const Vec2d& pt,
    double boundary_distance_limit);

// This function returns the nearest lane id (if exists) if the bbox corners are
// inside the lane boundary and satisfies the max_heading_diff condition,
// otherwise returns nullopt.
std::set<mapping::ElementId>
FindNearestLaneIdsByBBoxWithBoundaryDistLimitAndHeadingDiffLimit(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const Box2d& box, double boundary_distance_limit, double max_heading_diff);

std::set<mapping::ElementId> FilterLanesByConsecutiveRelationship(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const std::set<mapping::ElementId>& lane_ids, bool filter_virtual);

std::vector<mapping::LanePath> SearchLanePath(
    const Vec2d& pos,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    mapping::ElementId lane_id, double forward_len, bool is_reverse_driving,
    bool filter_virtual = false);

// Always find the lane sequence with minimal heading diff
mapping::LanePath SearchMostStraightLanePath(
    const Vec2d& pos,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    mapping::ElementId lane_id, double forward_len, bool is_reverse_driving);

mapping::LanePath ExtendMostStraightLanePath(
    const mapping::LanePath& lp,
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    double forward_len, bool is_reversed);
// Sample predicted points on the path to find consecutive lanes.
absl::StatusOr<std::vector<mapping::ElementId>>
FindConsecutiveLanesForDiscretizedPath(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const planner::DiscretizedPath& path, bool is_reversed);

// This function returns the pointer of the nearest intersection (if exists)
// along the current lane. otherwise returns nullopt.
std::optional<const mapping::IntersectionInfo*>
FindIntersectionsAlongTheLaneByDistance(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    double forward_len, mapping::ElementId lane_id, double start_lane_fraction);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_UTIL_LANE_PATH_FINDER_H_
