#ifndef ONBOARD_PLANNER_UTIL_SPATIAL_SEARCH_UTIL_H_
#define ONBOARD_PLANNER_UTIL_SPATIAL_SEARCH_UTIL_H_

#include <optional>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/level_id.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"

// TODO(Jinyun): adapt to newer version of semantic_map_util.
namespace qcraft::planner {

constexpr double kSearchRadiusThreshold = 10.0;

/*
  Return the lane point on any lanes that is closest to the input query point.
  Only lane points within the cutoff distance are considered, and if no lane
  points are within the cutoff distance, the returned lane point's id is
  kInvalidElementId. closest_point_on_lane will be populated with the 2D lane
  point if not null.
 */
absl::StatusOr<mapping::LanePoint> FindClosestLanePointToSmoothPointAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, Vec2d* closest_point_on_lane = nullptr,
    double cutoff_distance = kSearchRadiusThreshold);

/*
  Return the lane point on any lanes that is closest to the input query point,
  subject to the penalty weight by heading prior distribution. Only lane
  points within the cutoff distance are considered, and if no lane points are
  within the cutoff distance, the returned lane point's id is
  kInvalidElementId. The rest is the same as
  FindClosestLanePointToSmoothPointAtLevel.

  The penalty is evaluated as the weighted squared deviation of the lane's
  tangent orientation from the heading (all in radians).
 */
absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, double heading,
    double heading_penalty_weight = 0.0, Vec2d* closest_point_on_lane = nullptr,
    double cutoff_distance = kSearchRadiusThreshold);

/*
  Same as FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel but only
  consider lane points on lanes from a given single lane path. Only lane points
  within the cutoff distance are considered, and if no lane points are within
  the cutoff distance, the returned lane point's id is kInvalidElementId.
 */
absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, const mapping::LanePath& lane_path,
    double heading, double heading_penalty_weight = 0.0,
    Vec2d* closest_point_on_lane = nullptr,
    double cutoff_distance = kSearchRadiusThreshold);

/*
  Same as FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel but only
  consider lane points on the lane corresponding to the given lane id. Only lane
  points within the cutoff distance are considered, and if no lane points are
  within the cutoff distance, the returned lane point's id is kInvalidElementId.
 */
std::optional<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundOnLaneAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, mapping::ElementId lane_id, double heading,
    double heading_penalty_weight = 0.0, Vec2d* closest_point_on_lane = nullptr,
    double start_fraction = 0.0, double end_fraction = 1.0,
    double cutoff_distance = kSearchRadiusThreshold);

/*
  Same as FindClosestLanePointToSmoothPointWithHeadingBoundAtLevel but only
  consider lane points on lanes from given lanes. If given lane_ids is empty,
  all lanes from the semantic map will be considered. Only lane points within
  the cutoff distance are considered, and if no lane points are within the
  cutoff distance, the returned lane point's id is kInvalidElementId.
 */
template <typename T>  // T must be an STL container of type ElementId.
absl::StatusOr<mapping::LanePoint>
FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, const T& lane_ids, double heading,
    double heading_penalty_weight = 0.0, Vec2d* closest_point_on_lane = nullptr,
    double start_fraction = 0.0, double end_fraction = 1.0,
    double cutoff_distance = kSearchRadiusThreshold);

std::vector<std::pair<double, mapping::LanePoint>>
FindCloseLanePointsAndDistanceToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, double heading,
    double heading_penalty_weight = 0.0,
    double spatial_distance_threshold = 1.5,
    double angle_error_threshold = 0.35);

std::vector<mapping::LanePoint>
FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
    mapping::LevelId level_id, const PlannerSemanticMapManager& psmm,
    const Vec2d& query_point, double heading,
    double heading_penalty_weight = 0.0,
    double spatial_distance_threshold = 1.5,
    double angle_error_threshold = 0.35);

bool IsPointOnLanePathAtLevel(mapping::LevelId level_id,
                              const PlannerSemanticMapManager& psmm,
                              const Vec2d& query_point,
                              const mapping::LanePath& lane_path,
                              double* arc_len_on_lane_path,
                              double lateral_error_buffer = 2.0);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_SPATIAL_SEARCH_UTIL_H_
