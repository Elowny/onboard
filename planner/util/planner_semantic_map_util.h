#ifndef ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/level_id.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/range1d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

struct SamplePathPointsResult {
  std::vector<Vec2d> points;
  bool is_partial;
  std::string message;
};

bool IsOutgoingLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LaneInfo& source_lane, mapping::ElementId out_lane_id);

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s);

bool IsRightMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id);

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, double s);

bool IsLeftMostLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::ElementId lane_id);

bool IsTurningLane(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    mapping::ElementId lane_id);

bool IsLanePathBlockedByBox2dAtLevel(
    mapping::LevelId level_id,
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const Box2d& box, const mapping::LanePath& lane_path, double lat_thres);

std::vector<Vec2d> SampleLanePathPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path);

// Sample points by smooth coordinates. Can be unified with the above function.
absl::StatusOr<SamplePathPointsResult> SampleLanePathProtoPoints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePathProto& lane_path);

absl::StatusOr<mapping::LanePath> ClampLanePathFromPos(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const mapping::LanePath& lane_path, const Vec2d& pos);

absl::StatusOr<mapping::LanePoint> FindOutgoingLanePointWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm, mapping::ElementId id,
    bool allow_virtual);

std::vector<mapping::ElementId> RecomputeStartLanesByPosWithLaneIds(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const mapping::ElementId> start_search_lane_ids,
    const Vec2d& pos, double proj_thres, double max_backward_thres);

// Compute how much a point invades the support of a lane. A lane support is
// defined to be either
//   1) the region among the lane's boundaries (if boundaries exist); or
//   2) the laterally dilated region along the lane with the preset lane
//      width.
//
// Invasion distance: positive => invasion, negative => non-invasion

double GetDistOfPointInvasionLaneSupport(
    const Vec2d& smooth_coord,
    const PlannerSemanticMapManager& semantic_map_manager,
    const mapping::ElementId& lane_id, double lane_width = kDefaultLaneWidth);

bool HasSmoothPointsCrossedBoundary(const PlannerSemanticMapManager& psmm,
                                    absl::Span<const Vec2d> points,
                                    const Range1d<int>& ignore_range,
                                    bool solid_only);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_PLANNER_SEMANTIC_MAP_UTIL_H_
