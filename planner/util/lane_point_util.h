#ifndef ONBOARD_PLANNER_UTIL_LANE_POINT_UTIL_H_
#define ONBOARD_PLANNER_UTIL_LANE_POINT_UTIL_H_

#include <optional>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

Vec2d ComputeLanePointPos(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePoint& lane_point);

Vec2d ComputeLanePointTangent(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePoint& lane_point);

double ComputeLanePointLerpTheta(const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePoint& lane_point);

double ComputeLanePointLerpThetaWithSuccessorLane(
    const PlannerSemanticMapManager& psmm, mapping::ElementId succ_lane_id,
    const mapping::LanePoint& lane_point);

absl::StatusOr<Vec2d> ComputeLanePointGlobalPose(
    const mapping::v2::SemanticMapManager& smm,
    const mapping::LanePoint& lane_point);

absl::StatusOr<Vec2d> ComputeLanePointGlobalHeading(
    const mapping::v2::SemanticMapManager& smm,
    const mapping::LanePoint& lane_point);

absl::StatusOr<std::optional<mapping::LaneBoundaryProto::Type>>
QueryLanePointBoundaryType(const PlannerSemanticMapManager& psmm,
                           const mapping::LanePoint& lane_point, bool left);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_UTIL_LANE_POINT_UTIL_H_
