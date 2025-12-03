#ifndef ONBOARD_PLANNER_COMMON_PATH_GENERATOR_UTIL_H_
#define ONBOARD_PLANNER_COMMON_PATH_GENERATOR_UTIL_H_

#include <optional>

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

bool IsPointInLeftLaneBoundary(const DrivePassage& drive_passage,
                               const Vec2d& point);

bool IsPointInRightLaneBoundary(const DrivePassage& drive_passage,
                                const Vec2d& point);

bool IsPointInLaneBoundaries(const DrivePassage& drive_passage,
                             const Vec2d& point);

/**
 * @brief Build the drive passage with certain start lane id & start path point.
 * @param psmm Planner semantic map manager.
 * @param lane_id The start lane id to build drive passage.
 * @param start_path_point The start path point to build drive passage.
 * @param max_length Maximum forward extension length from start path point for
 * drive passage.
 * @return The expected drive passage. Will return std::nullopt if failed to
 * build drive passage.
 */
std::optional<DrivePassage> BuildDrivePassageFromLaneId(
    const PlannerSemanticMapManager& psmm, const mapping::ElementId& lane_id,
    const PathPoint& start_path_point, double max_length);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_PATH_GENERATOR_UTIL_H_
