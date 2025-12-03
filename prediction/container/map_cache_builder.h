
#ifndef ONBOARD_PREDICTION_CONTAINER_MAP_CACHE_BUILDER_H_
#define ONBOARD_PREDICTION_CONTAINER_MAP_CACHE_BUILDER_H_
// IWYU pragma: no_include <algorithm>
// IWYU pragma: no_include "absl/hash/hash.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace prediction {
using DrivePassages = std::vector<std::unique_ptr<planner::DrivePassage>>;
using DrivePassageCache =
    absl::flat_hash_map<mapping::ElementId,
                        std::vector<const planner::DrivePassage*>>;

planner::LaneBoundaryCache BuildLaneBoundaryCache(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const Vec2d& ego_pos, double ego_heading, double front_dist,
    double back_dist, double side_dist, double desire_sample_step);

std::pair<DrivePassages, DrivePassageCache> BuildDrivePassageCache(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const planner::LaneBoundaryCache& lane_boundary_cache, const Vec2d& ego_pos,
    double ego_heading, double front_dist, double back_dist, double side_dist,
    double desire_sample_step, int max_dp_num, bool filter_virtual);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_MAP_CACHE_BUILDER_H_
