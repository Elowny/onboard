#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/vec.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route_sections.h"
namespace qcraft {
namespace prediction {
mapping::LanePath PruneLanePathByLength(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const mapping::LanePath& lane_path, const mapping::LanePoint& lane_point,
    double front_length, double back_length);

std::vector<mapping::LanePath> FilterLanePathByDistance(
    const std::vector<mapping::LanePath>& lane_paths,
    const planner::PlannerSemanticMapManager& psmm, const Vec2d& pos,
    double heading, int max_num);

absl::StatusOr<planner::DrivePassage> BuildAvDrivePassageWithNearestLane(
    const planner::PlannerSemanticMapManager& psmm,
    const planner::LaneBoundaryCache& lane_boundary_cache, const Vec2d& av_pos,
    double av_heading, double max_heading_diff, double backward_extend_len,
    double front_length, double back_length);

absl::StatusOr<planner::DrivePassage> BuildAvDrivePassageWithRouting(
    const planner::PlannerSemanticMapManager& semantic_map_mgr,
    const planner::LaneBoundaryCache& lane_boundary_cache,
    const planner::RouteSections& sections, const Vec2d& query_point,
    double backward_extend_len, double front_length, double back_length);

}  // namespace prediction
}  // namespace qcraft
