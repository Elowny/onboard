#ifndef ONBOARD_PLANNER_COMMON_MAP_PATH_GENERATOR_H_
#define ONBOARD_PLANNER_COMMON_MAP_PATH_GENERATOR_H_

#include <optional>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"

#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

/**
 * @brief Judge whether a certain lane is opposite direction or not.
 * @param psmm Planner semantic map manager.
 * @param lane_info The lane info.
 * @param lane_attr_map The hash map with lane id and lane attribute which is
 * opposite direction or not.
 * @return If the lane is opposite.
 */
bool IsOppositeLane(
    const PlannerSemanticMapManager& psmm, const mapping::LaneInfo* lane_info,
    absl::flat_hash_map<mapping::ElementId, bool>* lane_attr_map);

/**
 * @brief Generate a smooth path with coarse points in x, y, s form.
 * Recommand to extend coarse path reversely to get a better approximation
 * of theta & kappa at s = 0.
 * @param speed Current speed of ego vehicle.
 * @param path_step Path step to generate smooth path.
 * @param max_check_distance The max length to check if path satisfies
 * motion constraints (e.g. kappa, accel, etc.).
 * @param max_lateral_acc_limit The max lateral acceleration limit in path
 * points.
 * @param x The x-coords of coarse points.
 * @param y The y-coords of coarse points.
 * @param s The arc length of coarse points.
 * @return Error status or expected smooth path.
 */
absl::StatusOr<DiscretizedPath> GenerateSmoothPath(
    double speed, double path_step, double max_check_distance,
    double max_lateral_acc_limit, std::vector<double>* x,
    std::vector<double>* y, std::vector<double>* s);

/**
 * @brief Generate a lane-keep path with a plan start point and drive passsage.
 * @param drive_passage The drive passage of current lane.
 * @param plan_start_point The start point to plan path.
 * @param path_length Expected path length.
 * @param path_min_length Minimum path length allowed. If not satisfied, an
 * error code will be returned.
 * @param path_min_extension_time The minimum time headway. If the path length
 * is greater than the path_min_length but less than this time headway, path
 * will be extended to that distance.
 * @return Error status or expected smooth path.
 */
absl::StatusOr<DiscretizedPath> GenerateLaneKeepPathFromDrivePassage(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point, double path_length,
    double path_step, double path_min_length, double path_min_extension_time);

/**
 * @brief Generate a lane-change path with a plan start point and targe drive
 * passsage.
 * @param target_drive_passage The drive passage of target lane.
 * @param plan_start_point The start point to plan path.
 * @param path_length Expected path length.
 * @param path_min_length Minimum path length allowed. If not satisfied, an
 * error code will be returned.
 * @param path_min_extension_time The minimum time headway. If the path length
 * is greater than the path_min_length but less than this time headway, path
 * will be extended to that distance.
 * @return Error status or expected smooth path.
 */
absl::StatusOr<DiscretizedPath> GenerateLaneChangePathFromDrivePassage(
    const DrivePassage& target_drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point, double path_length,
    double path_step, double path_min_length, double path_min_extension_time);

/**
 * @brief Generate a lane-keep path by motion prediction to preview and match
 * lanes.
 * @param psmm Planner semantic map manager.
 * @param plan_start_point The start point to plan path.
 * @param path_length Expected path length.
 * @param path_preview_time The maximum time to preview and match lanes by
 * motion prediction.
 * @param path_min_length Minimum path length allowed. If not satisfied, an
 * error code will be returned.
 * @param path_min_extension_time The minimum time headway. If the path length
 * is greater than the path_min_length but less than this time headway, path
 * will be extended to that distance.
 * @param path_min_extension_time The minimum time headway. If the path length
 * is greater than the path_min_length but less than this time headway, path
 * will be extended to that distance.
 * @param max_search_radius The maximum radius to search lanes.
 * @param max_heading_diff The maximum difference of heading to match lanes.
 * @param drive_passage_length_factor The factor to extend length to build drive
 * passage.
 * @param min_start_preview_distance The min distance to start to preview.
 * @param candidate_path The candidate map path.
 * @return Error status or expected smooth path.
 */
absl::StatusOr<DiscretizedPath> GeneratePreviewLaneKeepPathFromStartPoint(
    const PlannerSemanticMapManager& psmm,
    const ApolloTrajectoryPointProto& plan_start_point, double path_length,
    double path_preview_time, double path_step, double path_min_length,
    double path_min_extension_time, double max_search_radius,
    double max_heading_diff, double drive_passage_length_factor,
    double min_start_preview_distance,
    std::optional<DiscretizedPath>* candidate_path);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_MAP_PATH_GENERATOR_H_
