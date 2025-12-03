#ifndef ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
#define ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft {
namespace planner {

enum class LaneSemantic {
  NONE = 0,                     // NOLINT
  ROAD = 1,                     // NOLINT
  INTERSECTION_LEFT_TURN = 2,   // NOLINT
  INTERSECTION_RIGHT_TURN = 3,  // NOLINT
  INTERSECTION_STRAIGHT = 4,    // NOLINT
  INTERSECTION_UTURN = 5        // NOLINT
};

struct PathPointSemantic {
  mapping::LanePoint closest_lane_point;
  Vec2d closest_lane_point_pos;
  LaneSemantic lane_semantic = LaneSemantic::NONE;
  // Keep the lane path id history from path beginning to the current point.
  // Lane path id starts from zero, and is increased by one if there is a left
  // lane change and reduced by one if there is a right lane change.
  // Examples:
  // 1) { 0 } means there is no lane change from path beginning to current
  // point;
  // 2) { 0, 1 } means there is a left lane change from path beginning to
  // current point;
  // 3) { 0, 1, 2 } means there are two successive left lane changes from path
  // beginning to current point;
  // 4) { 0, 1, 0 } means current point returns to the original lane after a
  // left and a right lane change.
  std::vector<int> lane_path_id_history;
  // The distance of the ego vehicle's path deviation from lane center (absolute
  // value).
  double deviation_distance;
  const mapping::LaneInfo* lane_info = nullptr;
};

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    const DiscretizedPath& path, int max_analyze_path_index,
    const PlannerSemanticMapManager& psmm,
    const DrivingMapTopo* driving_map_topo, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
