#ifndef ONBOARD_PLANNER_ASSIST_LCC_MAP_BUILDER_H_
#define ONBOARD_PLANNER_ASSIST_LCC_MAP_BUILDER_H_

#include <array>

#include "absl/status/statusor.h"

#include "common/proto/qalc.pb.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/planner_semantic_map_manager.h"

namespace qcraft::planner {

absl::StatusOr<mapping::LanePath> ProjectLanePathToDrivingMap(
    const mapping::LanePath& lane_path, const DrivingMapTopo& dm,
    const PlannerSemanticMapManager& psmm);

struct LccDrivingMapUpdateResult {
  mapping::LanePath aligned_origin_lane_path;
  mapping::LanePath aligned_target_lane_path;
  DrivingMapTopo driving_map;
};
absl::StatusOr<LccDrivingMapUpdateResult> UpdateLccDrivingMapByOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& origin_lane_path,
    const mapping::LanePath& target_lane_path,
    const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_pos);

absl::StatusOr<LccDrivingMapUpdateResult> UpdateLccDrivingMapByOfflineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& origin_lane_path,
    const mapping::LanePath& target_lane_path, const Vec2d& ego_pos);

// Return < Left_lane_path, center_lane_path, right_lane_path>. If there is no
// neighbor lane path, construct an empty lane path.
struct BuildLocalMapInput {
  const PlannerSemanticMapManager* psmm;
  const DrivingMapTopo* driving_map_topo;
  const mapping::LanePath* origin_lane_path;
  const mapping::LanePath* target_lane_path;
  QALCState alc_state;
  LaneChangeDirection lc_direction;
  double cut_off_length;
  double projection_range;
  double keep_behind_length;
};

absl::StatusOr<std::array<mapping::LanePath, 3>> BuildLocalLaneMap(
    const BuildLocalMapInput& input);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_LCC_MAP_BUILDER_H_
