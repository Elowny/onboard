#ifndef ONBOARD_PLANNER_COMMON_DATA_FILTERING_UTILS_H_
#define ONBOARD_PLANNER_COMMON_DATA_FILTERING_UTILS_H_

#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

bool TrajIntentionSameAsExpert(
    const std::vector<TrajectoryPoint>& expert_traj_points,
    const std::vector<TrajectoryPoint>& candidate_traj_points,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_DATA_FILTERING_UTILS_H_
