#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_CONSTRAINT_BUILDER_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_CONSTRAINT_BUILDER_H_

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/maps_common.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<FreespaceMap> ConstructFreespaceMap(
    FreespaceTaskProto::TaskType task_type, double freespace_region_half_width,
    const VehicleGeometryParamsProto& vehicle_geom,
    const PlannerSemanticMapManager* psmm, const PoseProto& ego_pose,
    const mapping::ParkingSpotInfo* parking_spot_info, const PathPoint& goal);

void AddUTurnBoundary(const PlannerSemanticMapManager& psmm,
                      const mapping::LanePath* lane_path,
                      const VehicleGeometryParamsProto& vehicle_geom,
                      FreespaceMap* freespace_map);

absl::StatusOr<ConstraintManager> BuildFreespacePlannerConstraint(
    const VehicleGeometryParamsProto& veh_geo_params,
    const DirectionalPath& path, const PoseProto& ego_pose, bool force_stop,
    FreespacePlannerStateProto* state);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_CONSTRAINT_BUILDER_H_
