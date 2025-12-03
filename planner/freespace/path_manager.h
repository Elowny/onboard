#ifndef ONBOARD_PLANNER_FREESPACE_PATH_MANAGER_H_
#define ONBOARD_PLANNER_FREESPACE_PATH_MANAGER_H_

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/maps_common.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct PathManagerOutput {
  // Path.
  DirectionalPath path;
  // Whether this is a new path.
  bool is_new_path;
};

absl::StatusOr<PathManagerOutput> GeneratePath(
    FreespaceReplanReasonProto::ReplanReason replan_reason,
    FreespaceTaskProto::TaskType task_type, const PoseProto& ego_pose,
    const Chassis& chassis,
    const FreespacePathFinderParamsProto& path_finder_params,
    const SqpSmootherParamsProto& sqp_smoother_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const PlannerClusterObjectManager& cluster_obj_mgr,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
        stalled_cluster_object_ids,
    const FreespaceMap& freespace_map, const PathPoint& goal,
    const mapping::ParkingSpotInfo* nullable_parking_spot_info,
    PathManagerStateProto* path_mgr_state,
    PathFinderDebugProto* path_finder_debug);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_PATH_MANAGER_H_
