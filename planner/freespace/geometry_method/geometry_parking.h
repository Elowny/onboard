#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRY_PARKING_H
#define ONBOARD_PLANNER_FREESPACE_GEOMETRY_PARKING_H

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/maps/maps_common.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<DirectionalPath>> FindSinglePath(
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    FreespaceTaskProto::TaskType task_type,
    FreespaceReplanReasonProto::ReplanReason replan_reason,
    const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const mapping::ParkingSpotInfo* parking_spot_info, const PathPoint& start,
    const PathPoint& end, PathFinderDebugProto* debug_info);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRY_PARKING_H
