#ifndef ONBOARD_PLANNER_PATH_MANAGER_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PATH_MANAGER_PLANNER_UTIL_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

using DriveState = PathManagerStateProto::DriveState;

// This path safety check just check global paths after current driving
// segment.
absl::Status PathSafetyCheck(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    const FreespacePathFinderParamsProto& path_finder_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const PoseProto& ego_pose,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const FreespaceMap& freespace_map,
    const google::protobuf::RepeatedPtrField<DirectionalPathProto>& paths,
    int current_index);

absl::StatusOr<PathPoint> GetPathPointFromGlobalIndex(
    absl::Span<const DirectionalPath* const> paths, int global_index);

void UpdatePathManagerState(const VehicleGeometryParamsProto& vehicle_geom,
                            const VehicleDriveParamsProto& vehicle_drive,
                            const FreespaceTaskProto::TaskType& task_type,
                            const PoseProto& av_pose, const Chassis& chassis,
                            PathManagerStateProto* state,
                            bool* switched_to_new_path);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PATH_MANAGER_PLANNER_UTIL_H_
