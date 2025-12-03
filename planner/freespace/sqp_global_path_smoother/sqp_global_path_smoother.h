#ifndef ONBOARD_PLANNER_FREESPACE_SQP_GLOBAL_PATH_SMOOTHER_H
#define ONBOARD_PLANNER_FREESPACE_SQP_GLOBAL_PATH_SMOOTHER_H
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace sqp_global_smoother {
absl::StatusOr<std::vector<DirectionalPath>> SmoothGlobalPath(
    const std::vector<DirectionalPath>& init_path,
    const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SqpSmootherParamsProto& sqp_smoother_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const FreespacePathFinderParamsProto& freespace_path_finder_params);
}  // namespace sqp_global_smoother
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_FREESPACE_SQP_GLOBAL_PATH_SMOOTHER_H
