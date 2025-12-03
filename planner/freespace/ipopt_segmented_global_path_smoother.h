#ifndef ONBOARD_PLANNER_FREESPACE_IPOPT_SEGMENTED_GLOBAL_PATH_SMOOTHER_H_
#define ONBOARD_PLANNER_FREESPACE_IPOPT_SEGMENTED_GLOBAL_PATH_SMOOTHER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<DirectionalPath>> SmoothGlobalPath(
    const std::vector<DirectionalPath>& init_path,
    const FreespaceMap& freespace_map,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_IPOPT_SEGMENTED_GLOBAL_PATH_SMOOTHER_H_
