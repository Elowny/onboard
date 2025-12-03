#ifndef ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_WITHOUT_MAP_H_
#define ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_WITHOUT_MAP_H_

#include <optional>

#include "absl/status/statusor.h"

#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<AccCorridor> BuildAccCorridorFromPlanStartPoint(
    const PoseProto& pose, const ApolloTrajectoryPointProto& plan_start_point,
    const std::optional<double>& steering_percentage,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double corridor_step_s,
    double total_preview_time, double av_kappa_cache_average);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_WITHOUT_MAP_H_
