#ifndef ONBOARD_PLANNER_PLANNER_PARAMS_BUILDER_H_
#define ONBOARD_PLANNER_PLANNER_PARAMS_BUILDER_H_

#include "absl/status/statusor.h"

#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/v2/proto/vehicle/installation.pb.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<PlannerParamsProto> BuildPlannerParams(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    VehicleModel vehicle_model,
    VehicleInstallationProto::VehiclePlan vehicle_plan);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_PARAMS_BUILDER_H_
