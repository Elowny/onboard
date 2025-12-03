#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_MOVABILITY_FIXER_H_
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_MOVABILITY_FIXER_H_

#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner::ml {

void PostProcessTrajectoryMovability(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    captain_net::CaptainNetOutput* output);

}  // namespace qcraft::planner::ml
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_MOVABILITY_FIXER_H_
