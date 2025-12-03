#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_POST_PROCESS_POST_PROCESS_H_
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_POST_PROCESS_POST_PROCESS_H_

#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNetOutput
#include "onboard/proto/vehicle.pb.h"  // for VehicleDriveParamsProto, VehicleGeometryParamsProto

namespace qcraft::planner::ml {

void RunCaptainNetPostProcess(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    captain_net::CaptainNetOutput* output);

}  // namespace qcraft::planner::ml
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_POST_PROCESS_POST_PROCESS_H_
