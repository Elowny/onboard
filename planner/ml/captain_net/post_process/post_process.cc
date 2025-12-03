#include "onboard/planner/ml/captain_net/post_process/post_process.h"

#include "onboard/planner/ml/captain_net/post_process/movability_fixer.h"

namespace qcraft::planner::ml {
void RunCaptainNetPostProcess(
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    captain_net::CaptainNetOutput* output) {
  PostProcessTrajectoryMovability(vehicle_geometry_params, vehicle_drive_params,
                                  output);
}
}  // namespace qcraft::planner::ml
