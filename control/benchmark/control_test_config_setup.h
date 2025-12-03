#ifndef ONBOARD_CONTROL_BENCHMARK_CONTROL_TEST_CONFIG_SETUP_
#define ONBOARD_CONTROL_BENCHMARK_CONTROL_TEST_CONFIG_SETUP_

#include <string>

#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

struct ControllerParams {
  ControllerConf controller_conf;
  VehicleGeometryParamsProto vehicle_geo_params;
  VehicleDriveParamsProto vehicle_drive_params;
};

// Below are only for test file configurations;
TrajectoryProto BuildTrajectoryProtoForTest();
Chassis BuildChassisForTest();
AutonomyStateProto BuildAutonomyStateProtoForTest();
PoseProto BuildPoseForTest();
ControlCommand BuildControlCommandForTest();
ControllerDebugProto BuildControllerDebugProtoForTest();
ControllerParams BuildControllerParams(const std::string& car_id);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_cache_MANAGER_H_
