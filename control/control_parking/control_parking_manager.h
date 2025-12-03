#ifndef ONBOARD_CONTROL_PARKING_CONTROL_PARKING_MANAGER_H_
#define ONBOARD_CONTROL_PARKING_CONTROL_PARKING_MANAGER_H_

#include "onboard/control/control_parking/stationary_steer_control.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/control/trajectory_interface.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

struct ParkingManagerInput {
  const TrajectoryInterface* trajectory_interface = nullptr;
  const VehicleStateProto* vehicle_state = nullptr;
  const SteeringProtectionResult* steering_protection_result = nullptr;
  const SteeringConverter* steering_converter = nullptr;
  bool is_onboard = false;
};

struct ParkingState {
  bool reset_lon_controller = false;
  bool reset_lat_controller = false;
};

class ParkingManager {
 public:
  explicit ParkingManager(const VehicleModel& vehicle_model);
  ParkingState ParkingProcess(const ParkingManagerInput& input,
                              ControlCommand* command,
                              ControllerDebugProto* debug);

 private:
  VehicleModel vehicle_model_;
  StationarySteerControl stationary_steer_control_;
};

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_PARKING_CONTROL_PARKING_MANAGER_H_
