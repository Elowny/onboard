#ifndef ONBOARD_CONTROL_PARKING_STATIONARY_STEER_CONTROL_H_
#define ONBOARD_CONTROL_PARKING_STATIONARY_STEER_CONTROL_H_

#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

double CalcStationarySteeringCmd(double previous_control_kappa, double av_kappa,
                                 double ref_kappa,
                                 const SteeringConverter& steering_converter);

struct StationarySteerControlInput {
  bool is_stationary_steer = false;
  double kappa_cmd = 0.0;
  double kappa_trajectory = 0.0;
  const VehicleStateProto* vehicle_state = nullptr;
  const SteeringProtectionResult* steering_protection_result = nullptr;
  const SteeringConverter* steering_converter = nullptr;
};

class StationarySteerControl {
 public:
  double ComputestationarySteerCmd(StationarySteerControlInput input);

 private:
  bool first_hit_stationary_steering_ = true;
  double previous_stationary_steering_cmd_ = 0.0;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_PARKING_STATIONARY_STEER_CONTROL_H_
