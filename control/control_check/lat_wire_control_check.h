#ifndef ONBOARD_CONTROL_CONTROL_CHECK_LAT_WIRE_CONTROL_CHECK_H_
#define ONBOARD_CONTROL_CONTROL_CHECK_LAT_WIRE_CONTROL_CHECK_H_

#include <memory>

#include "boost/circular_buffer.hpp"

#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

class LatWireControlChecker {
 public:
  LatWireControlChecker(double steer_delay, double wheel_base,
                        double steer_ratio, double max_steer_angle);

  // Check hard throttle, avp hard speed, and brake decay rate.
  bool IsAbnormal(const VehicleStateProto& vehicle_state,
                  const ControlCommand& control_cmd, bool is_onboard_mode,
                  WireControlCheckDebugProto* debug);

 private:
  // Check steer wheel angle error > threshold, Kickout
  bool IsSteerAngleErrorTooLarge(double steer_error, double speed,
                                 bool is_onboard_mode);

  void Reset();

  boost::circular_buffer<double> steer_cmd_cache_;
  int num_steer_delay_ = 0;  // continuous cache num of kapparate cmd check time
  std::unique_ptr<SteeringConverter> steer_converter_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_CHECK_LAT_WIRE_CONTROL_CHECK_H_
