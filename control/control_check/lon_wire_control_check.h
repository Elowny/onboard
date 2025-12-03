#ifndef ONBOARD_CONTROL_CONTROL_CHECK_LON_WIRE_CONTROL_CHECK_H_
#define ONBOARD_CONTROL_CONTROL_CHECK_LON_WIRE_CONTROL_CHECK_H_

#include "boost/circular_buffer.hpp"

#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

class LonWireControlChecker {
 public:
  LonWireControlChecker();

  // Check hard throttle, avp hard speed, and brake decay rate.
  bool IsAbnormal(const VehicleStateProto& vehicle_state,
                  const ControlCommand& control_cmd,
                  WireControlCheckDebugProto* debug);

 private:
  // Check throttle acc >= 3.0m/s2, Kickout.
  bool IsAccelerateTooHard(double acc_fb);

  // Check AVP |speed| >= 3.0m/s2, acc >= 1.5m/s2, Kickout.
  bool IsAVPSpeedTooHard(bool is_freespace, double acc_fb, double speed);

  // If brake response rate < 0.7, Qevent.
  double ComputeBrakeResponseRate(double acc_cmd, double acc_fb, double speed);

  void Reset();

  boost::circular_buffer<double> acc_cmd_cache_;
  boost::circular_buffer<double> acc_fb_cache_;

  int num_acc_check_ = 0;
  int num_brake_delay_ = 0;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_CHECK_LON_WIRE_CONTROL_CHECK_H_
