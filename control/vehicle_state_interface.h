#ifndef ONBOARD_CONTROL_VEHICLE_STATE_INTERFACE_H_
#define ONBOARD_CONTROL_VEHICLE_STATE_INTERFACE_H_

#include <limits>
#include <optional>

#include "absl/status/status.h"

#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/math/filters/exponential_smoothing.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::control {

// Construct the current vehicle state from autonomy state, pose, and chassis.
class VehicleStateInterface {
 public:
  VehicleStateInterface();
  absl::Status Update(
      double yaw_bias, const AutonomyStateProto_State& autonomy_state,
      const PoseProto& pose, const Chassis& chassis,
      const std::optional<LocalizationViewerDebugProto>& localize_debug,
      const SteeringConverter& steering_converter,
      ControllerDebugProto* controller_debug_proto);

  VehicleStateProto Result() const;

 private:
  VehicleStateProto vehicle_state_proto_;

  double last_valid_chassis_steering_pct_ =
      std::numeric_limits<double>::quiet_NaN();
  int chassis_msg_loss_count_ = 0;
  std::optional<ExponentialSmoothing> lateral_velocity_estimator_;
  MeanFilter localize_y_offset_rate_estimator_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_VEHICLE_STATE_INTERFACE_H_
