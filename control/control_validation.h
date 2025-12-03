#ifndef ONBOARD_CONTROL_CONTROL_VALIDATION_H_
#define ONBOARD_CONTROL_CONTROL_VALIDATION_H_

#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

bool ValidateControlOutput(const VehicleStateProto& vehicle_state,
                           const SteeringConverter& steering_converter,
                           const ControllerConf& controller_conf,
                           const ControlCommand& control_cmd,
                           ControllerDebugProto* controller_debug_proto);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_VALIDATION_H_
