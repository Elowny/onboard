#ifndef ONBOARD_CONTROL_PARAM_CONTROL_PARAM_INTEGRATOR_
#define ONBOARD_CONTROL_PARAM_CONTROL_PARAM_INTEGRATOR_

#include "absl/status/status.h"

#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"

namespace qcraft {
namespace control {

absl::Status IntegrateControlParam(VehicleModel vehicle_model,
                                   ControllerConf* controller_conf);

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_PARAM_CONTROL_PARAM_INTEGRATOR_
