#ifndef ONBOARD_CONTROL_CONTROL_UTIL_H_
#define ONBOARD_CONTROL_CONTROL_UTIL_H_

#include "onboard/params/v2/proto/vehicle/common.pb.h"

namespace qcraft::control {

bool HasPhysicalSteeringWheel(const VehicleModel& vehicle_model);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROL_UTIL_H_
