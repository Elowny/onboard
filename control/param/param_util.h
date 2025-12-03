#include "onboard/params/v2/proto/vehicle/common.pb.h"

namespace qcraft {
namespace control {

enum class VehicleClassification {
  kUnknown = 0,
  kPassengerCar = 1,
  kMiniBus = 2,
  kShuttle = 3,
  kLogisticsVehicle = 4,
};

VehicleClassification ClassifyVehicle(VehicleModel vehicle_model);

}  // namespace control
}  // namespace qcraft
