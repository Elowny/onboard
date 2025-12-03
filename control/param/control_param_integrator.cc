#include "onboard/control/param/control_param_integrator.h"

#include <string>

#include "onboard/control/control_flags.h"
#include "onboard/control/param/param_util.h"
#include "onboard/lite/check.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace control {

namespace {

absl::Status IntegrateTsPkmpcControllerConf(VehicleModel vehicle_model,
                                            ControllerConf* controller_conf) {
  TsPkmpcControllerConf ts_pkmpc_controller_conf;

  std::string file_name =
      "onboard/control/param/param_file/ts_pkmpc_controller_conf/";
  switch (ClassifyVehicle(vehicle_model)) {
    case VehicleClassification::kPassengerCar:
      file_name += "passenger_car.pb.txt";
      break;
    case VehicleClassification::kMiniBus:
      file_name += "minibus.pb.txt";
      break;
    case VehicleClassification::kShuttle:
      file_name += "shuttle.pb.txt";
      break;
    case VehicleClassification::kLogisticsVehicle:
      file_name += "logistics.pb.txt";
      break;
    case VehicleClassification::kUnknown:
      return absl::InvalidArgumentError("Unknown vehicle type.");
  }
  QCHECK(file_util::TextFileToProto(file_name, &ts_pkmpc_controller_conf));
  // TODO(all): Add some basic check here if necessary.

  controller_conf->mutable_ts_pkmpc_controller_conf()->CopyFrom(
      ts_pkmpc_controller_conf);

  return absl::OkStatus();
}

absl::Status IntegrateLonTsPkmpcControllerConf(
    ControllerConf* controller_conf) {
  LonTsPkmpcControllerConf lon_ts_pkmpc_controller_conf;

  std::string file_name =
      "onboard/control/param/param_file/ts_pkmpc_controller_conf/"
      "lon_ts_pkmpc_controller_conf.pb.txt";
  QCHECK(file_util::TextFileToProto(file_name, &lon_ts_pkmpc_controller_conf));
  // TODO(all): Add some basic check here if necessary.

  controller_conf->mutable_lon_ts_pkmpc_controller_conf()->CopyFrom(
      lon_ts_pkmpc_controller_conf);

  return absl::OkStatus();
}

}  // namespace

absl::Status IntegrateControlParam(VehicleModel vehicle_model,
                                   ControllerConf* controller_conf) {
  QCHECK_NOTNULL(controller_conf);
  if (FLAGS_apply_control_code_param ||
      !controller_conf->has_ts_pkmpc_controller_conf()) {
    RETURN_IF_ERROR(
        IntegrateTsPkmpcControllerConf(vehicle_model, controller_conf));
  }
  if (FLAGS_apply_control_code_param ||
      !controller_conf->has_lon_ts_pkmpc_controller_conf()) {
    RETURN_IF_ERROR(IntegrateLonTsPkmpcControllerConf(controller_conf));
  }

  return absl::OkStatus();
}

}  // namespace control
}  // namespace qcraft
