#ifndef ONBOARD_PREDICTION_LONG_TERM_SLOW_FRONT_VEHICLE_RECOGNIZER_H_
#define ONBOARD_PREDICTION_LONG_TERM_SLOW_FRONT_VEHICLE_RECOGNIZER_H_

#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {

bool RecognizeSlowFrontVehicle(
    const AvContext& av_context,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ObjectPredictionScenario& object_scenario,
    const ObjectHistory& object_history);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_LONG_TERM_SLOW_FRONT_VEHICLE_RECOGNIZER_H_
