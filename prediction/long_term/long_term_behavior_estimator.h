#ifndef ONBOARD_PREDICTION_LONG_TERM_LONG_TERM_BEHAVIOR_ESTIMATOR_H_
#define ONBOARD_PREDICTION_LONG_TERM_LONG_TERM_BEHAVIOR_ESTIMATOR_H_

#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {

ObjectLongTermBehaviorProto EstimateLongTermBehavior(
    const AvContext& av_context,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ObjectPredictionScenario& object_scenario,
    const ObjectHistory& object_history);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_LONG_TERM_LONG_TERM_BEHAVIOR_ESTIMATOR_H_
