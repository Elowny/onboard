#include "onboard/prediction/long_term/long_term_behavior_estimator.h"

#include <numeric>

#include "boost/circular_buffer.hpp"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/long_term/slow_front_vehicle_recognizer.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {

ObjectLongTermBehaviorProto EstimateLongTermBehavior(
    const AvContext& av_context,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ObjectPredictionScenario& object_scenario,
    const ObjectHistory& object_history) {
  const auto hist = object_history.GetHistory();
  QCHECK_GT(hist.size(), 0);

  ObjectLongTermBehaviorProto res;
  res.set_observation_duration(hist.back().time - hist.front().time);

  const double sum_v = std::accumulate(
      hist.begin(), hist.end(), 0.0,
      [](const double val, const auto& obj) { return val + obj.val.v(); });
  res.set_average_speed(sum_v / static_cast<double>(hist.size()));

  auto* accel_hist = res.mutable_accel_history();
  for (const auto& [_, val] : hist) {
    accel_hist->Add(Vec2dFromProto(val.object_proto().accel())
                        .Dot(Vec2d::FastUnitFromAngle(val.heading())));
  }

  res.set_is_slow_front_vehicle(RecognizeSlowFrontVehicle(
      av_context, vehicle_geometry_params, object_scenario, object_history));

  return res;
}

}  // namespace prediction
}  // namespace qcraft
