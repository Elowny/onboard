#include "onboard/prediction/long_term/slow_front_vehicle_recognizer.h"

#include <string>

#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace prediction {
namespace {

TEST(SlowFrontVehicleRecognizerTest, RecognizeSlowFrontHehicle) {
  const std::string id = "object_0";
  constexpr int kCacheNum = 110;
  constexpr double kObjInitSpeed = 10.0;  // m/s.
  const auto object_history = BuildVehicleHistoryByConstVel(
      id, kCacheNum, Vec2d::Zero(), kObjInitSpeed);
  VehicleGeometryParamsProto vehicle_geo_params;
  ObjectPredictionScenario object_scenario;
  object_scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
  AvContext av_context(/*capacity=*/10, /*len=*/2.0);

  LocalizationTransformProto loc_transform;
  VehicleGeometryParamsProto veh_geom_params;
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  constexpr double kAvSpeed = 10;    // m/s.
  constexpr double kTimeStep = 0.1;  // s.
  for (int i = 0; i < kCacheNum; ++i) {
    pose.mutable_pos_smooth()->set_x(pose.pos_smooth().x() +
                                     kAvSpeed * kTimeStep);
    pose.set_timestamp(i * kTimeStep);
    av_context.Update(pose, loc_transform, veh_geom_params);
  }
  const bool is_slow_front_vehicle = RecognizeSlowFrontVehicle(
      av_context, vehicle_geo_params, object_scenario, object_history);
  EXPECT_FALSE(is_slow_front_vehicle);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
