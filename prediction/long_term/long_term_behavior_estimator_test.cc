#include "onboard/prediction/long_term/long_term_behavior_estimator.h"

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/test_util/object_history_builder.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace prediction {
namespace {

TEST(LongTermBehaviorTest, EstimateLongTermBehavior) {
  const std::string id = "object_0";
  const int history_num = 20;
  const double vel = 10.0;
  const auto object_history =
      BuildVehicleHistoryByConstVel(id, history_num, Vec2d::Zero(), vel);
  VehicleGeometryParamsProto vehicle_geo_params;
  ObjectPredictionScenario object_scenario;
  object_scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
  AvContext av_context(/*capacity=*/10, /*len=*/2.0);
  const auto long_term_behavior_proto = EstimateLongTermBehavior(
      av_context, vehicle_geo_params, object_scenario, object_history);
  EXPECT_THAT(long_term_behavior_proto,
              ProtoPartialApproximatelyMatchesText(R"(
              average_speed: 10.0
              observation_duration: 1.9)",
                                                   1e-6));
  ObjectLongTermBehavior object_behavior;
  object_behavior.FromProto(long_term_behavior_proto);
  EXPECT_NEAR(object_behavior.avg_speed, vel, 1e-3);
  ObjectLongTermBehaviorProto object_behavior_proto;
  object_behavior.ToProto(&object_behavior_proto);
  EXPECT_THAT(long_term_behavior_proto, ProtoEq(object_behavior_proto));
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft
