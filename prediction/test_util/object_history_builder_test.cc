#include "onboard/prediction/test_util/object_history_builder.h"

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(ObjectHistoryBuilderTest, CheckBuildResult) {
  const std::string id = "object_0";
  const int history_num = 10;
  const double vel = 5.0;
  auto object_history =
      BuildVehicleHistoryByConstVel(id, history_num, Vec2d::Zero(), vel);
  EXPECT_TRUE(object_history.UpdateStopTimeInfo());
  const auto stop_time_info = object_history.GetStopTimeInfo();
  const auto object_span = object_history.GetHistory();
  EXPECT_NEAR(object_span.heading(), 0.0, 1e-6);
  EXPECT_NEAR(object_span.v(), vel, 1e-6);
  EXPECT_EQ(object_span.GetHistoryFrom(0.5).id(), id);

  EXPECT_NEAR(object_span.bounding_box().length(), 4.0, 1e-6);
  EXPECT_NEAR(stop_time_info.last_time, 0.9, 1e-6);
  EXPECT_NEAR(stop_time_info.prev_stopped_since, 0.0, 1e-6);
  EXPECT_NEAR(stop_time_info.stopped_since, 0.9, 1e-6);
  EXPECT_NEAR(stop_time_info.last_moved_since, 0.9, 1e-6);
  EXPECT_EQ(object_history.id(), id);
  EXPECT_EQ(object_history.size(), history_num);
  EXPECT_EQ(object_history.timestamp(), 0.9);
  EXPECT_EQ(object_history.type(), OT_VEHICLE);
  EXPECT_THAT(object_history.object_proto(),
              ProtoPartialApproximatelyMatchesText(R"(
                                      pos { x: 4.5 y: 0.0 }
                                      vel { x: 5.0 y: 0.0 }
                                      accel { x: 0.0 y: 0.0 }
                                      timestamp: 0.9
                                      id: "object_0"
                                      yaw: 0.0
                                      yaw_rate: 0.0
                                      bounding_box {
                                        x: 4.5 
                                        y: 0.0
                                        heading: 0.0
                                        width: 2
                                        length: 4
                                      }
                                      bounding_box_source: FIERY_EYE_NET
                                      parked: false
                                      offroad: false)",
                                                   1e-6));
}

TEST(ObjectHistoryBuilderTest, CheckBuildResultWithType) {
  const std::string id = "object_0";
  const int history_num = 10;
  const double vel = 5.0;
  auto object_history =
      BuildHistoryByConstVel(id, history_num, Vec2d::Zero(), vel, OT_CYCLIST);
  EXPECT_TRUE(object_history.UpdateStopTimeInfo());
  const auto stop_time_info = object_history.GetStopTimeInfo();
  const auto object_span = object_history.GetHistory();
  EXPECT_NEAR(object_span.heading(), 0.0, 1e-6);
  EXPECT_NEAR(object_span.v(), vel, 1e-6);
  EXPECT_EQ(object_span.GetHistoryFrom(0.5).id(), id);

  EXPECT_NEAR(object_span.bounding_box().length(), 4.0, 1e-6);
  EXPECT_NEAR(stop_time_info.last_time, 0.9, 1e-6);
  EXPECT_NEAR(stop_time_info.prev_stopped_since, 0.0, 1e-6);
  EXPECT_NEAR(stop_time_info.stopped_since, 0.9, 1e-6);
  EXPECT_NEAR(stop_time_info.last_moved_since, 0.9, 1e-6);
  EXPECT_EQ(object_history.id(), id);
  EXPECT_EQ(object_history.type(), OT_CYCLIST);
  EXPECT_EQ(object_history.size(), history_num);
  EXPECT_EQ(object_history.timestamp(), 0.9);
  EXPECT_THAT(object_history.object_proto(),
              ProtoPartialApproximatelyMatchesText(R"(
                                      pos { x: 4.5 y: 0.0 }
                                      vel { x: 5.0 y: 0.0 }
                                      accel { x: 0.0 y: 0.0 }
                                      timestamp: 0.9
                                      id: "object_0"
                                      yaw: 0.0
                                      yaw_rate: 0.0
                                      bounding_box {
                                        x: 4.5 
                                        y: 0.0
                                        heading: 0.0
                                        width: 2
                                        length: 4
                                      }
                                      bounding_box_source: FIERY_EYE_NET
                                      parked: false
                                      offroad: false)",
                                                   1e-6));
}

TEST(BuildRawTrackerHistoryCTRA, BuildCorrectResult) {
  // Const V.
  const Vec2d start_pos(0.0, 0.0);
  const auto states = BuildRawTrackerHistoryCTRA(
      start_pos, /*vel=*/1.0, /*acc=*/0.0, /*heading=*/0.0, /*yaw_rate=*/0.0,
      /*length=*/5.0, /*width=*/2.0, /*history_num=*/21, /*start_ts=*/0.0,
      /*dt=*/0.1);
  const auto& last_state = states.back();
  EXPECT_NEAR(last_state.pos.x(), 2.0, 1e-6);
  EXPECT_NEAR(last_state.pos.y(), 0.0, 1e-6);

  // Decel acc.
  const auto decel_states = BuildRawTrackerHistoryCTRA(
      start_pos, /*vel=*/1.0, /*acc=*/-1.0, /*heading=*/0.0, /*yaw_rate=*/0.0,
      /*length=*/5.0, /*width=*/2.0, /*history_num=*/41, /*start_ts=*/0.0,
      /*dt=*/0.1);
  EXPECT_NEAR(decel_states.back().vel.Length(), 0.1, 1e-6);
}

TEST(ObjectHistoryBuilderTest, BuildVehicleMotionHistoryByConstVelTest) {
  const std::string id = "1";
  const int history_num = 21;
  const double vel = 1.0;
  const Vec2d start_pos(0.0, 0.0);
  const auto vehicle_motion =
      BuildVehicleMotionHistoryByConstVel(id, history_num, start_pos, vel);
  const auto& last_state = vehicle_motion.states.back();
  EXPECT_NEAR(last_state.pos.x(), 2.0, 1e-6);
  EXPECT_NEAR(last_state.pos.y(), 0.0, 1e-6);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
