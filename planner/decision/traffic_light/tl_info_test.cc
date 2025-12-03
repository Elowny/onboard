#include "onboard/planner/decision/traffic_light/tl_info.h"

#include "gtest/gtest.h"
namespace qcraft::planner {

namespace {

TlInfo MockSingleDirectionTlInfo(TrafficLightState tl_state) {
  const mapping::ElementId lane_id(940);
  const std::vector<double> control_point_relative_s = {50.0};
  const bool can_go_on_red = false;
  const bool is_fresh = true;
  const std::string last_error_msg = "None.";
  const auto direction = mapping::LaneProto::STRAIGHT;
  absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tl_info_map = {
      {TrafficLightDirection::UNMARKED,
       SingleTlInfo{.tl_id = mapping::ElementId(890),
                    .tl_state = tl_state,
                    .estimated_turn_red_time_left = 0.0,
                    .countdown = 5.0}}};

  return TlInfo(lane_id, control_point_relative_s, can_go_on_red,
                std::move(tl_info_map), is_fresh, last_error_msg, direction);
}

void TestTlColorAndFlashing(const SingleTrafficLightInfoProto& proto,
                            TrafficLightState tl_state) {
  switch (tl_state) {
    case TrafficLightState::TL_STATE_UNKNOWN:
      EXPECT_EQ(proto.color(), TlColor::TL_COLOR_UNKNOWN);
      EXPECT_FALSE(proto.flashing());
      return;
    case TrafficLightState::TL_STATE_RED:
      EXPECT_EQ(proto.color(), TlColor::TL_COLOR_RED);
      EXPECT_FALSE(proto.flashing());
      return;
    case TrafficLightState::TL_STATE_GREEN:
      EXPECT_EQ(proto.color(), TlColor::TL_COLOR_GREEN);
      EXPECT_FALSE(proto.flashing());
      return;
    case TrafficLightState::TL_STATE_YELLOW:
      EXPECT_EQ(proto.color(), TlColor::TL_COLOR_YELLOW);
      EXPECT_FALSE(proto.flashing());
      return;
    case TrafficLightState::TL_STATE_GREEN_FLASHING:
      EXPECT_EQ(proto.color(), TlColor::TL_COLOR_GREEN);
      EXPECT_TRUE(proto.flashing());
      return;
    case TrafficLightState::TL_STATE_YELLOW_FLASHING:
      EXPECT_EQ(proto.color(), TlColor::TL_COLOR_YELLOW);
      EXPECT_TRUE(proto.flashing());
      return;
  }
}

TEST(TlInfoTest, SingleLightTest) {
  for (const auto tl_state :
       {TrafficLightState::TL_STATE_RED, TrafficLightState::TL_STATE_GREEN,
        TrafficLightState::TL_STATE_YELLOW,
        TrafficLightState::TL_STATE_GREEN_FLASHING,
        TrafficLightState::TL_STATE_YELLOW_FLASHING}) {
    const auto tl_info = MockSingleDirectionTlInfo(tl_state);

    EXPECT_EQ(tl_info.lane_id(), mapping::ElementId(940));
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_TRUE(tl_info.is_fresh());
    EXPECT_EQ(tl_info.last_error_msg(), "None.");
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.empty());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::STRAIGHT);
    EXPECT_EQ(tl_info.last_error_msg(), "None.");
    EXPECT_FALSE(tl_info.DebugString().empty());

    TrafficLightInfoProto proto;
    tl_info.ToProto(&proto);
    EXPECT_EQ(proto.lane_id(), 940);
    EXPECT_EQ(proto.direction(), mapping::LaneProto::STRAIGHT);
    EXPECT_TRUE(proto.has_isolated_tl_info());
    const auto& isolated_tl_info = proto.isolated_tl_info();
    EXPECT_EQ(isolated_tl_info.traffic_light_id(), 890);
    EXPECT_EQ(isolated_tl_info.turn_red_time_left(), 0.0);
    EXPECT_EQ(isolated_tl_info.control_point_relative_s(), 50.0);
    EXPECT_EQ(isolated_tl_info.countdown(), 5.0);
    TestTlColorAndFlashing(isolated_tl_info, tl_state);
  }
}

TEST(TlInfoTest, MultiLightTest) {
  const mapping::ElementId lane_id(6825);
  const std::vector<double> control_point_relative_s = {70.0, 80.0};
  const bool can_go_on_red = false;
  const bool is_fresh = true;
  const std::string last_error_msg = "None.";
  const auto direction = mapping::LaneProto::STRAIGHT;
  absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tl_info_map = {
      {TrafficLightDirection::STRAIGHT,
       SingleTlInfo{.tl_id = mapping::ElementId(6846),
                    .tl_state = TrafficLightState::TL_STATE_RED,
                    .estimated_turn_red_time_left = 0.0,
                    .countdown = 5.0}},
      {TrafficLightDirection::LEFT,
       SingleTlInfo{.tl_id = mapping::ElementId(6845),
                    .tl_state = TrafficLightState::TL_STATE_RED,
                    .estimated_turn_red_time_left = 0.0,
                    .countdown = 5.0}}};

  TlInfo tl_info(lane_id, control_point_relative_s, can_go_on_red,
                 std::move(tl_info_map), is_fresh, last_error_msg, direction);

  EXPECT_EQ(tl_info.lane_id(), lane_id);
  EXPECT_FALSE(tl_info.can_go_on_red());
  EXPECT_TRUE(tl_info.is_fresh());
  EXPECT_EQ(tl_info.last_error_msg(), "None.");
  EXPECT_EQ(tl_info.tl_control_type(),
            TrafficLightControlType::LEFT_WAITING_AREA);
  EXPECT_FALSE(tl_info.empty());
  EXPECT_EQ(tl_info.direction(), mapping::LaneProto::STRAIGHT);
  EXPECT_EQ(tl_info.last_error_msg(), "None.");
  EXPECT_FALSE(tl_info.DebugString().empty());

  TrafficLightInfoProto proto;
  tl_info.ToProto(&proto);
  EXPECT_EQ(proto.lane_id(), 6825);
  EXPECT_EQ(proto.direction(), mapping::LaneProto::STRAIGHT);

  EXPECT_TRUE(proto.has_straight_tl_info());
  const auto& straight_tl_info = proto.straight_tl_info();
  EXPECT_EQ(straight_tl_info.traffic_light_id(), 6846);
  EXPECT_EQ(straight_tl_info.control_point_relative_s(), 70.0);
  EXPECT_EQ(straight_tl_info.countdown(), 5.0);
  TestTlColorAndFlashing(straight_tl_info, TrafficLightState::TL_STATE_RED);

  EXPECT_TRUE(proto.has_left_tl_info());
  const auto& left_tl_info = proto.left_tl_info();
  EXPECT_EQ(left_tl_info.traffic_light_id(), 6845);
  EXPECT_EQ(left_tl_info.control_point_relative_s(), 80.0);
  EXPECT_EQ(left_tl_info.countdown(), 5.0);
  TestTlColorAndFlashing(left_tl_info, TrafficLightState::TL_STATE_RED);
}

}  // namespace
}  // namespace qcraft::planner
