#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"

#include <map>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

constexpr double kEpsilon = 0.1;  // m

// Build traffic light states proto.
TrafficLightStatesProto BuildTrafficLightStatesProto(
    const std::vector<mapping::ElementId>& traffic_light_ids,
    const std::vector<bool>& traffic_light_flashings,
    const std::vector<TrafficLightColor>& traffic_light_colors) {
  QCHECK_EQ(traffic_light_ids.size(), traffic_light_flashings.size());
  QCHECK_EQ(traffic_light_ids.size(), traffic_light_colors.size());

  TrafficLightStatesProto traffic_light_states;
  const int size = traffic_light_ids.size();
  for (int i = 0; i < size; i++) {
    auto* tl_state = traffic_light_states.add_states();
    tl_state->set_traffic_light_id(traffic_light_ids[i].value());
    tl_state->set_flashing(traffic_light_flashings[i]);
    tl_state->set_color(traffic_light_colors[i]);
  }

  return traffic_light_states;
}

// StarPointControlTest. Av before stop line.
TEST(TLInfoCollectorTest, StartPointControl_BeforeStopLine_Test) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct route sections.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  SendRouteSectionsAreaToCanvas(psmm, route_sections,
                                "startPointControl/straight/before_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{mapping::ElementId(891), mapping::ElementId(889)},
      /*traffic_light_flashings=*/{false, false}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_GREEN, TrafficLightColor::TL_RED});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));

  EXPECT_EQ(output.tl_info_map.size(), 3);
  for (const auto& [id, tl_info] : output.tl_info_map) {
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 69.943, kEpsilon);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::STRAIGHT);

    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
    EXPECT_EQ(iter->second.tl_id, mapping::ElementId(889));
  }
}

// StarPointControlTest. Av has passed stop line.
TEST(TLInfoCollectorTest, StartPointControl_PassedStopLine_Test) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct route sections.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(87.267, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  SendRouteSectionsAreaToCanvas(psmm, route_sections,
                                "startPointControl/straight/passed_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{mapping::ElementId(891), mapping::ElementId(889)},
      /*traffic_light_flashings=*/{false, false}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_GREEN, TrafficLightColor::TL_RED});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));

  EXPECT_EQ(output.tl_info_map.size(), 3);
  for (const auto& [id, tl_info] : output.tl_info_map) {
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), -17.192, kEpsilon);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::STRAIGHT);

    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
    EXPECT_EQ(iter->second.tl_id, mapping::ElementId(889));
  }
}

// TODO(jiayu): Check green flashing traffic light turn red left time.
// MultiControlPointTest. Av before first stop line.
TEST(TLInfoCollectorTest, MultiControlPoint_BeforeFirstStopLine_Test) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(561.585, -461.978), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToLanePoint(
      *smm, cc, sdc_pose, mapping::LanePoint(mapping::ElementId(6613), 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, *route_path);
  SendRouteSectionsAreaToCanvas(
      psmm, route_sections,
      "MultiControlPoint/turn_left/before_first_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{mapping::ElementId(6846),
                             mapping::ElementId(6845)},
      /*traffic_light_flashings=*/{false, true}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_RED, TrafficLightColor::TL_GREEN});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  const auto& tl_info_map = output.tl_info_map;

  EXPECT_EQ(tl_info_map.size(), 4);
  // Go straight lane 6825.
  {
    const auto iter_tl_info = tl_info_map.find(mapping::ElementId(6825));
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto& tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::LEFT_WAITING_AREA);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::LEFT_TURN);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 50.408, kEpsilon);
    EXPECT_NEAR(tl_info.control_point_relative_s().back(), 62.006, kEpsilon);
    const auto iter_straight =
        tl_info.tls().find(TrafficLightDirection::STRAIGHT);
    EXPECT_EQ(iter_straight->second.tl_id, mapping::ElementId(6846));
    EXPECT_EQ(iter_straight->second.tl_state, TrafficLightState::TL_STATE_RED);

    const auto iter_left = tl_info.tls().find(TrafficLightDirection::LEFT);
    EXPECT_EQ(iter_left->second.tl_id, mapping::ElementId(6845));
    EXPECT_EQ(iter_left->second.tl_state,
              TrafficLightState::TL_STATE_GREEN_FLASHING);
  }

  // Go straight lane 6816.
  {
    const auto iter_tl_info = tl_info_map.find(mapping::ElementId(6816));
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto& tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::STRAIGHT);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 50.408, kEpsilon);
    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_id, mapping::ElementId(6846));
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
  }

  // UTurn lane 6830.
  {
    const auto iter_tl_info = tl_info_map.find(mapping::ElementId(6830));
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto& tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::UTURN);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 50.408, kEpsilon);
    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_id, mapping::ElementId(6845));
    EXPECT_EQ(iter->second.tl_state,
              TrafficLightState::TL_STATE_GREEN_FLASHING);
  }

  // Go Straight lane 6817.
  {
    const auto iter_tl_info = tl_info_map.find(mapping::ElementId(6817));
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto& tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_EQ(tl_info.direction(), mapping::LaneProto::STRAIGHT);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 50.408, kEpsilon);
    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_id, mapping::ElementId(6846));
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
  }
}

TEST(TLInfoCollectorTest, EmptyTest) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(412.277, 209.654), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "hw_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  SendRouteSectionsAreaToCanvas(psmm, route_sections, "EmptyTest");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{},
      /*traffic_light_flashings=*/{}, /*traffic_light_colors=*/
      {});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  EXPECT_TRUE(output.tl_info_map.empty());
}

// MultiControlPointTest. AV before second stop line, has passed first stop
// line.
TEST(TLInfoCollectorTest, MultiControlPoint_BeforeSecondStopline_Test) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Construct route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(618.370, -461.186), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToLanePoint(
      *smm, cc, sdc_pose, mapping::LanePoint(mapping::ElementId(6613), 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, *route_path);
  SendRouteSectionsAreaToCanvas(
      psmm, route_sections,
      "MultiControlPoint/turn_left/before_second_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{mapping::ElementId(6846),
                             mapping::ElementId(6845)},
      /*traffic_light_flashings=*/{false, false}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_RED, TrafficLightColor::TL_GREEN});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  const auto& tl_info_map = output.tl_info_map;

  EXPECT_EQ(tl_info_map.size(), 1);
  const auto iter_tl_info = tl_info_map.find(mapping::ElementId(6825));
  EXPECT_TRUE(iter_tl_info != tl_info_map.end());
  const auto& tl_info = iter_tl_info->second;
  EXPECT_EQ(tl_info.tl_control_type(),
            TrafficLightControlType::LEFT_WAITING_AREA);
  EXPECT_FALSE(tl_info.can_go_on_red());
  EXPECT_EQ(tl_info.direction(), mapping::LaneProto::LEFT_TURN);
  EXPECT_NEAR(tl_info.control_point_relative_s().front(), -6.425, kEpsilon);
  EXPECT_NEAR(tl_info.control_point_relative_s().back(), 5.172, kEpsilon);
  const auto iter_straight =
      tl_info.tls().find(TrafficLightDirection::STRAIGHT);
  EXPECT_EQ(iter_straight->second.tl_id, mapping::ElementId(6846));
  EXPECT_EQ(iter_straight->second.tl_state, TrafficLightColor::TL_RED);

  const auto iter_left = tl_info.tls().find(TrafficLightDirection::LEFT);
  EXPECT_EQ(iter_left->second.tl_id, mapping::ElementId(6845));
  EXPECT_EQ(iter_left->second.tl_state, TrafficLightState::TL_STATE_GREEN);
}

}  // namespace
}  // namespace qcraft::planner
