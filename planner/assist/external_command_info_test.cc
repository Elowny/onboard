#include "onboard/planner/assist/external_command_info.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
TEST(ExternalCommandInfoTest, ProtoTest) {
  ExternalCommandStatus ext_cmd_status;
  ext_cmd_status.enable_pull_over = true;
  ext_cmd_status.enable_traffic_light_stopping = false;
  ext_cmd_status.enable_lc_objects = false;
  ext_cmd_status.override_left_blinker_on = true;
  ext_cmd_status.override_right_blinker_on = true;
  ext_cmd_status.override_emergency_blinker_on = true;
  ext_cmd_status.enable_stop_polyline_stopping = false;

  ext_cmd_status.override_door_open = true;

  ext_cmd_status.brake_to_stop = 1.0;
  ext_cmd_status.lcc_state = QLCCState::LCC_ENABLE;
  ext_cmd_status.alc_state = QALCState::ALC_ONGOING;
  ext_cmd_status.acc_state = QACCState::ACC_ENABLED;

  ext_cmd_status.lane_change_style = LaneChangeStyle::LC_STYLE_RADICAL;

  ext_cmd_status.following_distance_level =
      QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR;

  ext_cmd_status.lane_change_command = DriverAction::LC_CMD_LEFT;

  ext_cmd_status.output.cruising_speed_limit = 20.0;
  ext_cmd_status.output.current_lane_width = 3.5;
  ext_cmd_status.output.current_lane_length = 100.0;
  ext_cmd_status.output.current_lane_average_curvature_radius = 200.0;
  ext_cmd_status.output.distance_to_toll = 100.0;
  ext_cmd_status.output.distance_to_traffic_light = 10.0;
  ext_cmd_status.output.is_av_overlap_boundary = false;
  ext_cmd_status.output.is_valid_both_side_boundary = true;
  ext_cmd_status.output.lane_path_lost = false;
  ext_cmd_status.output.is_av_in_emergency_lane = true;
  ext_cmd_status.output.is_route_lane_change_fail = true;
  ext_cmd_status.output.planner_to_router_command =
      ExternalRouterCommand::ANOTHER_ROUTE;

  PlannerExternalCommandStatusProto proto = ext_cmd_status.ToProto();
  ExternalCommandStatus ext_cmd_status_from_proto;
  ext_cmd_status_from_proto.FromProto(proto);

  EXPECT_EQ(ext_cmd_status, ext_cmd_status_from_proto);
}

TEST(ExternalCommandInfoTest, ClearTest) {
  ExternalCommandQueue ext_cmd_queue;
  ext_cmd_queue.pending_driver_actions.push_back(DriverAction());
  ext_cmd_queue.pending_lane_change_requests.push(LaneChangeRequestProto());
  ext_cmd_queue.pending_out_of_blocked_road_requests.push(
      OutOfBlockedRoadRequestProto());

  EXPECT_FALSE(ext_cmd_queue.pending_driver_actions.empty());
  EXPECT_FALSE(ext_cmd_queue.pending_lane_change_requests.empty());
  EXPECT_FALSE(ext_cmd_queue.pending_out_of_blocked_road_requests.empty());

  ext_cmd_queue.Clear();

  EXPECT_TRUE(ext_cmd_queue.pending_driver_actions.empty());
  EXPECT_TRUE(ext_cmd_queue.pending_lane_change_requests.empty());
  EXPECT_TRUE(ext_cmd_queue.pending_out_of_blocked_road_requests.empty());
}

}  // namespace
}  // namespace qcraft::planner
