
#include "onboard/planner/decision/traffic_light/traffic_light_decider.h"

#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

TlInfo MockSingleDirectionTlInfo(TrafficLightState tl_state, double relative_s,
                                 double estimated_turn_red_time_left,
                                 std::optional<double> countdown) {
  const mapping::ElementId lane_id(34);
  std::vector<double> control_point_relative_s = {relative_s};
  const bool can_go_on_red = false;
  const bool is_fresh = true;
  const std::string last_error_msg = "None.";
  const auto direction = mapping::LaneProto::STRAIGHT;
  absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tl_info_map = {
      {TrafficLightDirection::UNMARKED,
       SingleTlInfo{
           .tl_id = mapping::ElementId(889),
           .tl_state = tl_state,
           .estimated_turn_red_time_left = estimated_turn_red_time_left,
           .countdown = countdown}}};

  return TlInfo(lane_id, std::move(control_point_relative_s), can_go_on_red,
                std::move(tl_info_map), is_fresh, last_error_msg, direction);
}

TlInfo MockLeftWaitingAreaTlInfo(
    absl::Span<const TrafficLightState> tl_state_vec,
    absl::Span<const double> relative_s_vec,
    absl::Span<const double> estimated_turn_red_time_left_vec,
    absl::Span<const std::optional<double>> countdown_vec) {
  QCHECK_EQ(tl_state_vec.size(), 2);
  QCHECK_EQ(relative_s_vec.size(), 2);
  QCHECK_EQ(estimated_turn_red_time_left_vec.size(), 2);
  QCHECK_EQ(countdown_vec.size(), 2);

  const mapping::ElementId lane_id(6825);
  std::vector<double> control_point_relative_s = {relative_s_vec.front(),
                                                  relative_s_vec.back()};
  const bool can_go_on_red = false;
  const bool is_fresh = true;
  const std::string last_error_msg = "None.";
  const auto direction = mapping::LaneProto::STRAIGHT;
  absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tl_info_map = {
      {TrafficLightDirection::STRAIGHT,
       SingleTlInfo{.tl_id = mapping::ElementId(6846),
                    .tl_state = tl_state_vec.front(),
                    .estimated_turn_red_time_left =
                        estimated_turn_red_time_left_vec.front(),
                    .countdown = countdown_vec.front()}},
      {TrafficLightDirection::LEFT,
       SingleTlInfo{.tl_id = mapping::ElementId(6845),
                    .tl_state = tl_state_vec.back(),
                    .estimated_turn_red_time_left =
                        estimated_turn_red_time_left_vec.back(),
                    .countdown = countdown_vec.back()}}};

  return TlInfo(lane_id, std::move(control_point_relative_s), can_go_on_red,
                std::move(tl_info_map), is_fresh, last_error_msg, direction);
}

// NOLINTNEXTLINE
TEST(TrafficLightDeciderTest, SingleTlDecisionTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  const auto& psmm = CreateDojoTestPSMM();

  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  // Red light test.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(19.62);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_RED,
                                   /*relative_s*/ 50.0,
                                   /*estimated_turn_red_time_left*/ 0.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_EQ(stop_lines.size(), 1);
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_FALSE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Green light test.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(19.62);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_GREEN,
                                   /*relative_s*/ 50.0,
                                   /*estimated_turn_red_time_left*/ 0.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Green flashing light test,
  // Av can stop before intersection.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(19.62);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_GREEN_FLASHING,
                                   /*relative_s*/ 50.0,
                                   /*estimated_turn_red_time_left*/ 3.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_last_tl_proceed(true);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_EQ(stop_lines.size(), 1);
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_FALSE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Green flashing light test.
  // Av can't stop before intersection, but av can cross stop line before
  // traffic light turn red.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(49.62);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_GREEN_FLASHING,
                                   /*relative_s*/ 20.0,
                                   /*estimated_turn_red_time_left*/ 3.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_last_tl_proceed(true);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Yellow light test.
  // Av can't stop before intersection comfortably, and av can't emergency stop
  // before intersection.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(59.62);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_YELLOW,
                                   /*relative_s*/ 10.0,
                                   /*estimated_turn_red_time_left*/ 1.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_last_tl_proceed(true);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Unknown light test.
  // Has received colored traffic lights.
  // Last decision is pass.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(59.62);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_UNKNOWN,
                                   /*relative_s*/ 10.0,
                                   /*estimated_turn_red_time_left*/ 1.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_last_tl_proceed(true);
    decider_state_proto.set_has_received_known_traffic_light(true);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Unknown light test.
  // Has not received colored traffic lights.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(9.7);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_UNKNOWN,
                                   /*relative_s*/ 60.0,
                                   /*estimated_turn_red_time_left*/ 1.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_has_received_known_traffic_light(false);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_FALSE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Unknown light test.
  // Has not received colored traffic lights.
  // Av speed is too fast to stop comfortably before stop line.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(9.7);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(15.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_UNKNOWN,
                                   /*relative_s*/ 60.0,
                                   /*estimated_turn_red_time_left*/ 1.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_has_received_known_traffic_light(false);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_EQ(stop_lines.size(), 1);
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_FALSE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_FALSE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Av has been passed stop line test.
  // The traffic light is green.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(77.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(5.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_GREEN,
                                   /*relative_s*/ -7.0,
                                   /*estimated_turn_red_time_left*/ 1.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }

  // Av has been passed stop line test.
  // The traffic light is red, and the last decision is stop.
  {
    // Plan start point.
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(75.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(5.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    // Traffic light info map.
    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(34),
         MockSingleDirectionTlInfo(TrafficLightState::TL_STATE_RED,
                                   /*relative_s*/ -5.0,
                                   /*estimated_turn_red_time_left*/ 1.0,
                                   /*countdown*/ std::nullopt)}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(34);
    decider_state_proto.set_last_tl_proceed(false);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_EQ(stop_lines.size(), 1);
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_FALSE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_TRUE(state_proto.has_received_known_traffic_light());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 34);
  }
}

// NOLINTNEXTLINE
TEST(TrafficLightDeciderTest, MultiTlDecisionTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  const auto& psmm = CreateDojoTestPSMM();

  const auto current_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(6777), mapping::ElementId(6779),
                         mapping::ElementId(6825), mapping::ElementId(10709)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  // Left is green, straight is red, first stop line is ahead.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(591.921);
    plan_start_point.mutable_path_point()->set_y(-461.976);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(6825),
         MockLeftWaitingAreaTlInfo(
             /*tl_state_vec*/ {TrafficLightState::TL_STATE_RED,
                               TrafficLightState::TL_STATE_GREEN},
             /*relative_s_vec*/ {20.0, 32.0},
             /*estimated_turn_red_time_left_vec*/ {0.0, 0.0},
             /*countdown_vec*/ {std::nullopt, std::nullopt})}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(6825);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 6825);
  }

  // Left is green, straight is red, first stop line is behind, second stop line
  // is ahead.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(616.733);
    plan_start_point.mutable_path_point()->set_y(-461.488);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    const TrafficLightInfoMap tl_info_map = {
        {mapping::ElementId(6825),
         MockLeftWaitingAreaTlInfo(
             /*tl_state_vec*/ {TrafficLightState::TL_STATE_RED,
                               TrafficLightState::TL_STATE_GREEN},
             /*relative_s_vec*/ {-5.2, 6.8},
             /*estimated_turn_red_time_left_vec*/ {0.0, 0.0},
             /*countdown_vec*/ {std::nullopt, std::nullopt})}};

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(6825);

    ASSIGN_OR_DIE(
        auto output,
        BuildTrafficLightConstraints(
            psmm, vehicle_geometry_params, plan_start_point, drive_passage,
            current_lane_path, /*s_offset*/ 0.0, tl_info_map,
            preliminary_speed_profile, decider_state_proto));
    const auto stop_lines = std::move(output.stop_lines);
    const auto speed_profiles = std::move(output.speed_profiles);
    const auto state_proto = std::move(output.traffic_light_decider_state);

    EXPECT_TRUE(stop_lines.empty());
    EXPECT_TRUE(speed_profiles.empty());
    EXPECT_TRUE(state_proto.last_tl_proceed());
    EXPECT_TRUE(state_proto.entry_with_left_light_not_red());
    EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 6825);
  }

  // Left is red/unknown, straight is green/green flashing/yellow, first stop
  // line is ahead.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(591.921);
    plan_start_point.mutable_path_point()->set_y(-461.976);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(6825);

    for (const auto left_tl_state : {TrafficLightState::TL_STATE_RED,
                                     TrafficLightState::TL_STATE_UNKNOWN}) {
      for (const auto straight_tl_state :
           {TrafficLightState::TL_STATE_GREEN,
            TrafficLightState::TL_STATE_YELLOW,
            TrafficLightState::TL_STATE_GREEN_FLASHING}) {
        const TrafficLightInfoMap tl_info_map = {
            {mapping::ElementId(6825),
             MockLeftWaitingAreaTlInfo(
                 /*tl_state_vec*/
                 {straight_tl_state, left_tl_state},
                 /*relative_s_vec*/ {20.0, 32.0},
                 /*estimated_turn_red_time_left_vec*/ {0.0, 0.0},
                 /*countdown_vec*/ {std::nullopt, std::nullopt})}};

        ASSIGN_OR_DIE(
            auto output,
            BuildTrafficLightConstraints(
                psmm, vehicle_geometry_params, plan_start_point, drive_passage,
                current_lane_path, /*s_offset*/ 0.0, tl_info_map,
                preliminary_speed_profile, decider_state_proto));
        const auto stop_lines = std::move(output.stop_lines);
        const auto speed_profiles = std::move(output.speed_profiles);
        const auto state_proto = std::move(output.traffic_light_decider_state);

        EXPECT_EQ(stop_lines.size(), 1);
        EXPECT_TRUE(speed_profiles.empty());
        EXPECT_FALSE(state_proto.last_tl_proceed());
        EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
        EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 6825);

        const auto& stop_line = stop_lines.front();
        EXPECT_NEAR(stop_line.s(), 32.0, 1e-1);
      }
    }
  }

  // Left is red/unknown, straight is red/unknown, first stop line is ahead.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(591.921);
    plan_start_point.mutable_path_point()->set_y(-461.976);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(6825);

    for (const auto left_tl_state : {TrafficLightState::TL_STATE_RED,
                                     TrafficLightState::TL_STATE_UNKNOWN}) {
      for (const auto straight_tl_state :
           {TrafficLightState::TL_STATE_RED,
            TrafficLightState::TL_STATE_UNKNOWN}) {
        const TrafficLightInfoMap tl_info_map = {
            {mapping::ElementId(6825),
             MockLeftWaitingAreaTlInfo(
                 /*tl_state_vec*/
                 {straight_tl_state, left_tl_state},
                 /*relative_s_vec*/ {20.0, 32.0},
                 /*estimated_turn_red_time_left_vec*/ {0.0, 0.0},
                 /*countdown_vec*/ {std::nullopt, std::nullopt})}};

        ASSIGN_OR_DIE(
            auto output,
            BuildTrafficLightConstraints(
                psmm, vehicle_geometry_params, plan_start_point, drive_passage,
                current_lane_path, /*s_offset*/ 0.0, tl_info_map,
                preliminary_speed_profile, decider_state_proto));
        const auto stop_lines = std::move(output.stop_lines);
        const auto speed_profiles = std::move(output.speed_profiles);
        const auto state_proto = std::move(output.traffic_light_decider_state);

        EXPECT_EQ(stop_lines.size(), 1);
        EXPECT_TRUE(speed_profiles.empty());
        EXPECT_FALSE(state_proto.last_tl_proceed());
        EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
        EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 6825);

        const auto& stop_line = stop_lines.front();
        EXPECT_NEAR(stop_line.s(), 20.0, 1e-1);
      }
    }
  }

  // Left is red/unknown, whatever straight light is, first stop line is behind,
  // second stop is ahead, entry left waiting area when left is green.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(616.733);
    plan_start_point.mutable_path_point()->set_y(-461.488);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(6825);
    decider_state_proto.set_entry_with_left_light_not_red(true);

    for (const auto left_tl_state : {TrafficLightState::TL_STATE_RED,
                                     TrafficLightState::TL_STATE_UNKNOWN}) {
      for (const auto straight_tl_state :
           {TrafficLightState::TL_STATE_RED,
            TrafficLightState::TL_STATE_UNKNOWN,
            TrafficLightState::TL_STATE_GREEN,
            TrafficLightState::TL_STATE_YELLOW,
            TrafficLightState::TL_STATE_GREEN_FLASHING,
            TrafficLightState::TL_STATE_YELLOW_FLASHING}) {
        const TrafficLightInfoMap tl_info_map = {
            {mapping::ElementId(6825),
             MockLeftWaitingAreaTlInfo(
                 /*tl_state_vec*/
                 {straight_tl_state, left_tl_state},
                 /*relative_s_vec*/ {-5.2, 6.8},
                 /*estimated_turn_red_time_left_vec*/ {0.0, 0.0},
                 /*countdown_vec*/ {std::nullopt, std::nullopt})}};

        ASSIGN_OR_DIE(
            auto output,
            BuildTrafficLightConstraints(
                psmm, vehicle_geometry_params, plan_start_point, drive_passage,
                current_lane_path, /*s_offset*/ 0.0, tl_info_map,
                preliminary_speed_profile, decider_state_proto));
        const auto stop_lines = std::move(output.stop_lines);
        const auto speed_profiles = std::move(output.speed_profiles);
        const auto state_proto = std::move(output.traffic_light_decider_state);

        EXPECT_TRUE(stop_lines.empty());
        EXPECT_TRUE(speed_profiles.empty());
        EXPECT_TRUE(state_proto.last_tl_proceed());
        EXPECT_TRUE(state_proto.entry_with_left_light_not_red());
        EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 6825);
      }
    }
  }

  // Left is red/unknown, whatever straight light is, first stop line is behind,
  // second stop is ahead, entry left waiting area when left isn't green.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(616.733);
    plan_start_point.mutable_path_point()->set_y(-461.488);
    plan_start_point.set_v(10.0);

    const SpeedProfile preliminary_speed_profile =
        CreateSpeedProfile(plan_start_point.v(), drive_passage,
                           /*speed_zones*/ {}, /*stop_points*/ {});

    TrafficLightDeciderStateProto decider_state_proto;
    decider_state_proto.set_last_tl_decision_lane_id(6825);

    for (const auto left_tl_state : {TrafficLightState::TL_STATE_RED,
                                     TrafficLightState::TL_STATE_UNKNOWN}) {
      for (const auto straight_tl_state :
           {TrafficLightState::TL_STATE_RED,
            TrafficLightState::TL_STATE_UNKNOWN,
            TrafficLightState::TL_STATE_GREEN,
            TrafficLightState::TL_STATE_YELLOW,
            TrafficLightState::TL_STATE_GREEN_FLASHING,
            TrafficLightState::TL_STATE_YELLOW_FLASHING}) {
        const TrafficLightInfoMap tl_info_map = {
            {mapping::ElementId(6825),
             MockLeftWaitingAreaTlInfo(
                 /*tl_state_vec*/
                 {straight_tl_state, left_tl_state},
                 /*relative_s_vec*/ {-5.2, 6.8},
                 /*estimated_turn_red_time_left_vec*/ {0.0, 0.0},
                 /*countdown_vec*/ {std::nullopt, std::nullopt})}};

        ASSIGN_OR_DIE(
            auto output,
            BuildTrafficLightConstraints(
                psmm, vehicle_geometry_params, plan_start_point, drive_passage,
                current_lane_path, /*s_offset*/ 0.0, tl_info_map,
                preliminary_speed_profile, decider_state_proto));
        const auto stop_lines = std::move(output.stop_lines);
        const auto speed_profiles = std::move(output.speed_profiles);
        const auto state_proto = std::move(output.traffic_light_decider_state);

        EXPECT_EQ(stop_lines.size(), 1);
        EXPECT_TRUE(speed_profiles.empty());
        EXPECT_FALSE(state_proto.last_tl_proceed());
        EXPECT_FALSE(state_proto.entry_with_left_light_not_red());
        EXPECT_EQ(state_proto.last_tl_decision_lane_id(), 6825);

        const auto& stop_line = stop_lines.front();
        EXPECT_NEAR(stop_line.s(), 6.8, 1e-1);
      }
    }
  }
}
}  // namespace
}  // namespace qcraft::planner
