#include "onboard/planner/decision/turn_signal_decider.h"

#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

TEST(TurnSignalDeciderTest, BasicTest) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  // Build drive passage.
  const auto current_lane_path = mapping::LanePath(
      smm, /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, current_lane_path, /*step_s=*/1.0,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
      /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  // Calculate ego frenet box.
  const PoseProto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const Box2d ego_box =
      ComputeAvBox(Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
                   pose.yaw(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto ego_sl_box, drive_passage.QueryFrenetBoxAt(ego_box));

  // Teleop override turn signal test.
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;

    ExternalCommandStatus ext_cmd_status_1;
    ext_cmd_status_1.override_emergency_blinker_on = true;
    const auto turn_signal_result_1 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, ext_cmd_status_1,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_1.signal, TURN_SIGNAL_EMERGENCY);
    EXPECT_EQ(turn_signal_result_1.reason, TELEOP_TURN_SIGNAL);

    ExternalCommandStatus ext_cmd_status_2;
    ext_cmd_status_2.override_left_blinker_on = true;
    ext_cmd_status_2.override_right_blinker_on = true;
    const auto turn_signal_result_2 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, ext_cmd_status_2,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_2.signal, TURN_SIGNAL_EMERGENCY);
    EXPECT_EQ(turn_signal_result_2.reason, TELEOP_TURN_SIGNAL);

    ExternalCommandStatus ext_cmd_status_3;
    ext_cmd_status_3.override_left_blinker_on = true;
    const auto turn_signal_result_3 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, ext_cmd_status_3,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_3.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result_3.reason, TELEOP_TURN_SIGNAL);

    ExternalCommandStatus ext_cmd_status_4;
    ext_cmd_status_4.override_right_blinker_on = true;
    const auto turn_signal_result_4 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, ext_cmd_status_4,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_4.signal, TURN_SIGNAL_RIGHT);
    EXPECT_EQ(turn_signal_result_4.reason, TELEOP_TURN_SIGNAL);
  }

  // Route turn signal test.
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;
    const ExternalCommandStatus mock_ext_cmd_status;

    const auto turn_signal_result = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_LEFT,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result.reason, STARTING_TURN_SIGNAL);
  }
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;
    const ExternalCommandStatus mock_ext_cmd_status;

    const auto turn_signal_result = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_RIGHT,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_RIGHT);
    EXPECT_EQ(turn_signal_result.reason, PULL_OVER_TURN_SIGNAL);
  }

  // Lane change signal test.
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;
    const ExternalCommandStatus mock_ext_cmd_status;

    LaneChangeStateProto lc_state_1;
    lc_state_1.set_stage(LCS_PAUSE);
    lc_state_1.set_lc_left(true);
    const auto turn_signal_result_1 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, lc_state_1, mock_ext_cmd_status,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_1.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result_1.reason, LANE_CHANGE_TURN_SIGNAL);

    LaneChangeStateProto lc_state_2;
    lc_state_2.set_stage(LCS_PAUSE);
    lc_state_2.set_lc_left(false);
    const auto turn_signal_result_2 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, lc_state_2, mock_ext_cmd_status,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_2.signal, TURN_SIGNAL_RIGHT);
    EXPECT_EQ(turn_signal_result_2.reason, LANE_CHANGE_TURN_SIGNAL);

    const Box2d ego_box_lc_left =
        ComputeAvBox(Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y() - 2.5),
                     pose.yaw(), vehicle_geometry_params);
    ASSIGN_OR_DIE(const auto ego_sl_box_lc_left,
                  drive_passage.QueryFrenetBoxAt(ego_box_lc_left));
    const auto turn_signal_result_3 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box_lc_left, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_3.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result_3.reason, LANE_CHANGE_TURN_SIGNAL);

    const Box2d ego_box_lc_right =
        ComputeAvBox(Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y() + 2.5),
                     pose.yaw(), vehicle_geometry_params);
    ASSIGN_OR_DIE(const auto ego_sl_box_lc_right,
                  drive_passage.QueryFrenetBoxAt(ego_box_lc_right));
    const auto turn_signal_result_4 = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box_lc_right, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result_4.signal, TURN_SIGNAL_RIGHT);
    EXPECT_EQ(turn_signal_result_4.reason, LANE_CHANGE_TURN_SIGNAL);
  }

  // Planned signal test.
  {
    const LaneChangeStateProto mock_lc_state;
    const ExternalCommandStatus mock_ext_cmd_status;

    const TurnSignalResult planned_result = {
        .signal = TURN_SIGNAL_LEFT, .reason = PREPARE_LANE_CHANGE_TURN_SIGNAL};

    const auto turn_signal_result = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box, planned_result, pose);
    EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result.reason, PREPARE_LANE_CHANGE_TURN_SIGNAL);
  }

  // Prepare lane change turn signal test.
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;
    const ExternalCommandStatus mock_ext_cmd_status;

    const auto turn_signal_result = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_LEFT, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result.reason, PREPARE_LANE_CHANGE_TURN_SIGNAL);
  }
}

TEST(TurnSignalDeciderTest, ForkSignalTest) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  // Build drive passage.
  const mapping::LanePath current_lane_path(
      smm,
      {mapping::ElementId(155), mapping::ElementId(232),
       mapping::ElementId(262), mapping::ElementId(242)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, current_lane_path, /*step_s=*/1.0, /*avoid_loop=*/true,
      /*backward_extend_len=*/0.0, /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  const PoseProto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(305.0, 70.0), 0.0, Vec2d(9.0, 0.0));
  // Calculate ego frenet box.
  const Box2d ego_box(Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
                      pose.yaw(), /*length=*/4.2, /*width=*/2.1);
  ASSIGN_OR_DIE(const auto ego_sl_box, drive_passage.QueryFrenetBoxAt(ego_box));

  // Without red light.
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;
    const ExternalCommandStatus mock_ext_cmd_status;

    const auto turn_signal_result = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
        drive_passage, ego_sl_box, mock_planned_result, pose);
    EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_LEFT);
    EXPECT_EQ(turn_signal_result.reason, FORK_TURN_SIGNAL);
  }
  // With red light.
  {
    const LaneChangeStateProto mock_lc_state;
    const TurnSignalResult mock_planned_result;
    const ExternalCommandStatus mock_ext_cmd_status;

    const auto turn_signal_result = DecideTurnSignal(
        psmm, /*route_signal=*/TURN_SIGNAL_NONE,
        /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
        /*redlight_lane_id=*/current_lane_path.lane_id(1), mock_lc_state,
        mock_ext_cmd_status, drive_passage, ego_sl_box, mock_planned_result,
        pose);
    EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_NONE);
    EXPECT_EQ(turn_signal_result.reason, TURN_SIGNAL_OFF);
  }
}

TEST(TurnSignalDeciderTest, MergeSignalTest) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  // Build drive passage.
  const mapping::LanePath current_lane_path(
      smm, {mapping::ElementId(35404), mapping::ElementId(35398)},
      /*start_fraction=*/0.0, /*end_fraction=*/0.1);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, current_lane_path, /*step_s=*/1.0, /*avoid_loop=*/true,
      /*backward_extend_len=*/0.0, /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  const PoseProto pose = CreatePose(/*timestamp=*/10.0, Vec2d(-2966.0, -1686.0),
                                    -1.4, Vec2d(20.0, 0.0));
  // Calculate ego frenet box.
  const Box2d ego_box(Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
                      pose.yaw(), /*length=*/4.2, /*width=*/2.1);
  ASSIGN_OR_DIE(const auto ego_sl_box, drive_passage.QueryFrenetBoxAt(ego_box));

  const LaneChangeStateProto mock_lc_state;
  const TurnSignalResult mock_planned_result;
  const ExternalCommandStatus mock_ext_cmd_status;

  const auto turn_signal_result = DecideTurnSignal(
      psmm, /*route_signal=*/TURN_SIGNAL_NONE,
      /*pre_lane_change_signal=*/TURN_SIGNAL_NONE, current_lane_path,
      /*redlight_lane_id=*/std::nullopt, mock_lc_state, mock_ext_cmd_status,
      drive_passage, ego_sl_box, mock_planned_result, pose);
  EXPECT_EQ(turn_signal_result.signal, TURN_SIGNAL_LEFT);
  EXPECT_EQ(turn_signal_result.reason, MERGE_TURN_SIGNAL);
}

}  // namespace
}  // namespace qcraft::planner
