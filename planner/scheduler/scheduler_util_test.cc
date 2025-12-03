#include "onboard/planner/scheduler/scheduler_util.h"

#include <memory>
#include <optional>

#include "absl/status/statusor.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

TEST(SchedulerUtil, MakeNoneLaneChangeStateTest) {
  const auto lc_state = MakeNoneLaneChangeState();

  EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_NONE);
  EXPECT_FALSE(lc_state.has_entered_target_lane());
  EXPECT_FALSE(lc_state.has_lc_left());
  EXPECT_FALSE(lc_state.has_force_merge());
}

TEST(SchedulerUtil, CalcAvhRefCenterL) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();

  const mapping::LanePath lane_path(
      &smm, {mapping::ElementId(939), mapping::ElementId(50)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(
      const auto passage,
      BuildDrivePassageFromLanePath(
          psmm, lane_path, /*step_s=*/1.0, /*avoid_loop=*/true,
          /*backward_extend_len=*/0.0, /*required_planning_horizon=*/0.0,
          /*required_backward_len=*/0.0,
          /*override_speed_limit_mps=*/std::nullopt));
  const FrenetBox av_frenet_box{
      .s_max = 4.0, .s_min = 0.0, .l_max = 1.0, .l_min = -1.0};
  // Not Smooth test.
  {
    EXPECT_NEAR(CalcAvhRefCenterL(psmm, passage, av_frenet_box,
                                  SmoothedReferenceLineResultMap(),
                                  /*should_smooth=*/false),
                0.0, 1e-1);
  }

  // Should smooth test.
  {
    const bool should_smooth =
        ShouldSmoothRefLane(TrafficLightInfoMap(), passage,
                            /*prev_smooth_state=*/false);
    EXPECT_TRUE(should_smooth);
  }
  // TODO(jiayu): Smooth test.
}

TEST(SchedulerUtil, MakeLaneChangeStateTest) {
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  const auto& psmm = CreateDojoTestPSMM();

  const mapping::LanePath left_lane_path(
      &smm, {mapping::ElementId(2475), mapping::ElementId(2490)},
      /*start_fraction=*/0.3, /*end_fraction=*/1.0);
  const mapping::LanePath left_extended_lane_path(
      &smm, {mapping::ElementId(2475), mapping::ElementId(2490)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  const mapping::LanePath mid_lane_path(
      &smm, {mapping::ElementId(2476), mapping::ElementId(2491)},
      /*start_fraction=*/0.3, /*end_fraction=*/1.0);
  const mapping::LanePath mid_extended_lane_path(
      &smm, {mapping::ElementId(2476), mapping::ElementId(2491)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  const mapping::LanePath right_lane_path(
      &smm, {mapping::ElementId(61), mapping::ElementId(498)},
      /*start_fraction=*/0.3, /*end_fraction=*/1.0);
  const mapping::LanePath right_extended_lane_path(
      &smm, {mapping::ElementId(61), mapping::ElementId(498)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Normal transitions.
  {
    const auto dp_or = BuildDrivePassage(
        psmm, /*vision_map_ptr=*/nullptr, mid_lane_path, mid_extended_lane_path,
        /*anchor_point=*/mapping::LanePoint(),
        /*planning_horizon=*/200.0, mid_lane_path.back());
    ASSERT_OK(dp_or);

    // Simple transitions.
    {
      // Lane keep.
      const Vec2d ego_pos(127.0, 65.9);
      const FrenetBox lk_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 0.4, .l_min = -1.6};
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lk_frenet_box, mid_lane_path,
          /*prev_lane_path_before_lc_from_start=*/mapping::LanePath(),
          MakeNoneLaneChangeState(),
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_NONE);
    }
    {
      // Single lane change, not entering target lane path.
      const Vec2d ego_pos(127.0, 64.5);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = -1.0, .l_min = -3.0};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, mid_lane_path, right_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);
      EXPECT_FALSE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
    {
      // Single lane change, entering target lane path.
      const Vec2d ego_pos(127.0, 65.7);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 0.2, .l_min = -1.8};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, mid_lane_path, right_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);
      EXPECT_TRUE(lc_state.crossed_boundary());
      EXPECT_TRUE(lc_state.entered_target_lane());
    }
    {
      // Recover from lc pause to lane change.
      const Vec2d ego_pos(127.0, 64.5);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = -1.0, .l_min = -3.0};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, mid_lane_path, right_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);
      EXPECT_FALSE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
    {
      // Recover from lc pause to lane change return.
      const Vec2d ego_pos(127.0, 64.5);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = -1.0, .l_min = -3.0};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, mid_lane_path, mid_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_RETURN);
      EXPECT_FALSE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }

    // Complex transitions.
    {
      // Lane change -> lane change return.
      const Vec2d ego_pos(127.0, 67.9);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 2.4, .l_min = 0.4};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, left_lane_path, mid_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_RETURN);
      EXPECT_TRUE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
    {
      // Lane change return -> lane change.
      const Vec2d ego_pos(127.0, 67.9);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 2.4, .l_min = 0.4};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_RETURN);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, left_lane_path, left_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);
      EXPECT_TRUE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
    {
      // Lane change pause -> lane change return.
      const Vec2d ego_pos(127.0, 67.9);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 2.4, .l_min = 0.4};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, left_lane_path, mid_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_RETURN);
      EXPECT_TRUE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
    {
      // Lane change pause -> lane change.
      const Vec2d ego_pos(127.0, 67.9);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 2.4, .l_min = 0.4};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_PAUSE);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, left_lane_path, left_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);
      EXPECT_TRUE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
  }
  // Jumping transitions.
  {
    const auto dp_or =
        BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr, right_lane_path,
                          right_extended_lane_path,
                          /*anchor_point=*/mapping::LanePoint(),
                          /*planning_horizon=*/200.0, right_lane_path.back());
    ASSERT_OK(dp_or);

    {
      // Lane change -> lane change.
      const Vec2d ego_pos(127.0, 68.2);
      const FrenetBox lc_frenet_box{
          .s_max = 4.0, .s_min = 0.0, .l_max = 6.2, .l_min = 4.2};
      LaneChangeStateProto prev_lc_state;
      prev_lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
      prev_lc_state.set_lc_left(true);
      prev_lc_state.set_entered_target_lane(false);
      const auto lc_state = *MakeLaneChangeState(
          *dp_or, ego_pos, lc_frenet_box, left_lane_path, mid_lane_path,
          prev_lc_state,
          /*ref_center_l=*/0.0, AutonomyStateProto::AUTO_DRIVE);
      EXPECT_EQ(lc_state.stage(), LaneChangeStage::LCS_EXECUTING);
      EXPECT_FALSE(lc_state.crossed_boundary());
      EXPECT_FALSE(lc_state.entered_target_lane());
    }
  }
}

}  // namespace
}  // namespace qcraft::planner
