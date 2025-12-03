#include "onboard/planner/assist/assist_util.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/remote_assist_common.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

std::vector<ApolloTrajectoryPointProto> MakeStraightTrajectory(
    const Vec2d& start, const Vec2d& end, int num_pt) {
  const Vec2d step = (end - start) / (num_pt - 1);
  const double step_s = step.norm();
  const double heading = (end - start).FastAngle();
  const double v = step_s / kTrajectoryTimeStep;

  std::vector<ApolloTrajectoryPointProto> traj_pts;
  traj_pts.reserve(num_pt);

  Vec2d prev_pt = start - step;
  double prev_s = -step_s;
  for (int i = 0; i < num_pt; ++i) {
    ApolloTrajectoryPointProto pt;
    pt.mutable_path_point()->set_x(prev_pt.x() + step.x());
    pt.mutable_path_point()->set_y(prev_pt.y() + step.y());
    pt.mutable_path_point()->set_theta(heading);
    pt.mutable_path_point()->set_s(prev_s += step_s);
    pt.set_v(v);
    pt.set_a(0.0);
    pt.set_j(0.0);
    pt.set_relative_time(i * kTrajectoryTimeStep);

    traj_pts.push_back(std::move(pt));
    prev_pt += step;
  }
  return traj_pts;
}

TEST(AssitUtilTest, AlignLanePathToThisFrame) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  // Construct prev target lane path.
  PoseProto prev_pose;
  prev_pose.mutable_pos_smooth()->set_x(30.0);
  prev_pose.mutable_pos_smooth()->set_y(0.0);
  constexpr double kAlccTargetLanePathRequiredLength = 200.0;  // m.
  ASSIGN_OR_DIE(const auto prev_lane_path,
                FindNearestLanePathFromEgoPose(
                    prev_pose, psmm, kAlccTargetLanePathRequiredLength));

  ASSIGN_OR_DIE(const auto extend_lane_path,
                ForwardExtendLanePathWithMinimumHeadingDiff(
                    psmm, prev_lane_path,
                    kAlccTargetLanePathRequiredLength - prev_lane_path.length(),
                    /*allow_virtual=*/true));

  ASSIGN_OR_DIE(const auto dm, BuildDrivingMapByRouteOnOfflineMap(
                                   psmm, RouteSections::BuildFromLanePath(
                                             psmm, extend_lane_path)));

  // Update target lane path from start.
  PoseProto cur_pose;
  cur_pose.mutable_pos_smooth()->set_x(50.0);
  cur_pose.mutable_pos_smooth()->set_y(0.0);
  ASSIGN_OR_DIE(
      const auto target_lane_paths,
      AlignLanePathToThisFrame(
          psmm, dm, prev_lane_path, cur_pose, kAlccTargetLanePathRequiredLength,
          kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          kDrivePassageKeepBehindLength));
  const auto& [target_lane_path_from_start, target_lane_path_with_behind] =
      target_lane_paths;

  // Test target lane path from start.
  {
    EXPECT_THAT(
        std::vector<mapping::ElementId>(
            {mapping::ElementId(2448), mapping::ElementId(1),
             mapping::ElementId(34), mapping::ElementId(2471)}),
        testing::ElementsAreArray(target_lane_path_from_start.lane_ids()));
    EXPECT_NEAR(target_lane_path_from_start.start_fraction(), 0.892640, 1e-3);
    EXPECT_NEAR(target_lane_path_from_start.end_fraction(), 0.961707, 1e-3);
    EXPECT_NEAR(target_lane_path_from_start.length(), 180, 1e-1);
  }

  // Test target lane path with behind.
  {
    EXPECT_THAT(
        std::vector<mapping::ElementId>(
            {mapping::ElementId(2448), mapping::ElementId(1),
             mapping::ElementId(34), mapping::ElementId(2471)}),
        testing::ElementsAreArray(target_lane_path_with_behind.lane_ids()));
    EXPECT_NEAR(target_lane_path_with_behind.start_fraction(), 0.714533, 1e-3);
    EXPECT_NEAR(target_lane_path_with_behind.end_fraction(), 0.961707, 1e-3);
    EXPECT_NEAR(target_lane_path_with_behind.length(), 190.0, 1e-1);
  }
}

TEST(AssistUtilTest, ConvertToFollowHeadwayTime) {
  EXPECT_NEAR(
      ConvertToFollowHeadwayTime(QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE),
      1.7, 1e-3);
  EXPECT_NEAR(
      ConvertToFollowHeadwayTime(QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR),
      1.2, 1e-3);
  EXPECT_NEAR(ConvertToFollowHeadwayTime(
                  QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR),
              1.4, 1e-3);
  EXPECT_NEAR(ConvertToFollowHeadwayTime(
                  QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM),
              1.7, 1e-3);
  EXPECT_NEAR(ConvertToFollowHeadwayTime(
                  QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR),
              2.0, 1e-3);
  EXPECT_NEAR(
      ConvertToFollowHeadwayTime(QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR),
      2.4, 1e-3);
}

TEST(AssistUtilTest, GetNextFollowingDistanceLevel) {
  EXPECT_EQ(GetNextFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR);
  EXPECT_EQ(GetNextFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR);
  EXPECT_EQ(GetNextFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM);
  EXPECT_EQ(GetNextFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR);
  EXPECT_EQ(GetNextFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR);
  EXPECT_EQ(GetNextFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR);
}

TEST(AssistUtilTest, GetPreviousFollowingDistanceLevel) {
  EXPECT_EQ(GetPreviousFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR);
  EXPECT_EQ(GetPreviousFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR);
  EXPECT_EQ(GetPreviousFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR);
  EXPECT_EQ(GetPreviousFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR);
  EXPECT_EQ(GetPreviousFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM);
  EXPECT_EQ(GetPreviousFollowingDistanceLevel(
                QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR),
            QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR);
}

TEST(AssistUtilTest, UpdateAlcState) {
  SetMap("dojo");
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const auto lane_path =
      mapping::LanePath(smm, /*lane_ids=*/
                        {mapping::ElementId(2448), mapping::ElementId(1)},
                        /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, lane_path, /*step_s=*/1.0,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
      /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  absl::Time plan_time = absl::Now();

  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(5.0, 0.0), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    QALCState qalc_state;

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_OFF, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_NONE,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_OFF);

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_STANDBY, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_NONE,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_STANDBY);

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_STANDBY_ENABLE, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_NONE,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_STANDBY_ENABLE);

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_PREPARE, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_PREPARE);

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_RETURN_COMPLETED, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_CANCEL,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_STANDBY_ENABLE);

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_RETURN_COMPLETED, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_PREPARE);

    ASSIGN_OR_DIE(
        qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_COMPLETED, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_STANDBY_ENABLE);
  }

  // ALC_CROSSING_LANE -> ALC_COMPLETED.
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_CROSSING_LANE, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_COMPLETED);
  }

  // ALC_CROSSING_LANE -> ALC_CROSSING_LANE, heading diff greater than 10
  // degree.
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0), 0.3, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_CROSSING_LANE, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_CROSSING_LANE);
  }

  // ALC_CROSSING_LANE -> ALC_CROSSING_LANE, lateral distance to target lane
  // condition is not satisfy.
  {
    // Construct sdc pose.
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(10.0, -0.8), 0.0,
                   Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_CROSSING_LANE, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_CROSSING_LANE);
  }

  // ALC_CROSSING_LANE -> ALC_CROSSING_LANE, not reach to lane change target
  // point
  {
    /// Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);
    Vec2dProto lane_change_target_point;
    lane_change_target_point.set_x(12.0);
    lane_change_target_point.set_y(0.0);
    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_CROSSING_LANE, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       std::make_optional(lane_change_target_point),
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_CROSSING_LANE);
  }

  // ALC_RETURNING -> ALC_RETURN_COMPLETED.
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_RETURNING, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_RETURN_COMPLETED);
  }

  // ALC_RETURNING -> ALC_RETURNING, heading diff greater than 10
  // degree.
  {
    // Construct sdc pose.
    const PoseProto sdc_pose =
        CreatePose(ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.0), -0.3,
                   Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_RETURNING, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_RETURNING);
  }

  // ALC_CROSSING_LANE -> ALC_CROSSING_LANE, lateral distance to target lane
  // condition is not satisfy.
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.8), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_RETURNING, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_RETURNING);
  }

  // ALC_ONGOING -> ALC_CROSSING_LANE
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 0.9), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_ONGOING, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_CROSSING_LANE);
  }

  // ALC_ONGOING -> ALC_CROSSING_LANE
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 1.5), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_ONGOING, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_CROSSING_LANE);
  }

  // ALC_ONGOING -> ALC_ONGOING
  {
    // Construct sdc pose.
    const PoseProto sdc_pose = CreatePose(
        ToUnixDoubleSeconds(plan_time), Vec2d(10.0, 2.6), 0.0, Vec2d(0.0, 0.0));
    const auto start_point = ConvertToTrajPointProto(sdc_pose);

    ASSIGN_OR_DIE(
        const auto qalc_state,
        UpdateAlcState(/*state=*/QALCState::ALC_ONGOING, drive_passage,
                       {start_point}, /*lc_cmd=*/DriverAction::LC_CMD_LEFT,
                       /*lane_change_target_point=*/std::nullopt,
                       /*preview_duration=*/0.0, /*preview_length=*/0.0));
    EXPECT_EQ(qalc_state, QALCState::ALC_ONGOING);
  }
}

TEST(AssistUtilTest, UpdateFollowHeadwayTimeAccordHmi) {
  SpeedFinderParamsProto proto;
  proto.set_follow_time_headway(-0.1);
  proto.set_large_vehicle_follow_time_headway(-0.2);
  UpdateFollowHeadwayTimeAccordHmi(
      &proto, /*level=*/QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM);
  EXPECT_NEAR(proto.follow_time_headway(), 1.7, 1e-2);
  EXPECT_NEAR(proto.large_vehicle_follow_time_headway(), 2.0, 1e-2);
}

TEST(AssistUtilTest, CalculateLaneChangeState) {
  EXPECT_EQ(
      CalculateLaneChangeState(FrenetBox(),
                               /*state=*/QALCState::ALC_OFF,
                               /*lc_direction=*/LaneChangeDirection::LCD_NONE)
          .stage(),
      LaneChangeStage::LCS_NONE);

  EXPECT_EQ(
      CalculateLaneChangeState(FrenetBox(), /*state=*/QALCState::ALC_STANDBY,
                               /*lc_direction=*/LaneChangeDirection::LCD_NONE)
          .stage(),
      LaneChangeStage::LCS_NONE);

  EXPECT_EQ(CalculateLaneChangeState(
                FrenetBox(), /*state=*/QALCState::ALC_STANDBY_ENABLE,
                /*lc_direction=*/LaneChangeDirection::LCD_NONE)
                .stage(),
            LaneChangeStage::LCS_NONE);

  EXPECT_EQ(
      CalculateLaneChangeState(FrenetBox(), /*state=*/QALCState::ALC_COMPLETED,
                               /*lc_direction=*/LaneChangeDirection::LCD_NONE)
          .stage(),
      LaneChangeStage::LCS_NONE);

  EXPECT_EQ(CalculateLaneChangeState(
                FrenetBox(), /*state=*/QALCState::ALC_RETURN_COMPLETED,
                /*lc_direction=*/LaneChangeDirection::LCD_NONE)
                .stage(),
            LaneChangeStage::LCS_NONE);

  EXPECT_EQ(
      CalculateLaneChangeState(FrenetBox(), /*state=*/QALCState::ALC_PREPARE,
                               /*lc_direction=*/LaneChangeDirection::LCD_NONE)
          .stage(),
      LaneChangeStage::LCS_NONE);
  {
    FrenetBox frenet_box{
        .s_max = 1.0, .s_min = 0.0, .l_max = -2.0, .l_min = -3.0};
    const auto proto = CalculateLaneChangeState(
        frenet_box, /*state=*/QALCState::ALC_ONGOING,
        /*lc_direction=*/LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(proto.stage(), LaneChangeStage::LCS_EXECUTING);
    EXPECT_TRUE(proto.lc_left());
    EXPECT_FALSE(proto.entered_target_lane());
  }

  {
    FrenetBox frenet_box{
        .s_max = 1.0, .s_min = 0.0, .l_max = -1.0, .l_min = -2.0};
    const auto proto = CalculateLaneChangeState(
        frenet_box, /*state=*/QALCState::ALC_CROSSING_LANE,
        /*lc_direction=*/LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(proto.stage(), LaneChangeStage::LCS_EXECUTING);
    EXPECT_TRUE(proto.lc_left());
    EXPECT_FALSE(proto.entered_target_lane());
  }

  {
    FrenetBox frenet_box{
        .s_max = 1.0, .s_min = 0.0, .l_max = 0.8, .l_min = -0.2};
    const auto proto = CalculateLaneChangeState(
        frenet_box, /*state=*/QALCState::ALC_CROSSING_LANE,
        /*lc_direction=*/LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(proto.stage(), LaneChangeStage::LCS_EXECUTING);
    EXPECT_TRUE(proto.lc_left());
    EXPECT_TRUE(proto.entered_target_lane());
  }

  {
    FrenetBox frenet_box{
        .s_max = 1.0, .s_min = 0.0, .l_max = 0.8, .l_min = -0.2};
    const auto proto = CalculateLaneChangeState(
        frenet_box,
        /*state=*/QALCState::ALC_RETURNING,
        /*lc_direction=*/LaneChangeDirection::LCD_NONE);
    EXPECT_EQ(proto.stage(), LaneChangeStage::LCS_RETURN);
    EXPECT_FALSE(proto.lc_left());
    EXPECT_TRUE(proto.entered_target_lane());
  }
}

TEST(AssistUtilTest, UpdateExternalCmdQueueFromDriverAction) {
  // Update success test.
  {
    DriverAction driver_action;
    ExternalCommandQueue queue;
    EXPECT_OK(UpdateExternalCmdQueueFromDriverAction(driver_action, &queue));
    EXPECT_EQ(queue.pending_driver_actions.size(), 1);
  }

  // Update failed test.
  {
    DriverAction driver_action;
    driver_action.mutable_header()->set_timestamp(
        absl::ToUnixMicros(absl::Now() - absl::Seconds(5.0)));
    ExternalCommandQueue queue;
    EXPECT_NOT_OK(
        UpdateExternalCmdQueueFromDriverAction(driver_action, &queue));
  }
}

TEST(AssistUtilTest, UpdateExternalCmdStatusFromRemoteAssist) {
  RemoteAssistToCarProto proto;
  ExternalCommandStatus status;

  {
    proto.mutable_left_blinker_override()->set_on(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.override_left_blinker_on);
  }

  {
    proto.mutable_right_blinker_override()->set_on(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.override_right_blinker_on);
  }

  {
    proto.mutable_emergency_blinker_override()->set_on(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.override_emergency_blinker_on);
  }

  {
    proto.mutable_door_override()->set_open(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.override_door_open);
  }

  {
    proto.set_lane_change_style(LC_STYLE_RADICAL);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_EQ(status.lane_change_style, LC_STYLE_RADICAL);
  }

  {
    auto* enable_feature_override = proto.mutable_enable_feature_override();
    enable_feature_override->set_enable_traffic_light_stopping(true);
    enable_feature_override->set_enable_lc_objects(true);
    enable_feature_override->set_enable_pull_over(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.enable_traffic_light_stopping);
    EXPECT_TRUE(status.enable_lc_objects);
    EXPECT_TRUE(status.enable_pull_over);
  }

  {
    proto.mutable_stop_vehicle()->set_brake(1.0);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.brake_to_stop.has_value());
    EXPECT_NEAR(*status.brake_to_stop, 1.0, 1e-1);
  }

  {
    proto.set_enable_stop_polyline_stopping_override(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
    EXPECT_TRUE(status.enable_stop_polyline_stopping);
  }

  {
    proto.mutable_driving_action_request();
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }

  {
    proto.mutable_heartbeat();
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }

  {
    proto.mutable_drivable_agent_update_request();
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }

  {
    proto.set_use_manual_control_cmd(true);
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }

  {
    proto.mutable_play_audio_request();
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }

  {
    proto.mutable_aeb_request();
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }

  {
    proto.clear_request();
    EXPECT_OK(UpdateExternalCmdStatusFromRemoteAssist(proto, &status));
  }
}

TEST(AssistUtilTest, AlcStateToLaneChangeStage) {
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_OFF), LaneChangeStage::LCS_NONE);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_STANDBY), LaneChangeStage::LCS_NONE);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_STANDBY_ENABLE),
            LaneChangeStage::LCS_NONE);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_COMPLETED),
            LaneChangeStage::LCS_NONE);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_RETURN_COMPLETED),
            LaneChangeStage::LCS_NONE);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_PREPARE), LaneChangeStage::LCS_NONE);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_ONGOING),
            LaneChangeStage::LCS_EXECUTING);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_CROSSING_LANE),
            LaneChangeStage::LCS_EXECUTING);
  EXPECT_EQ(AlcStateToLaneChangeStage(ALC_RETURNING),
            LaneChangeStage::LCS_RETURN);
}

TEST(AssistUtilTest, IsLaneChangeState) {
  EXPECT_TRUE(IsLaneChangeState(/*state=*/QALCState::ALC_ONGOING));
  EXPECT_TRUE(IsLaneChangeState(/*state=*/QALCState::ALC_COMPLETED));
  EXPECT_TRUE(IsLaneChangeState(/*state=*/QALCState::ALC_CROSSING_LANE));
  EXPECT_TRUE(IsLaneChangeState(/*state=*/QALCState::ALC_RETURNING));
  EXPECT_TRUE(IsLaneChangeState(/*state=*/QALCState::ALC_RETURN_COMPLETED));

  EXPECT_FALSE(IsLaneChangeState(/*state=*/QALCState::ALC_OFF));
  EXPECT_FALSE(IsLaneChangeState(/*state=*/QALCState::ALC_STANDBY));
  EXPECT_FALSE(IsLaneChangeState(/*state=*/QALCState::ALC_STANDBY_ENABLE));
  EXPECT_FALSE(IsLaneChangeState(/*state=*/QALCState::ALC_PREPARE));
}

TEST(AssistUtilTest, ProcessLaneChangeCommands) {
  // Commands from LaneChangeRequestProto.
  {
    std::queue<LaneChangeRequestProto> lc_requests;
    LaneChangeRequestProto lc_req;

    lc_req.set_direction(LaneChangeRequestProto::LEFT);
    lc_requests.push(lc_req);
    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_lane_change_requests = lc_requests}),
              DriverAction::LC_CMD_LEFT);

    lc_req.set_direction(LaneChangeRequestProto::RIGHT);
    lc_requests.push(lc_req);
    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_lane_change_requests = lc_requests}),
              DriverAction::LC_CMD_RIGHT);

    lc_req.set_direction(LaneChangeRequestProto::CANCEL);
    lc_requests.push(lc_req);
    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_lane_change_requests = lc_requests}),
              DriverAction::LC_CMD_CANCEL);

    lc_req.set_direction(LaneChangeRequestProto::STRAIGHT);
    lc_requests.push(lc_req);
    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_lane_change_requests = lc_requests}),
              DriverAction::LC_CMD_STRAIGHT);
  }

  // {Left, Cancel, None} ---> Cancel.
  {
    std::deque<DriverAction> driver_actions;
    DriverAction action;
    action.set_lane_change_command(DriverAction::LC_CMD_LEFT);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_CANCEL);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_NONE);
    driver_actions.push_back(action);

    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_driver_actions = driver_actions}),
              DriverAction::LC_CMD_CANCEL);
  }

  // {Left, Cancel, Right} ---> Cancel.
  {
    std::deque<DriverAction> driver_actions;
    DriverAction action;
    action.set_lane_change_command(DriverAction::LC_CMD_LEFT);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_CANCEL);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_RIGHT);
    driver_actions.push_back(action);

    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_driver_actions = driver_actions}),
              DriverAction::LC_CMD_CANCEL);
  }

  // {None, Left, None} ---> Left.
  {
    std::deque<DriverAction> driver_actions;
    DriverAction action;
    action.set_lane_change_command(DriverAction::LC_CMD_NONE);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_LEFT);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_NONE);
    driver_actions.push_back(action);

    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_driver_actions = driver_actions}),
              DriverAction::LC_CMD_LEFT);
  }

  // {None, None, Right} ---> Right.
  {
    std::deque<DriverAction> driver_actions;
    DriverAction action;
    action.set_lane_change_command(DriverAction::LC_CMD_NONE);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_NONE);
    driver_actions.push_back(action);
    action.set_lane_change_command(DriverAction::LC_CMD_RIGHT);
    driver_actions.push_back(action);

    EXPECT_EQ(ProcessLaneChangeCommands(ExternalCommandQueue{
                  .pending_driver_actions = driver_actions}),
              DriverAction::LC_CMD_RIGHT);
  }
}

TEST(AssistUtilTest, HasTrajectoryCrossedSolidBoundary) {
  SetMap("dojo");
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();
  const auto& vehicle_geom = vehicle_params.vehicle_geometry_params();

  const auto lane_path =
      mapping::LanePath(smm, /*lane_ids=*/
                        {mapping::ElementId(2491), mapping::ElementId(2484),
                         mapping::ElementId(62)},
                        /*start_fraction=*/0.5, /*end_fraction=*/1.0);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, lane_path, /*step_s=*/1.0, /*avoid_loop=*/true,
      /*backward_extend_len=*/0.0, /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);
  const auto sl_boundary = CreateFakePathSlBoundary(drive_passage);

  {
    const auto traj_pts = MakeStraightTrajectory(
        Vec2d(190.0, 66.5), Vec2d(245.0, 66.5), kTrajectorySteps);
    const auto crossed_or = HasTrajectoryCrossedSolidBoundary(
        drive_passage, sl_boundary, traj_pts, vehicle_geom,
        /*lc_pause=*/false, /*is_vision_map=*/false);
    ASSERT_OK(crossed_or);
    EXPECT_FALSE(*crossed_or);
  }
  {
    const auto traj_pts = MakeStraightTrajectory(
        Vec2d(190.0, 66.5), Vec2d(230.0, 63.0), kTrajectorySteps);
    const auto crossed_or = HasTrajectoryCrossedSolidBoundary(
        drive_passage, sl_boundary, traj_pts, vehicle_geom,
        /*lc_pause=*/false, /*is_vision_map=*/false);
    ASSERT_OK(crossed_or);
    EXPECT_FALSE(*crossed_or);
  }
  {
    const auto traj_pts = MakeStraightTrajectory(
        Vec2d(190.0, 66.5), Vec2d(235.0, 63.0), kTrajectorySteps);
    const auto crossed_or = HasTrajectoryCrossedSolidBoundary(
        drive_passage, sl_boundary, traj_pts, vehicle_geom,
        /*lc_pause=*/false, /*is_vision_map=*/false);
    ASSERT_OK(crossed_or);
    EXPECT_TRUE(*crossed_or);
  }
  {
    const auto traj_pts = MakeStraightTrajectory(
        Vec2d(190.0, 66.5), Vec2d(245.0, 63.0), kTrajectorySteps);
    const auto crossed_or = HasTrajectoryCrossedSolidBoundary(
        drive_passage, sl_boundary, traj_pts, vehicle_geom,
        /*lc_pause=*/false, /*is_vision_map=*/false);
    ASSERT_OK(crossed_or);
    EXPECT_TRUE(*crossed_or);
  }
}

TEST(AssistUtilTest, ReportPlcEventSignal) {
  {
    ReportPlcEventSignal(/*old_state*/ ALC_OFF, /*new_state*/ ALC_OFF,
                         /*lc_cmd*/ DriverAction::LC_CMD_NONE,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY, /*new_state*/ ALC_STANDBY,
                         /*lc_cmd*/ DriverAction::LC_CMD_NONE,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_STANDBY_ENABLE,
                         /*lc_cmd*/ DriverAction::LC_CMD_NONE,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_CROSSING_LANE,
                         /*new_state*/ ALC_CROSSING_LANE,
                         /*lc_cmd*/ DriverAction::LC_CMD_NONE,
                         /*plc_status*/ PlcInternalStatus::OK);
  }

  {
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_PREPARE,
                         /*lc_cmd*/ DriverAction::LC_CMD_LEFT,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_PREPARE,
                         /*lc_cmd*/ DriverAction::LC_CMD_RIGHT,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_PREPARE,
                         /*lc_cmd*/ DriverAction::LC_CMD_LEFT,
                         /*plc_status*/ PlcInternalStatus::SOLID_BOUNDARY);
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_PREPARE,
                         /*lc_cmd*/ DriverAction::LC_CMD_RIGHT,
                         /*plc_status*/ PlcInternalStatus::SOLID_BOUNDARY);
  }

  {
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_ONGOING,
                         /*lc_cmd*/ DriverAction::LC_CMD_LEFT,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_STANDBY_ENABLE,
                         /*new_state*/ ALC_ONGOING,
                         /*lc_cmd*/ DriverAction::LC_CMD_RIGHT,
                         /*plc_status*/ PlcInternalStatus::OK);
  }

  {
    ReportPlcEventSignal(/*old_state*/ ALC_ONGOING,
                         /*new_state*/ ALC_RETURNING,
                         /*lc_cmd*/ DriverAction::LC_CMD_LEFT,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_ONGOING,
                         /*new_state*/ ALC_RETURNING,
                         /*lc_cmd*/ DriverAction::LC_CMD_RIGHT,
                         /*plc_status*/ PlcInternalStatus::OK);
  }

  {
    ReportPlcEventSignal(/*old_state*/ ALC_CROSSING_LANE,
                         /*new_state*/ ALC_COMPLETED,
                         /*lc_cmd*/ DriverAction::LC_CMD_LEFT,
                         /*plc_status*/ PlcInternalStatus::OK);
    ReportPlcEventSignal(/*old_state*/ ALC_RETURNING,
                         /*new_state*/ ALC_RETURN_COMPLETED,
                         /*lc_cmd*/ DriverAction::LC_CMD_LEFT,
                         /*plc_status*/ PlcInternalStatus::OK);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
