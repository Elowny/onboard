#include "onboard/prediction/util/trajectory_developer.h"

#include <algorithm>  // for max
#include <cmath>
#include <optional>  // for optional
#include <ostream>   // for char_traits, operator<<, basic_ostream
#include <utility>   // for pair

#include "absl/status/statusor.h"  // for StatusOr

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"  // for BufferedLoggerWrapper
#include "onboard/lite/check.h"              // for QCHECK
#include "onboard/maps/lane_path.h"          // for LanePath
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/speed/speed_point.h"  // for SpeedPoint
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/post_process/conflict_resolver_util.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/proto/prediction.pb.h"  // for PredictionType

namespace qcraft::prediction {
namespace {
absl::StatusOr<planner::DrivePassage> BuildDrivePassage(
    const ObjectMotionState& cur_state) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  const auto lane_id_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          psmm, cur_state.pos, cur_state.heading,
          /*boundary_distance_limit=*/0.0,
          /*max_heading_diff=*/M_PI / 3.0);

  const auto lps =
      SearchLanePath(cur_state.pos, psmm, *lane_id_or, /*forward_len=*/50.0,
                     /*is_reverse_driving=*/false);
  QCHECK(!lps.empty()) << "Please pass reasonable pos to search lane";
  const auto extended_lp = ExtendMostStraightLanePath(lps[0], psmm, 10.0,
                                                      /*is_reversed=*/false);
  return planner::BuildDrivePassageForPrediction(
      psmm, extended_lp, 5.0,
      /*avoid_loop=*/true,
      /*backward_extend_len=*/20.0, kMaxLateralBoundaryForPredictionDp);
}

constexpr double kEpsilon = 1e-3;
TEST(TrajectoryDeveloperTest, StaticTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopStaticTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_EQ(last_pt.pos(), Vec2d(0.0, 0.0));
  EXPECT_EQ(last_pt.s(), 0.0);
  EXPECT_EQ(last_pt.v(), 0.0);
}

TEST(TrajectoryDeveloperTest, VoidTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopVoidTrajectory(state);
  EXPECT_EQ(pts.size(), 1);
  const auto& last_pt = pts.back();
  EXPECT_EQ(last_pt.t(), 0.0);
  EXPECT_EQ(last_pt.pos(), Vec2d(0.0, 0.0));
  EXPECT_EQ(last_pt.s(), 0.0);
  EXPECT_EQ(last_pt.v(), 1.0);
}

TEST(TrajectoryDeveloperTest, CYCVTrajectoryAxisAlignedTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCYCVTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0,
                                         /*is_reversed=*/false);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().y(), 0.0, kEpsilon);

  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, CYCVTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = M_PI / 4.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCYCVTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0,
                                         /*is_reversed=*/false);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.9 * std::sqrt(2.0) / 2.0, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI / 4.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, CYCVReverseTrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = M_PI / 4.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts = DevelopCYCVTrajectory(state, /*dt=*/0.1, /*horizon=*/3.0,
                                         /*is_reversed=*/true);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), -2.9 * std::sqrt(2.0) / 2.0, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), -3.0 * M_PI / 4.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, DummyCTRATrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = M_PI / 4.0,
                            .yaw_rate = 0.0,
                            .acc = 0.0};
  const auto pts =
      DevelopForwardCTRATrajectory(state, /*dt=*/0.1, /*acc_stop_time=*/3.0,
                                   /*yaw_rate_stop_time=*/3.0, /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.9 * std::sqrt(2.0) / 2.0, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI / 4.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, AccCTRATrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 0.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 1.0};
  const auto pts =
      DevelopForwardCTRATrajectory(state, /*dt=*/0.1, /*acc_stop_time=*/1.0,
                                   /*yaw_rate_stop_time=*/1.0,
                                   /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 2.4, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 2.4, 1e-1);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, BrakeCTRATrajectoryTest) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = -1.0};
  auto pts =
      DevelopForwardCTRATrajectory(state, /*dt=*/0.1, /*acc_stop_time=*/1.0,
                                   /*yaw_rate_stop_time=*/1.0,
                                   /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 0.5, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 0.5, 1e-1);
  EXPECT_NEAR(last_pt.v(), 0.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);

  const UniCycleState state2{.x = 0.0,
                             .y = 0.0,
                             .v = 1.0,
                             .heading = 0.0,
                             .yaw_rate = 0.0,
                             .acc = -2.0};
  pts = DevelopForwardCTRATrajectory(state2, /*dt=*/0.1, /*acc_stop_time=*/1.0,
                                     /*yaw_rate_stop_time=*/1.0,
                                     /*horizon=*/3.0);
  const auto& last_pt2 = pts.back();
  EXPECT_NEAR(last_pt2.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt2.pos().x(), 0.25, kEpsilon);
  EXPECT_NEAR(last_pt2.s(), 0.25, 1e-1);
  EXPECT_NEAR(last_pt2.v(), 0.0, kEpsilon);
  EXPECT_NEAR(last_pt2.theta(), 0.0, kEpsilon);

  const UniCycleState state3{.x = 0.0,
                             .y = 0.0,
                             .v = 2.0,
                             .heading = 0.0,
                             .yaw_rate = 0.0,
                             .acc = -1.0};
  pts = DevelopForwardCTRATrajectory(state3, /*dt=*/0.1, /*acc_stop_time=*/1.0,
                                     /*yaw_rate_stop_time=*/1.0,
                                     /*horizon=*/3.0);
  const auto& last_pt3 = pts.back();
  EXPECT_NEAR(last_pt3.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt3.pos().x(), 3.4, kEpsilon);
  EXPECT_NEAR(last_pt3.s(), 3.4, 1e-1);
  EXPECT_NEAR(last_pt3.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt3.theta(), 0.0, kEpsilon);

  pts = DevelopForwardCTRATrajectory(state3, /*dt=*/0.1, /*acc_stop_time=*/0.0,
                                     /*yaw_rate_stop_time=*/1.0,
                                     /*horizon=*/3.0);
  const auto& last_pt4 = pts.back();
  EXPECT_NEAR(last_pt4.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt4.pos().x(), 5.8, kEpsilon);
  EXPECT_NEAR(last_pt4.s(), 5.8, 1e-1);
  EXPECT_NEAR(last_pt4.v(), 2.0, kEpsilon);
  EXPECT_NEAR(last_pt4.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, AccCTRATrajectoryTest2) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 0.0,
                            .heading = 0.0,
                            .yaw_rate = 0.0,
                            .acc = 1.0};
  const auto pts =
      DevelopForwardCTRATrajectory(state, /*dt=*/0.1, /*acc_stop_time=*/3.0,
                                   /*yaw_rate_stop_time=*/3.0, /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 0.5 * 2.9 * 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.s(), 0.5 * 2.9 * 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.v(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), 0.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, YawRateCTRATrajectoryTest2) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = M_PI / 2.9,
                            .acc = 0.0};
  const auto pts =
      DevelopForwardCTRATrajectory(state, /*dt=*/0.1, /*acc_stop_time=*/3.0,
                                   /*yaw_rate_stop_time=*/3.0, /*horizon=*/3.0);
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 0.0, kEpsilon);
  EXPECT_NEAR(last_pt.pos().y(), 2.0 * 2.9 / M_PI, 1e-2);
  EXPECT_NEAR(last_pt.s(), 2.9, 1e-2);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI, kEpsilon);
}

TEST(TrajectoryDeveloperTest, ConstAccSpeedProfileTest) {
  const auto speed_vec =
      DevelopConstAccSpeedProfile(5.0, 1.0, 1.0, 3.0, 0.1, 3.0);
  EXPECT_EQ(speed_vec.size(), 30);
  EXPECT_NEAR(speed_vec.back().v(), 6.0, kEpsilon);
}

TEST(TrajectoryDeveloperTest, LineFitAccelerationByMotionHistory) {
  std::vector<ObjectMotionState> motion_states;
  motion_states.reserve(10);
  for (int i = 0; i < 10; ++i) {
    motion_states.push_back(ObjectMotionState{
        .timestamp = i * 0.1, .vel = Vec2d(5.0 + i * 0.1 * 1.0, 0.0)});
  }
  const double fitted_acc =
      LineFitAccelerationByMotionHistory(motion_states, kAccelerationFitTime);
  EXPECT_NEAR(fitted_acc, 1.0, 0.1);
}

TEST(TrajectoryDeveloperTest, LineFitAccelerationByMotionHistoryCut) {
  std::vector<ObjectMotionState> motion_states;
  motion_states.reserve(10);
  for (int i = 0; i < 10; ++i) {
    if (i == 5) {
      // bad case
      motion_states.push_back(
          ObjectMotionState{.timestamp = i * 0.1, .vel = Vec2d(100.0, 0.0)});

    } else {
      motion_states.push_back(ObjectMotionState{
          .timestamp = i * 0.1, .vel = Vec2d(5.0 + i * 0.1 * 2.0, 0.0)});
    }
  }
  const double fitted_acc =
      LineFitAccelerationByMotionHistory(motion_states, kAccelerationFitTime);
  EXPECT_NEAR(fitted_acc, 2.0, 0.1);
}

TEST(TrajectoryDeveloperTest, LineFitLateralSpeedByMotionHistory) {
  std::vector<ObjectMotionState> motion_states;
  motion_states.reserve(10);
  for (int i = 0; i < 10; ++i) {
    motion_states.push_back(ObjectMotionState{
        .timestamp = i * 0.1,
        .pos = Vec2d(0.0 + 2.0 * i * 0.1, 0.0),
        .heading = 0.0,
        .vel = Vec2d(2.0, 0.0),
    });
  }
  const auto drive_passage = BuildDrivePassage(motion_states.back());
  const double lateral_speed = LineFitLateralSpeedByMotionHistory(
      motion_states, *drive_passage, 1.0, /*clamp_by_lane_width=*/true);
  EXPECT_NEAR(lateral_speed, 0.0, 0.1);
}

TEST(TrajectoryDeveloperTest, PredictedTrajectoryPointsToDiscretizedPath) {
  const auto traj =
      planner::PredictedTrajectoryBuilder()
          .set_annotation("Const acc trajectory")
          .set_type(PredictionType::PT_CTRA)
          .set_probability(1.0)
          .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 10.0), 5.0, 10.0)
          .Build();
  const auto discreted_path =
      PredictedTrajectoryPointsToDiscretizedPath(traj.points());
  EXPECT_EQ(discreted_path.size(), traj.points().size());
}
TEST(TrajectoryDeveloperTest, CombinePathAndSpeedForPredictedTrajectoryPoints) {
  const auto traj =
      planner::PredictedTrajectoryBuilder()
          .set_annotation("Const acc trajectory")
          .set_type(PredictionType::PT_CTRA)
          .set_probability(1.0)
          .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 10.0), 5.0, 10.0)
          .Build();
  const auto path_speed = PredictedTrajectoryToPurePathAndSpeedVector(traj);

  const auto traj_points = CombinePathAndSpeedForPredictedTrajectoryPoints(
      path_speed.first, path_speed.second);

  EXPECT_EQ(traj_points.size(), traj.points().size());
}

TEST(ExtendTrajTest, ExtendTraj) {
  const UniCycleState state{.x = 0.0,
                            .y = 0.0,
                            .v = 1.0,
                            .heading = 0.0,
                            .yaw_rate = M_PI / 2.9,
                            .acc = 0.0};
  auto pts =
      DevelopForwardCTRATrajectory(state, /*dt=*/0.1, /*acc_stop_time=*/3.0,
                                   /*yaw_rate_stop_time=*/3.0, /*horizon=*/3.0);
  constexpr double kEpsilon = 1e-2;
  const auto& last_pt = pts.back();
  EXPECT_NEAR(last_pt.t(), 2.9, kEpsilon);
  EXPECT_NEAR(last_pt.pos().x(), 0.0, kEpsilon);
  EXPECT_NEAR(last_pt.pos().y(), 2.0 * 2.9 / M_PI, 1e-2);
  EXPECT_NEAR(last_pt.s(), 2.9, 1e-2);
  EXPECT_NEAR(last_pt.v(), 1.0, kEpsilon);
  EXPECT_NEAR(last_pt.theta(), M_PI, kEpsilon);
  ExtendTrajectoryAlongTangent(kComfortableHorizon, Vec2d::UnitFromAngle(M_PI),
                               &pts);
  EXPECT_EQ(pts.size(), 81);
  EXPECT_NEAR(pts.back().t(), kComfortableHorizon, kEpsilon);
  EXPECT_NEAR(pts.back().s(), kComfortableHorizon, kEpsilon);
  EXPECT_NEAR(pts.back().pos().x(), -(kComfortableHorizon - 2.9), kEpsilon);
  EXPECT_NEAR(pts.back().pos().y(), 2.0 * 2.9 / M_PI, kEpsilon);
}

}  // namespace
}  // namespace qcraft::prediction
