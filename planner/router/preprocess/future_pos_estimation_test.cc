#include "onboard/planner/router/preprocess/future_pos_estimation.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
namespace qcraft::planner::route {
namespace {
TEST(FuturePosEstimation, InferPosFromTrajectoryTest) {
  TrajectoryProto trajectory_proto;
  double start_ts_secs = 1660036194.846;
  trajectory_proto.set_trajectory_start_timestamp(start_ts_secs);
  for (int i = 0; i < 20; i++) {
    auto* trajectory_point = trajectory_proto.add_trajectory_point();
    auto* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(i);
    path_point->set_y(i);
    trajectory_point->set_relative_time(i * 0.1);
  }
  {
    const auto infer_pos_or =
        InferPosFromTrajectory(trajectory_proto, start_ts_secs + 0.05, 1.5);
    EXPECT_TRUE(infer_pos_or.ok());
    EXPECT_NEAR(infer_pos_or->x(), 0.5, 1E-6);
    EXPECT_NEAR(infer_pos_or->y(), 0.5, 1E-6);
  }

  {
    const auto infer_pos_or =
        InferPosFromTrajectory(trajectory_proto, start_ts_secs + 0.5, 1.5);
    EXPECT_TRUE(infer_pos_or.ok());
    EXPECT_NEAR(infer_pos_or->x(), 5, 1E-6);
    EXPECT_NEAR(infer_pos_or->y(), 5, 1E-6);
  }

  {
    const auto infer_pos_or =
        InferPosFromTrajectory(trajectory_proto, start_ts_secs + 2.5, 2.48);
    EXPECT_FALSE(infer_pos_or.ok());
  }
}

}  // namespace

}  // namespace qcraft::planner::route
