#include "onboard/planner/trajectory_validation/trajectory_validation.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {

TEST(ValidateTrajectoryOverlapTest, PointsEmpty) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<ApolloTrajectoryPointProto> past_points;
  TrajectoryValidationResultProto result;
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));
  ApolloTrajectoryPointProto point;
  traj_points.push_back(point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));
  past_points.push_back(point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));
}

TEST(ValidateTrajectoryOverlapTest, PlanStartPointNearPastPoints) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<ApolloTrajectoryPointProto> past_points;
  TrajectoryValidationResultProto result;
  ApolloTrajectoryPointProto point;
  auto* pt = point.mutable_path_point();
  pt->set_x(0.0);
  pt->set_y(0.0);
  pt->set_theta(0.25 * M_PI);
  traj_points.push_back(point);

  ApolloTrajectoryPointProto past_point;
  auto* past_pt = past_point.mutable_path_point();
  past_pt->set_x(0.05);
  past_pt->set_y(0.05);
  past_points.push_back(past_point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));
}

TEST(ValidateTrajectoryOverlapTest, Overlap) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<ApolloTrajectoryPointProto> past_points;
  TrajectoryValidationResultProto result;
  // Special theta 0.
  ApolloTrajectoryPointProto point;
  auto* pt = point.mutable_path_point();
  pt->set_x(0.0);
  pt->set_y(0.0);
  pt->set_theta(0.0);
  traj_points.push_back(point);

  ApolloTrajectoryPointProto past_point;
  auto* past_pt = past_point.mutable_path_point();
  past_pt->set_x(1.0);
  past_pt->set_y(0.0);
  past_points.push_back(past_point);
  EXPECT_FALSE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  traj_points.clear();
  pt->set_theta(0.25 * M_PI);
  traj_points.push_back(point);
  // Don't have overlap.
  past_pt->set_x(-1.0);
  past_pt->set_y(-1.0);
  past_points.push_back(past_point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  past_pt->set_x(-1.0);
  past_pt->set_y(0.9);
  past_points.push_back(past_point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  past_pt->set_x(1.0);
  past_pt->set_y(-1.1);
  past_points.push_back(past_point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  // Perpendicular.
  past_pt->set_x(-1.0);
  past_pt->set_y(1.0);
  past_points.push_back(past_point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  past_pt->set_x(1.0);
  past_pt->set_y(-1.0);
  past_points.push_back(past_point);
  EXPECT_TRUE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  // Has overlap.
  past_pt->set_x(-1.0);
  past_pt->set_y(1.1);
  past_points.push_back(past_point);
  EXPECT_FALSE(ValidateTrajectoryOverlap(traj_points, past_points, &result));

  past_pt->set_x(1.0);
  past_pt->set_y(-0.9);
  past_points.push_back(past_point);
  EXPECT_FALSE(ValidateTrajectoryOverlap(traj_points, past_points, &result));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
