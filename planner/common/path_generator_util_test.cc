#include "onboard/planner/common/path_generator_util.h"

#include "gtest/gtest.h"

#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PathGeneratorUtilTest, TestBuildDrivePassage) {
  const auto& planner_semantic_map_manager = CreateDojoTestPSMM();

  PathPoint start_point;
  start_point.set_x(0.0);
  start_point.set_y(0.0);
  start_point.set_theta(0.0);

  const auto drive_passage = BuildDrivePassageFromLaneId(
      planner_semantic_map_manager, /*lane_id=*/mapping::ElementId(2448),
      start_point,
      /*max_length=*/100.0);
  EXPECT_TRUE(drive_passage.has_value());
}

TEST(PathGeneratorUtilTest, TestIsPointIn) {
  const auto& planner_semantic_map_manager = CreateDojoTestPSMM();

  PathPoint start_point;
  start_point.set_x(0.0);
  start_point.set_y(0.0);
  start_point.set_theta(0.0);

  const auto drive_passage = BuildDrivePassageFromLaneId(
      planner_semantic_map_manager, /*lane_id=*/mapping::ElementId(2448),
      start_point,
      /*max_length=*/100.0);
  EXPECT_TRUE(drive_passage.has_value());

  EXPECT_TRUE(IsPointInLeftLaneBoundary(*drive_passage, Vec2d(10.1, 1.1)));
  EXPECT_TRUE(IsPointInLeftLaneBoundary(*drive_passage, Vec2d(18.4, -2.7)));
  EXPECT_FALSE(IsPointInLeftLaneBoundary(*drive_passage, Vec2d(22.6, 2.8)));
  EXPECT_FALSE(IsPointInLeftLaneBoundary(*drive_passage, Vec2d(24.2, 19.2)));

  EXPECT_TRUE(IsPointInRightLaneBoundary(*drive_passage, Vec2d(34.2, -0.7)));
  EXPECT_TRUE(IsPointInRightLaneBoundary(*drive_passage, Vec2d(22.4, 2.6)));
  EXPECT_FALSE(IsPointInRightLaneBoundary(*drive_passage, Vec2d(25.4, -4.0)));
  EXPECT_FALSE(IsPointInRightLaneBoundary(*drive_passage, Vec2d(20.6, -7.5)));

  EXPECT_TRUE(IsPointInLaneBoundaries(*drive_passage, Vec2d(23.0, 0.9)));
  EXPECT_TRUE(IsPointInLaneBoundaries(*drive_passage, Vec2d(64.3, -0.9)));
  EXPECT_FALSE(IsPointInLaneBoundaries(*drive_passage, Vec2d(42.4, 6.4)));
  EXPECT_FALSE(IsPointInLaneBoundaries(*drive_passage, Vec2d(27.7, -4.4)));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
