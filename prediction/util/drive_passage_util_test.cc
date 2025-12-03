#include "onboard/prediction/util/drive_passage_util.h"

#include <vector>

#include "absl/status/statusor.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/spatial_search_util.h"

namespace qcraft::prediction {
namespace {

TEST(DrivePassageUtilTest, PruneLanePathByLengthTest1) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  Vec2d pos(62.0, 69.0);
  std::vector<mapping::ElementId> lane_ids = {mapping::ElementId(15),
                                              mapping::ElementId(80)};
  mapping::LanePath lp =
      mapping::LanePath(psmm.semantic_map_manager(), lane_ids, 0.0, 1.0);
  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm.GetLevel(), psmm, pos, lp, 0.0);
  const auto pruned_lane_path =
      PruneLanePathByLength(psmm, lp, *closest_lane_point_or, 30, 10);
  EXPECT_EQ(pruned_lane_path.lane_ids_size(), 2);
  EXPECT_NEAR(pruned_lane_path.start_fraction(), 0.7418, 0.01);
  EXPECT_NEAR(pruned_lane_path.end_fraction(), 0.5538, 0.01);
}

TEST(DrivePassageUtilTest, PruneLanePathByLengthTest2) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  Vec2d pos(78.0, 69.0);
  std::vector<mapping::ElementId> lane_ids = {mapping::ElementId(15),
                                              mapping::ElementId(80)};
  mapping::LanePath lp =
      mapping::LanePath(psmm.semantic_map_manager(), lane_ids, 0.0, 1.0);
  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm.GetLevel(), psmm, pos, lp, 0.0);
  const auto pruned_lane_path =
      PruneLanePathByLength(psmm, lp, *closest_lane_point_or, 10, 20);
  EXPECT_EQ(pruned_lane_path.lane_ids_size(), 2);
  EXPECT_NEAR(pruned_lane_path.start_fraction(), 0.823, 0.01);
  EXPECT_NEAR(pruned_lane_path.end_fraction(), 0.4447, 0.01);
}

TEST(DrivePassageUtilTest, PruneLanePathByLengthTest3) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  Vec2d pos(88.0, 76.0);
  std::vector<mapping::ElementId> lane_ids = {mapping::ElementId(15),
                                              mapping::ElementId(80)};
  mapping::LanePath lp =
      mapping::LanePath(psmm.semantic_map_manager(), lane_ids, 0.0, 1.0);
  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm.GetLevel(), psmm, pos, lp, 0.0);
  const auto pruned_lane_path =
      PruneLanePathByLength(psmm, lp, *closest_lane_point_or, 10, 10);
  EXPECT_EQ(pruned_lane_path.lane_ids_size(), 1);
  EXPECT_NEAR(pruned_lane_path.start_fraction(), 0.238, 0.01);
  EXPECT_NEAR(pruned_lane_path.end_fraction(), 0.743, 0.01);
}

}  // namespace
}  // namespace qcraft::prediction
