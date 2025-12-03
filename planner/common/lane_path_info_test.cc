#include "onboard/planner/common/lane_path_info.h"

#include "gtest/gtest.h"

#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/lane_path_util.h"

namespace qcraft::planner {

namespace {

TEST(LanePathInfo, LanePathInfoQueryTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  const auto lane_path_or = BuildLanePathFromData(
      mapping::LanePathData(0.0, 1.0, {mapping::ElementId(2471)}), psmm);
  EXPECT_OK(lane_path_or);

  const LanePathInfo lane_path_info(*lane_path_or,
                                    /*len_along_route=*/0.0, /*path_cost=*/0.0,
                                    psmm);
  const auto sl = lane_path_info.ProjectionSL(Vec2d(140.0, 3.5));
  EXPECT_NEAR(sl.s, 24.5, 0.1);
  EXPECT_NEAR(sl.l, 3.5, 0.1);

  const Vec2d xy = lane_path_info.ProjectionXY(sl);
  EXPECT_NEAR(xy.x(), 140.0, 1e-3);
  EXPECT_NEAR(xy.y(), 3.5, 1e-3);

  const auto ranged_sl =
      lane_path_info.ProjectionSLInRange(Vec2d(140.0, 3.5), 0.0, 10.0);
  EXPECT_NEAR(ranged_sl.s, 24.5, 0.1);
  EXPECT_NEAR(ranged_sl.l, 3.5, 0.1);
}

}  // namespace

}  // namespace qcraft::planner
