#include "onboard/planner/scheduler/local_map_builder.h"

#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "onboard/container/strong_int.h"
#include "onboard/planner/test_util/load_psmm_util.h"
// #include "absl/strings/str_cat.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_test_util.h"
// #include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

TEST(LocalMapBuilder, DivergingLanePathTest) {
  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const auto route_navi_info = *CalcNaviInfoByLaneGraph(
      *route_result.smm, RouteSectionsInfo(psmm, &route_result.route_sections),
      /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_paths_or =
      BuildLocalMap(psmm, route_result.route_sections, route_navi_info);
  ASSERT_OK(lane_paths_or);
  const auto& lane_paths = *lane_paths_or;

  EXPECT_EQ(lane_paths.size(), 2);
  EXPECT_EQ(lane_paths[0].back().lane_id(), mapping::ElementId(2381));
  EXPECT_EQ(lane_paths[1].back().lane_id(), mapping::ElementId(1759));

  // for (int i = 0; i < lane_paths.size(); ++i) {
  //   DrawLanePathToCanvas(psmm, lane_paths[i],
  //                        absl::StrCat("test/local_map_fork/target_lane_",
  //                                     lane_paths[i].front().lane_id()),
  //                        vis::Color::kMint);
  // }
}

TEST(LocalMapBuilder, DivergingWithAvoidLaneTest) {
  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const auto route_navi_info = *CalcNaviInfoByLaneGraph(
      *route_result.smm, RouteSectionsInfo(psmm, &route_result.route_sections),
      /*avoid_lanes=*/{mapping::ElementId(1757)}, /*preview_dist=*/1000.0);

  const auto lane_paths_or =
      BuildLocalMap(psmm, route_result.route_sections, route_navi_info);
  ASSERT_OK(lane_paths_or);
  const auto& lane_paths = *lane_paths_or;

  EXPECT_EQ(lane_paths.size(), 2);
  EXPECT_EQ(lane_paths[0].back().lane_id(), mapping::ElementId(2381));
  EXPECT_EQ(lane_paths[1].back().lane_id(), mapping::ElementId(2381));

  // for (int i = 0; i < lane_paths.size(); ++i) {
  //   DrawLanePathToCanvas(
  //       *route_result.smm, lane_paths[i],
  //       absl::StrCat("test/local_map_fork_avoid_lane/target_lane_",
  //                    lane_paths[i].front().lane_id()),
  //       vis::Color::kMint);
  // }
}

}  // namespace

}  // namespace qcraft::planner
