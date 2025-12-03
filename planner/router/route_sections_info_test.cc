#include "onboard/planner/router/route_sections_info.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {

namespace {

TEST(RouteSectionsInfoTest, BuildTest) {
  FLAGS_planner_enable_dynamic_lane_speed_limit = false;

  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  SendRouteSectionsAreaToCanvas(psmm, route_result.route_sections,
                                "test/fork_route");

  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
}

TEST(RouteSectionsInfoTest, PlanningHorizonTest) {
  const TestRouteResult route_result = CreateAContinuousLaneChangeRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  SendRouteSectionsAreaToCanvas(psmm, route_result.route_sections,
                                "test/continuous_lc_route");

  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);

  EXPECT_GT(sections_info.planning_horizon(), sections_info.length());

  // v2 test
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> v2smm = map_fut.Get();
  RouteSectionsInfo sections_info2(*v2smm, &route_result.route_sections);
  EXPECT_GT(sections_info2.planning_horizon(), sections_info2.length());
}

}  // namespace

}  // namespace qcraft::planner
