#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"

#include <algorithm>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

TEST(LaneGraph, ForkLaneTest) {
  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*route_result.smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);
  SendLaneGraphToCanvas(lane_graph, psmm, sections_info, "lane_graph_fork");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  absl::flat_hash_map<mapping::ElementId, LanePathInfo> id_info_map;
  for (auto& lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    if (!id_info_map.contains(start_id) ||
        lp_info.length_along_route() >
            FindOrDie(id_info_map, start_id).length_along_route()) {
      id_info_map[start_id] = std::move(lp_info);
    }
  }
  for (const auto& [lane_id, lp_info] : id_info_map) {
    SendLanePathInfoToCanvas(
        lp_info, psmm, "lane_graph_fork_lp/" + std::to_string(lane_id.value()));
  }

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(1701)).lane_path().length(),
              158.893, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(1701)).length_along_route(),
              130.0, 1e-3);
}

TEST(LaneGraph, ContinuousLaneChangeTest) {
  const TestRouteResult route_result = CreateAContinuousLaneChangeRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*route_result.smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);
  SendLaneGraphToCanvas(lane_graph, psmm, sections_info, "lane_graph_cont_lc");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  absl::flat_hash_map<mapping::ElementId, LanePathInfo> id_info_map;
  for (auto& lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    if (!id_info_map.contains(start_id) ||
        lp_info.length_along_route() >
            FindOrDie(id_info_map, start_id).length_along_route()) {
      id_info_map[start_id] = std::move(lp_info);
    }
  }
  for (const auto& [lane_id, lp_info] : id_info_map) {
    SendLanePathInfoToCanvas(
        lp_info, psmm,
        "lane_graph_cont_lc_lp/" + std::to_string(lane_id.value()));
  }

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2334)).lane_path().length(),
              156.514, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2334)).length_along_route(),
              90.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2333)).lane_path().length(),
              156.467, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2333)).length_along_route(),
              110.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2332)).lane_path().length(),
              156.371, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2332)).length_along_route(),
              130.0, 1e-3);

  EXPECT_TRUE(id_info_map.contains(mapping::ElementId(1697)));
}

TEST(LaneGraph, ContinuousLaneChangeWithSolidTest) {
  const TestRouteResult route_result =
      CreateAContinuousLaneChangeRouteWithSolidInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*route_result.smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  // Build one object
  ObjectVector<PlannerObject> objects;
  const auto perc_obj = PerceptionObjectBuilder()
                            .set_id("Phantom")
                            .set_type(ObjectType::OT_VEHICLE)
                            .set_pos(Vec2d(155.0, 0.0))
                            .set_length_width(4.5, 2.2)
                            .set_yaw(0.0)
                            .Build();

  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perc_obj)
      .set_stationary(true)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_stationary_traj(Vec2d(155.0, 0.0), 0.0);

  objects.push_back(builder.Build());
  PlannerObjectManager object_mgr(objects);
  DrawPlannerObjectManagerToCanvas(object_mgr, "object",
                                   vis::Color::kLightGreen);

  const absl::flat_hash_set<std::string> stalled_objects{"Phantom"};

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, object_mgr, stalled_objects,
      /*avoid_lanes=*/{mapping::ElementId(2472)}, route_navi_info);
  SendLaneGraphToCanvas(lane_graph, psmm, sections_info, "lane_graph_solid");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  absl::flat_hash_map<mapping::ElementId, LanePathInfo> id_info_map;
  for (auto& lp_info : lp_infos) {
    const auto start_id = lp_info.start_lane_id();
    if (!id_info_map.contains(start_id) ||
        lp_info.length_along_route() >
            FindOrDie(id_info_map, start_id).length_along_route()) {
      id_info_map[start_id] = std::move(lp_info);
    }
  }
  for (const auto& [lane_id, lp_info] : id_info_map) {
    SendLanePathInfoToCanvas(
        lp_info, psmm,
        "lane_graph_solid_lp/" + std::to_string(lane_id.value()));
  }

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2470)).lane_path().length(),
              134.864, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2470)).length_along_route(),
              90.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2471)).lane_path().length(),
              134.768, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2471)).length_along_route(),
              20.0, 1e-3);

  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2472)).lane_path().length(),
              134.769, 1e-3);
  EXPECT_NEAR(id_info_map.at(mapping::ElementId(2472)).length_along_route(),
              0.0, 1e-3);
}

}  // namespace

}  // namespace qcraft::planner
