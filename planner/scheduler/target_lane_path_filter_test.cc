#include "onboard/planner/scheduler/target_lane_path_filter.h"

#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

namespace {

TEST(TargetLanePathFilter, DivergingLanePathTest) {
  const TestRouteResult route_result = CreateAForkLaneRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*route_result.smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);
  SendLaneGraphToCanvas(lane_graph, psmm, sections_info,
                        "target_lane_filter/fork_lane/lane_graph");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(route_result.pose.pos_smooth().x());
  pose.mutable_path_point()->set_y(route_result.pose.pos_smooth().y());
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info,
      route_result.route_lane_path.lane_paths().front(), pose,
      /*preferred_lane_path=*/mapping::LanePath(),
      /*lc_preview_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 2);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), mapping::ElementId(1699));
  EXPECT_EQ(target_lp_infos[1].start_lane_id(), mapping::ElementId(1701));

  for (int i = 0; i < target_lp_infos.size(); ++i) {
    SendLanePathInfoToCanvas(
        target_lp_infos[i], psmm,
        absl::StrCat("target_lane_filter/fork_lane/target_lane_",
                     target_lp_infos[i].start_lane_id()));
  }
}

TEST(TargetLanePathFilter, SingleLanePathTest) {
  const TestRouteResult route_result = CreateASingleLaneRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*route_result.smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);

  SendLaneGraphToCanvas(lane_graph, psmm, sections_info,
                        "target_lane_filter/single_lane/lane_graph");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(route_result.pose.pos_smooth().x());
  pose.mutable_path_point()->set_y(route_result.pose.pos_smooth().y());
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info,
      route_result.route_lane_path.lane_paths().front(), pose,
      /*preferred_lane_path=*/mapping::LanePath(),
      /*lc_preview_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 1);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), mapping::ElementId(692));

  SendLanePathInfoToCanvas(
      target_lp_infos[0], psmm,
      absl::StrCat("target_lane_filter/single_lane/target_lane_",
                   target_lp_infos[0].start_lane_id()));
}

TEST(TargetLanePathFilter, MultipleLanePathTest) {
  // Load map and create optimized route path.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePoint destination(mapping::ElementId(63), 1.0);
  const RouteSections route_sections(
      0.999, 1.0,
      {mapping::SectionId(12415), mapping::SectionId(12417),
       mapping::SectionId(12416), mapping::SectionId(12418)},
      destination);
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);

  SendLaneGraphToCanvas(lane_graph, psmm, sections_info,
                        "target_lane_filter/multi_lane/lane_graph");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(150.1);
  pose.mutable_path_point()->set_y(66.0);
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info,
      mapping::LanePath(smm,
                        {mapping::ElementId(2476), mapping::ElementId(2491),
                         mapping::ElementId(2484), mapping::ElementId(62)},
                        0.95, 1.0),
      pose, /*preferred_lane_path=*/mapping::LanePath(),
      /*lc_preview_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 2);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), mapping::ElementId(2476));
  EXPECT_EQ(target_lp_infos[1].start_lane_id(), mapping::ElementId(61));

  for (int i = 0; i < target_lp_infos.size(); ++i) {
    SendLanePathInfoToCanvas(
        target_lp_infos[i], psmm,
        absl::StrCat("target_lane_filter/multi_lane/target_lane_",
                     target_lp_infos[i].start_lane_id()));
  }
}

TEST(TargetLanePathFilter, LeftTurnLanePathTest) {
  const TestRouteResult route_result = CreateALeftTurnRouteInDojo();
  const auto& psmm = CreateDojoTestPSMM();
  const RouteSectionsInfo sections_info(psmm, &route_result.route_sections);
  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*route_result.smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);
  SendLaneGraphToCanvas(lane_graph, psmm, sections_info,
                        "target_lane_filter/left_turn/lane_graph");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(238.0);
  pose.mutable_path_point()->set_y(63.0);
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info,
      route_result.route_lane_path.lane_paths().front(), pose,
      /*preferred_lane_path=*/mapping::LanePath(),
      /*lc_preview_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 1);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), mapping::ElementId(63));

  SendLanePathInfoToCanvas(
      target_lp_infos[0], psmm,
      absl::StrCat("target_lane_filter/left_turn/target_lane_",
                   target_lp_infos[0].start_lane_id()));
}

TEST(TargetLanePathFilter, ForkMergeLanePathTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePoint destination(mapping::ElementId(35398), 0.1);
  const RouteSections route_sections(
      0.9, 0.1,
      {mapping::SectionId(39112), mapping::SectionId(39063),
       mapping::SectionId(39067), mapping::SectionId(39071),
       mapping::SectionId(35444)},
      destination);
  const RouteSectionsInfo sections_info(psmm, &route_sections);

  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);
  SendLaneGraphToCanvas(lane_graph, psmm, sections_info,
                        "target_lane_filter/fork_merge/lane_graph");

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(-2963.7);
  pose.mutable_path_point()->set_y(-1486.3);
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info, mapping::LanePath(), pose,
      /*preferred_lane_path=*/mapping::LanePath(),
      /*lc_preview_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 2);
  EXPECT_EQ(target_lp_infos[0].lane_path().lane_ids()[1],
            mapping::ElementId(39055));

  for (int i = 0; i < target_lp_infos.size(); ++i) {
    SendLanePathInfoToCanvas(
        target_lp_infos[i], psmm,
        absl::StrCat("target_lane_filter/fork_merge/target_lane_",
                     target_lp_infos[i].start_lane_id()));
  }
}

TEST(TargetLanePathFilter, IsSolidLaneChangeTest) {
  FLAGS_planner_est_parallel_branch_num = 3;
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePoint destination(mapping::ElementId(14185), 0.2);
  const RouteSections route_sections(
      0.0, 0.2, {mapping::SectionId(14327), mapping::SectionId(14335)},
      destination);
  const RouteSectionsInfo sections_info(psmm, &route_sections);

  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(151.850);
  pose.mutable_path_point()->set_y(-474.507);
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info, mapping::LanePath(), pose,
      /*preferred_lane_path=*/mapping::LanePath(),
      /*lc_preview_lane_path=*/mapping::LanePath(), &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 2);
  EXPECT_FALSE(target_lp_infos[0].is_solid_lane_change());
  EXPECT_TRUE(target_lp_infos[1].is_solid_lane_change());
}

TEST(TargetLanePathFilter, LcPreviewLanePathTest) {
  FLAGS_planner_est_parallel_branch_num = 2;
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePoint destination(mapping::ElementId(1), 0.8);
  const RouteSections route_sections(
      0.0, 0.8, {mapping::SectionId(12401), mapping::SectionId(12400)},
      destination);
  const RouteSectionsInfo sections_info(psmm, &route_sections);

  const auto route_navi_info =
      *CalcNaviInfoByLaneGraph(*smm, sections_info,
                               /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);

  auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);

  ApolloTrajectoryPointProto pose;
  pose.mutable_path_point()->set_x(0.0);
  pose.mutable_path_point()->set_y(0.0);
  pose.set_v(0.0);

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      psmm, sections_info, route_navi_info,
      mapping::LanePath(smm, {mapping::ElementId(2448), mapping::ElementId(1)},
                        /*start_fraction=*/0.0, /*end_fraction=*/0.8),
      pose,
      /*preferred_lane_path=*/mapping::LanePath(),
      mapping::LanePath(smm, {mapping::ElementId(3), mapping::ElementId(950)},
                        /*start_fraction=*/0.0, /*end_fraction=*/0.8),
      &lp_infos);

  EXPECT_EQ(target_lp_infos.size(), 2);
  EXPECT_EQ(target_lp_infos[0].start_lane_id(), mapping::ElementId(2448));
  EXPECT_EQ(target_lp_infos[1].start_lane_id(), mapping::ElementId(3));
}

}  // namespace

}  // namespace qcraft::planner
