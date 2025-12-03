#include "onboard/planner/router/navi/route_navi_info_builder.h"

#include <memory>
#include <optional>
#include <tuple>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/lane_point.pb.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_core_searcher.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/util/map_index.h"

namespace qcraft::planner {
namespace {
TEST(NaviInfoTest, CalculateNaviInfo) {
  const auto& semantic_map_manager = LoadDojoMap();
  const route::MapIndex* map_index = LoadDojoIndex();
  const auto route_params = CreateDefaultRouteParam();

  mapping::LanePoint origin(mapping::ElementId(2448), 0.2);

  absl::flat_hash_set<mapping::ElementId> blacklist;

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(93);
  dest1.mutable_lane_point()->set_fraction(0.9);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const route::RoutingRequestContext route_context = {
      .semantic_map_manager = &semantic_map_manager,
      .map_index = map_index,
      .route_param_proto = &route_params,
      .pred_trajectory_point = std::nullopt};

  route::RouteCore route_core(&semantic_map_manager, &route_params);
  const auto sections_lanes_or =
      planner::route::SearchForRouteCorePathToRoutingRequest(
          route_context, routing_request,
          {{
              .lane_proto = mapping::FindLaneProto(semantic_map_manager,
                                                   origin.lane_id()),
              .dist = 0.0,
              .fraction = origin.fraction(),
          }},
          {
              .is_bus = true,
              .use_time = true,
          },
          &route_core);
  EXPECT_OK(sections_lanes_or);
  const auto& route_sections = sections_lanes_or->first;

  const auto sections_info_or = BuildRouteSectionsInfoFromData(
      semantic_map_manager, &route_sections, /*planning_horizon=*/0.0);
  EXPECT_OK(sections_info_or);

  const auto navi_info_or =
      CalcNaviInfoByLaneGraph(semantic_map_manager, *sections_info_or,
                              /*avoid_lanes=*/{}, /*preview_dist=*/100.0);
  EXPECT_OK(navi_info_or);

  const auto& navi_info = *navi_info_or;
  EXPECT_TRUE(navi_info.route_lane_info_map.contains(mapping::ElementId(2)));
  EXPECT_EQ(navi_info.route_lane_info_map.at(mapping::ElementId(2))
                .min_lc_num_to_target,
            2);

  EXPECT_TRUE(navi_info.route_lane_info_map.contains(mapping::ElementId(2448)));
  EXPECT_EQ(navi_info.route_lane_info_map.at(mapping::ElementId(2448))
                .min_lc_num_to_target,
            1);

  EXPECT_TRUE(navi_info.route_lane_info_map.contains(mapping::ElementId(88)));
  EXPECT_EQ(navi_info.route_lane_info_map.at(mapping::ElementId(88))
                .min_lc_num_to_target,
            0);
  // BANDAIT(zuowei): Just for UT coverage.
  std::ignore = navi_info.DebugString();
}

TEST(NaviInfoTest, CalculateNaviInfoSpecialCaseTest) {
  const auto& semantic_map_manager = LoadDojoMap();
  const route::MapIndex* map_index = LoadDojoIndex();
  const auto route_params = CreateDefaultRouteParam();

  mapping::LanePoint origin(mapping::ElementId(2448), 1.0);

  absl::flat_hash_set<mapping::ElementId> blacklist;

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(2470);
  dest1.mutable_lane_point()->set_fraction(0.9);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const route::RoutingRequestContext route_context = {
      .semantic_map_manager = &semantic_map_manager,
      .map_index = map_index,
      .route_param_proto = &route_params,
      .pred_trajectory_point = std::nullopt};

  route::RouteCore route_core(&semantic_map_manager, &route_params);
  const auto sections_lanes_or =
      planner::route::SearchForRouteCorePathToRoutingRequest(
          route_context, routing_request,
          {{
              .lane_proto = mapping::FindLaneProto(semantic_map_manager,
                                                   origin.lane_id()),
              .dist = 0.0,
              .fraction = origin.fraction(),
          }},
          {
              .is_bus = true,
              .use_time = true,
          },
          &route_core);
  EXPECT_OK(sections_lanes_or);
  const auto& route_sections = sections_lanes_or->first;

  const auto sections_info_or = BuildRouteSectionsInfoFromData(
      semantic_map_manager, &route_sections, /*planning_horizon=*/0.0);
  EXPECT_OK(sections_info_or);

  const auto navi_info_or =
      CalcNaviInfoByLaneGraph(semantic_map_manager, *sections_info_or,
                              /*avoid_lanes=*/{}, /*preview_dist=*/100.0);
  EXPECT_OK(navi_info_or);

  const auto& navi_info = *navi_info_or;

  EXPECT_TRUE(navi_info.route_lane_info_map.contains(mapping::ElementId(2448)));
  EXPECT_EQ(navi_info.route_lane_info_map.at(mapping::ElementId(2448))
                .min_lc_num_to_target,
            1);
  EXPECT_NEAR(navi_info.route_lane_info_map.at(mapping::ElementId(2448))
                  .max_reach_length,
              156, 1);

  // Add v2smm test
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  std::shared_ptr<mapping::v2::SemanticMapManager> v2smm = map_fut.Get();

  const auto navi_info_v2 = CalcNaviInfoByLaneGraph(
      *v2smm, *sections_info_or, /*avoid_lanes=*/{}, /*preview_dist=*/100.0);
  EXPECT_OK(navi_info_v2);
  EXPECT_TRUE(
      navi_info_v2->route_lane_info_map.contains(mapping::ElementId(2448)));
  EXPECT_EQ(navi_info_v2->route_lane_info_map.at(mapping::ElementId(2448))
                .min_lc_num_to_target,
            1);
  EXPECT_NEAR(navi_info_v2->route_lane_info_map.at(mapping::ElementId(2448))
                  .max_reach_length,
              156, 1);
}

TEST(NaviInfoTest, CalculateNaviInfoForMergeLane) {
  const auto& semantic_map_manager = LoadDojoMap();
  const route::MapIndex* map_index = LoadDojoIndex();
  const auto route_params = CreateDefaultRouteParam();

  mapping::LanePoint origin(mapping::ElementId(34242), 0.8);

  absl::flat_hash_set<mapping::ElementId> blacklist;

  RoutingRequestProto routing_request;
  RoutingDestinationProto dest1;
  dest1.mutable_lane_point()->set_lane_id(33676);
  dest1.mutable_lane_point()->set_fraction(0.9);
  routing_request.mutable_destinations()->Add(std::move(dest1));

  const route::RoutingRequestContext route_context = {
      .semantic_map_manager = &semantic_map_manager,
      .map_index = map_index,
      .route_param_proto = &route_params,
      .pred_trajectory_point = std::nullopt};

  route::RouteCore route_core(&semantic_map_manager, &route_params);
  const auto sections_lanes_or =
      planner::route::SearchForRouteCorePathToRoutingRequest(
          route_context, routing_request,
          {{
              .lane_proto = mapping::FindLaneProto(semantic_map_manager,
                                                   origin.lane_id()),
              .dist = 0.0,
              .fraction = origin.fraction(),
          }},
          {
              .is_bus = true,
              .use_time = true,
          },
          &route_core);
  EXPECT_OK(sections_lanes_or);
  const auto& route_sections = sections_lanes_or->first;

  const auto sections_info_or = BuildRouteSectionsInfoFromData(
      semantic_map_manager, &route_sections, /*planning_horizon=*/0.0);
  EXPECT_OK(sections_info_or);

  const auto navi_info_or =
      CalcNaviInfoByLaneGraph(semantic_map_manager, *sections_info_or,
                              /*avoid_lanes=*/{}, /*preview_dist=*/2000.0);
  EXPECT_OK(navi_info_or);

  const auto& navi_info = *navi_info_or;

  EXPECT_TRUE(
      navi_info.route_lane_info_map.contains(mapping::ElementId(34242)));
  EXPECT_EQ(
      navi_info.route_lane_info_map.at(mapping::ElementId(32616)).merge_targets,
      absl::flat_hash_set<mapping::ElementId>{mapping::ElementId(32620)});
  EXPECT_EQ(
      navi_info.route_lane_info_map.at(mapping::ElementId(34242)).merge_targets,
      absl::flat_hash_set<mapping::ElementId>{mapping::ElementId(34234)});
  EXPECT_NEAR(navi_info.route_lane_info_map.at(mapping::ElementId(34242))
                  .len_before_merge_lane,
              690, 1);
}

}  // namespace
}  // namespace qcraft::planner
