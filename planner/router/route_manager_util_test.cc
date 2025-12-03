#include "onboard/planner/router/route_manager_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"

#include "gtest/gtest.h"

#include "common/proto/lane_point.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {
namespace {

TEST(RouteManagerUtilTest, FindNextDestinationIndexTest) {
  SetMap("dojo");
  const auto& semantic_map_manager = LoadDojoMap();
  const auto& map_index = *LoadDojoIndex();

  std::vector<std::vector<mapping::LanePoint>> lane_point_destinations = {
      {mapping::LanePoint(mapping::ElementId(2448), 0.1),
       mapping::LanePoint(mapping::ElementId(2448), 0.5),
       mapping::LanePoint(mapping::ElementId(1), 0.5)},
      {mapping::LanePoint(mapping::ElementId(2471), 0.3),
       mapping::LanePoint(mapping::ElementId(2471), 0.9)},
      {mapping::LanePoint(mapping::ElementId(53), 0.5)}};

  RoutingRequestProto routing_request;

  RoutingDestinationProto des1;
  des1.mutable_lane_point()->set_lane_id(2448);
  des1.mutable_lane_point()->set_fraction(0.1);

  RoutingDestinationProto des2;
  des2.mutable_lane_point()->set_lane_id(2448);
  des2.mutable_lane_point()->set_fraction(0.5);

  RoutingDestinationProto des3;
  des3.mutable_lane_point()->set_lane_id(1);
  des3.mutable_lane_point()->set_fraction(0.5);

  RoutingDestinationProto des4;
  des4.mutable_lane_point()->set_lane_id(2471);
  des4.mutable_lane_point()->set_fraction(0.3);

  RoutingDestinationProto des5;
  des5.mutable_lane_point()->set_lane_id(2471);
  des5.mutable_lane_point()->set_fraction(0.9);

  RoutingDestinationProto des6;
  des6.mutable_lane_point()->set_lane_id(53);
  des6.mutable_lane_point()->set_fraction(0.5);

  MultipleStopsRequestProto multi_stops_proto;

  auto* stop1 = multi_stops_proto.add_stops();
  *stop1->add_via_points() = des1;
  *stop1->add_via_points() = des2;
  *stop1->mutable_stop_point() = des3;
  stop1->set_stop_name("1");

  auto* stop2 = multi_stops_proto.add_stops();
  *stop2->add_via_points() = des4;
  *stop2->mutable_stop_point() = des5;
  stop2->set_stop_name("2");

  auto* stop3 = multi_stops_proto.add_stops();
  *stop3->mutable_stop_point() = des6;
  stop3->set_stop_name("3");

  multi_stops_proto.set_infinite_loop(true);

  *routing_request.mutable_multi_stops() = multi_stops_proto;

  MultipleStopsRequest multi_stops_request =
      BuildMultipleStopsRequest(semantic_map_manager, &map_index,
                                routing_request)
          .value();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);

  route::RouteCore route_core(&semantic_map_manager, &route_params);

  // Shoule add IsPointOnSection UT independent
  const auto next_idx_or = FindNextDestinationIndexViaLanePoints(
      semantic_map_manager, multi_stops_request,
      [&](const auto& seq) {
        SMM_ASSIGN_LANE_OR_RETURN(lane, semantic_map_manager,
                                  mapping::ElementId(1), false);
        return seq.front().id == mapping::SectionId(lane.Proto().section_id());
      },
      &route_core);
  if (!next_idx_or.ok()) {
    EXPECT_TRUE(false);
  }
  EXPECT_EQ(next_idx_or.value(), 3);
}

TEST(RouteManagerUtilTest, GenerateAndCheckRoutingRequestProtoToNextStopTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);

  RoutingRequestProto routing_request;

  RoutingDestinationProto des1;
  des1.mutable_global_point()->set_longitude(9.8966163192525165e-06);
  des1.mutable_global_point()->set_latitude(9.100590669662013e-09);
  des1.mutable_global_point()->set_altitude(0.0);

  MultipleStopsRequestProto multi_stops_proto;

  auto* stop1 = multi_stops_proto.add_stops();
  *stop1->mutable_stop_point() = des1;

  *routing_request.mutable_multi_stops() = multi_stops_proto;

  MultipleStopsRequest multi_stops_request =
      BuildMultipleStopsRequest(smm, map_index, routing_request).value();

  const Vec2d global_point(8.79418430011424e-06, 1.1854678626358358e-08);

  const auto routing_request_or = GenerateAndCheckRoutingRequestProtoToNextStop(
      smm, map_index, route_params, multi_stops_request, global_point,
      /*heading=*/0.0, /*next_destination_index=*/0);

  EXPECT_OK(routing_request_or);
  EXPECT_TRUE(!routing_request_or->destinations().empty());
}

TEST(RouteManagerUtilTest, BackWardExtendSectionsFromCurrentAlongRouteTest) {
  const auto& smm = LoadDojoMap();

  RouteSectionSequenceProto sections_proto;
  sections_proto.set_start_fraction(0.1);
  sections_proto.set_end_fraction(0.8);
  auto* section_ids_ptr = sections_proto.mutable_section_id();
  section_ids_ptr->Add(12401);
  section_ids_ptr->Add(12400);
  section_ids_ptr->Add(12408);

  sections_proto.mutable_destination()->set_lane_id(34);
  sections_proto.mutable_destination()->set_fraction(0.8);

  const auto back_extend_sections_pair_or_1 =
      BackWardExtendSectionsFromCurrentAlongRoute(
          smm, sections_proto,
          mapping::LanePoint(mapping::ElementId(2448), 0.11),
          /*start_sec_idx=*/0, /*extend_length=*/10.0);

  EXPECT_FALSE(back_extend_sections_pair_or_1.ok());

  const auto back_extend_sections_pair_or_2 =
      BackWardExtendSectionsFromCurrentAlongRoute(
          smm, sections_proto,
          mapping::LanePoint(mapping::ElementId(2448), 0.3),
          /*start_sec_idx=*/0, /*extend_length=*/10.0);
  EXPECT_TRUE(back_extend_sections_pair_or_2.ok());
  EXPECT_NEAR(back_extend_sections_pair_or_2->second.start_fraction(), 0.121,
              0.01);
  EXPECT_EQ(back_extend_sections_pair_or_2->second.size(), 3);
}

TEST(RouteManagerUtilTest, TrackAlternateRouteTest) {
  const auto& smm = LoadDojoMap();
  const auto route_params = CreateDefaultRouteParam();
  route::RouteCore route_core(&smm, &route_params);
  route::RouteCorePrecondition route_pre;
  route_pre.is_bus = true;
  route_pre.use_time = true;

  mapping::PointToLane origin1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(2448)),
      .dist = 0.5,
      .fraction = 0.5};

  mapping::PointToLane dest1{
      .lane_proto = mapping::FindLaneProto(smm, mapping::ElementId(89)),
      .dist = 0.5,
      .fraction = 0.9};

  const auto route_result_or =
      route_core.SearchForRoutePathFromLanePoint(route_pre, {origin1}, {dest1});
  EXPECT_TRUE(route_result_or.ok());

  RouteProto route;
  route_result_or->second.ToProto(route.mutable_lane_path());
  route_result_or->first.ToProto(route.mutable_route_section_sequence());

  const auto updated_route_or = TrackAlternateRoute(
      smm, route, mapping::LanePoint(mapping::ElementId(951), 0.1),
      /*travel_dist=*/50);
  EXPECT_TRUE(updated_route_or.ok());
  EXPECT_EQ(updated_route_or->lane_path().lane_paths(0).lane_ids(0), 951);
  EXPECT_EQ(updated_route_or->lane_path().lane_paths(0).start_fraction(), 0.1);
  EXPECT_EQ(updated_route_or->route_section_sequence().section_id(0), 12082);
}

TEST(RouteManagerUtilTest, CalcCurRouteNaviInfoTest) {
  const auto& smm = LoadDojoMap();

  RouteSectionSequenceProto global_sections_proto;
  global_sections_proto.set_start_fraction(0.1);
  global_sections_proto.set_end_fraction(0.8);
  auto* global_section_ids_ptr = global_sections_proto.mutable_section_id();
  global_section_ids_ptr->Add(12401);
  global_section_ids_ptr->Add(12400);
  global_section_ids_ptr->Add(12408);

  {
    RouteSectionSequenceProto cur_sections_proto;
    cur_sections_proto.set_start_fraction(0.13);
    cur_sections_proto.set_end_fraction(0.8);
    auto* cur_section_ids_ptr = cur_sections_proto.mutable_section_id();
    cur_section_ids_ptr->Add(12401);
    cur_section_ids_ptr->Add(12400);
    cur_section_ids_ptr->Add(12408);

    const auto route_navi_info_or = CalcCurRouteNaviInfo(
        smm, global_sections_proto, cur_sections_proto, /*avoid_lanes=*/{},
        mapping::LanePoint(mapping::ElementId(3), 0.13), /*extend_length=*/10.0,
        /*preview_dist=*/100.0);
    EXPECT_TRUE(route_navi_info_or.ok());
    EXPECT_TRUE(route_navi_info_or->route_lane_info_map.contains(
        mapping::ElementId(57)));
    EXPECT_TRUE(route_navi_info_or->route_lane_info_map.contains(
        mapping::ElementId(267)));
  }
  // Add v2smm test
  {
    auto loader =
        mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
    auto map_fut = loader->PreloadWholeMap();
    std::shared_ptr<mapping::v2::SemanticMapManager> v2smm = map_fut.Get();

    RouteSectionSequenceProto cur_sections_proto;
    cur_sections_proto.set_start_fraction(0.3);
    cur_sections_proto.set_end_fraction(0.8);
    auto* cur_section_ids_ptr = cur_sections_proto.mutable_section_id();
    cur_section_ids_ptr->Add(12401);
    cur_section_ids_ptr->Add(12400);
    cur_section_ids_ptr->Add(12408);

    const auto route_navi_info_or = CalcCurRouteNaviInfo(
        *v2smm, global_sections_proto, cur_sections_proto, /*avoid_lanes=*/{},
        mapping::LanePoint(mapping::ElementId(3), 0.13), /*extend_length=*/10.0,
        /*preview_dist=*/100.0);
    EXPECT_TRUE(route_navi_info_or.ok());
    EXPECT_TRUE(route_navi_info_or->route_lane_info_map.contains(
        mapping::ElementId(3)));
    EXPECT_TRUE(route_navi_info_or->route_lane_info_map.contains(
        mapping::ElementId(2)));
  }
}

TEST(RouteManagerUtilTest, RecoverRoutingRequestTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  RoutingRequestProto routing_request;

  RoutingDestinationProto des1;
  des1.mutable_global_point()->set_longitude(9.8966163192525165e-06);
  des1.mutable_global_point()->set_latitude(9.100590669662013e-09);
  des1.mutable_global_point()->set_altitude(0.0);

  MultipleStopsRequestProto multi_stops_proto;

  auto* stop1 = multi_stops_proto.add_stops();
  *stop1->mutable_stop_point() = des1;

  *routing_request.mutable_multi_stops() = multi_stops_proto;

  const auto multi_stops =
      RecoverRoutingRequest(routing_request, smm, map_index);
  EXPECT_OK(multi_stops);
  EXPECT_EQ(multi_stops->destination_size(), 1);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
