#include "onboard/planner/router/interface/generate_route.h"

#include "absl/container/flat_hash_set.h"

#include "gtest/gtest.h"
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/proto_util.h"

namespace qcraft::planner::route {
namespace {

// V1 to V2
TEST(GenerateRouteTest, V1ToV2Test) {
  const auto& smm = LoadDojoMapV1();
  MapIndex map_index;
  map_index.InitIndex(smm);
  {
    const mapping::LanePoint origin(mapping::ElementId(2448), 0.9);
    const mapping::LanePoint dest1(mapping::ElementId(2470), 0.9);
    const mapping::LanePoint dest2(mapping::ElementId(64), 0.9);
    const auto sections_clp_or = SearchRouteResultsBetweenLanePoints(
        smm, origin, {dest1, dest2}, /*is_bus=*/true);
    EXPECT_OK(sections_clp_or);
    EXPECT_EQ(sections_clp_or->first.size(), 9);
    EXPECT_EQ(sections_clp_or->second.lane_paths().size(), 2);
  }

  {
    const Vec3d origin(-1.8552155302746883e-08, 1.6905497148730788e-09, 0.0);
    const Vec3d dest(-1.8552155302746883e-08, 1.6905497148730788e-09, 0.0);
    const auto clp_or =
        SearchLanePathBetweenVec3ds(smm, &map_index, origin, dest,
                                    /*is_bus=*/true);
    EXPECT_OK(clp_or);
    EXPECT_EQ(clp_or->lane_paths().size(), 1);
  }

  {
    const mapping::LanePoint origin(mapping::ElementId(2448), 0.9);
    const mapping::LanePoint dest1(mapping::ElementId(2470), 0.9);
    const mapping::LanePoint dest2(mapping::ElementId(64), 0.9);
    const auto clp_or =
        SearchLanePathBetweenLanePoints(smm, origin, {dest1, dest2},
                                        /*is_bus=*/true);
    EXPECT_OK(clp_or);
    EXPECT_EQ(clp_or->lane_paths().size(), 2);
  }
}

TEST(GenerateRouteTest, SearchRouteResultsBetweenDestinationsTest) {
  const auto& smm = LoadDojoMap();
  const auto& map_index = LoadDojoIndex();

  google::protobuf::RepeatedPtrField<RoutingDestinationProto> dests;
  dests.Reserve(3);

  RoutingDestinationProto dest1;
  dest1.mutable_global_point()->set_longitude(-1.8552155302746883e-08);
  dest1.mutable_global_point()->set_latitude(1.6905497148730788e-09);
  dest1.mutable_global_point()->set_altitude(0.0);
  dests.Add(std::move(dest1));

  RoutingDestinationProto dest2;
  dest2.mutable_global_point()->set_longitude(3.383218513364392e-05);
  dest2.mutable_global_point()->set_latitude(-5.5029147397546393e-07);
  dest2.mutable_global_point()->set_altitude(0.0);
  dests.Add(std::move(dest2));

  RoutingDestinationProto dest3;
  dest3.mutable_global_point()->set_longitude(4.23146843552218e-05);
  dest3.mutable_global_point()->set_latitude(-5.4942489436951642e-06);
  dest3.mutable_global_point()->set_altitude(0.0);
  dests.Add(std::move(dest3));

  RouteRestrictDistrict route_restrict;
  route_restrict.avoid_lanes.insert(mapping::ElementId(2471));

  const auto route_result = SearchRouteResultsBetweenDestinations(
      smm, map_index, dests, route_restrict, /*is_bus=*/true);
  EXPECT_OK(route_result);
  EXPECT_EQ(route_result->first.size(), 7);
  EXPECT_EQ(route_result->second.num_lane_paths(), 2);
}

TEST(GenerateRouteTest, SearchRouteResultsBetweenLanePointsTest) {
  const auto& smm = LoadDojoMap();

  const mapping::LanePoint origin(mapping::ElementId(2448), 0.9);
  const mapping::LanePoint dest1(mapping::ElementId(2470), 0.9);
  const mapping::LanePoint dest2(mapping::ElementId(64), 0.9);
  const auto sections_clp_or = SearchRouteResultsBetweenLanePoints(
      smm, origin, {dest1, dest2}, /*is_bus=*/true);
  EXPECT_OK(sections_clp_or);
  EXPECT_EQ(sections_clp_or->first.size(), 9);
  EXPECT_EQ(sections_clp_or->second.lane_paths().size(), 2);
}

TEST(GenerateRouteTest, GenerateRoutePathFromStopsTest) {
  const auto& smm = LoadDojoMap();

  const mapping::LanePoint dest0(mapping::ElementId(2448), 0.9);
  const mapping::LanePoint dest1(mapping::ElementId(2470), 0.9);
  const mapping::LanePoint dest2(mapping::ElementId(64), 0.9);
  const auto clp_or_1 =
      GenerateRoutePathFromStops(smm, {dest0, dest1, dest2}, {1, 2},
                                 /*infinite_loop=*/true, /*is_bus=*/true);
  EXPECT_OK(clp_or_1);
  EXPECT_EQ(clp_or_1->size(), 2);
  EXPECT_EQ(clp_or_1->at(0).lane_paths().size(), 1);
  EXPECT_EQ(clp_or_1->at(1).lane_paths().size(), 2);
}

TEST(GenerateRouteTest, GenerateRoutePathByRoutingRequestTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  RoutingRequestProto request;

  RoutingDestinationProto des1;
  des1.mutable_global_point()->set_longitude(-1.8552155302746883e-08);
  des1.mutable_global_point()->set_latitude(1.6905497148730788e-09);
  des1.mutable_global_point()->set_altitude(0.0);

  RoutingDestinationProto des2;
  des2.mutable_global_point()->set_longitude(8.79418430011424e-06);
  des2.mutable_global_point()->set_latitude(1.1854678626358358e-08);
  des2.mutable_global_point()->set_altitude(0.0);

  RoutingDestinationProto des3;
  des3.mutable_global_point()->set_longitude(-1.8552155302746883e-08);
  des3.mutable_global_point()->set_latitude(1.6905497148730788e-09);
  des3.mutable_global_point()->set_altitude(0.0);

  MultipleStopsRequestProto multi_stops_proto;
  multi_stops_proto.set_infinite_loop(false);

  auto* stop = multi_stops_proto.add_stops();
  *stop->add_via_points() = des1;
  *stop->mutable_stop_point() = des2;

  auto* stop2 = multi_stops_proto.add_stops();
  *stop2->mutable_stop_point() = des3;

  *request.mutable_multi_stops() = multi_stops_proto;

  const auto clp_or = GenerateRoutePathByRoutingRequest(smm, map_index, request,
                                                        /*is_bus=*/true);
  EXPECT_OK(clp_or);
  EXPECT_EQ(clp_or->size(), 2);
  EXPECT_EQ(clp_or->at(0).lane_paths().size(), 1);
  EXPECT_EQ(clp_or->at(1).lane_paths().size(), 2);
}

TEST(GenerateRouteTest, SearchLanePathBetweenLanePointsTest) {
  const auto& smm = LoadDojoMap();

  const mapping::LanePoint origin(mapping::ElementId(2448), 0.9);
  const mapping::LanePoint dest1(mapping::ElementId(2470), 0.9);
  const mapping::LanePoint dest2(mapping::ElementId(64), 0.9);
  const auto clp_or =
      SearchLanePathBetweenLanePoints(smm, origin, {dest1, dest2},
                                      /*is_bus=*/true);
  EXPECT_OK(clp_or);
  EXPECT_EQ(clp_or->lane_paths().size(), 2);
}

TEST(GenerateRouteTest, SearchLanePathBetweenVec3dsTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  const Vec3d origin(-1.8552155302746883e-08, 1.6905497148730788e-09, 0.0);
  const Vec3d dest(-1.8552155302746883e-08, 1.6905497148730788e-09, 0.0);
  const auto clp_or = SearchLanePathBetweenVec3ds(smm, map_index, origin, dest,
                                                  /*is_bus=*/true);
  EXPECT_OK(clp_or);
  EXPECT_EQ(clp_or->lane_paths().size(), 1);
}

TEST(GenerateRouteTest, SearchLanePathBetweenGeoPointsTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  mapping::GeoPointProto origin;
  origin.set_longitude(8.79418430011424e-06);
  origin.set_latitude(1.1854678626358358e-08);
  origin.set_altitude(0.0);

  mapping::GeoPointProto dest;
  dest.set_longitude(-1.8552155302746883e-08);
  dest.set_latitude(1.6905497148730788e-09);
  dest.set_altitude(0.0);

  const auto clp_or =
      SearchLanePathBetweenGeoPoints(smm, map_index, origin, dest,
                                     /*is_bus=*/true);
  EXPECT_OK(clp_or);
  EXPECT_EQ(clp_or->lane_paths().size(), 2);
}

TEST(GenerateRouteTest, SearchRoutingRequestTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();
  const auto cc = CoordinateConverter::FromMap("dojo");

  RoutingRequestProto request;

  RoutingDestinationProto des1;
  des1.mutable_global_point()->set_longitude(8.79418430011424e-06);
  des1.mutable_global_point()->set_latitude(1.1854678626358358e-08);
  des1.mutable_global_point()->set_altitude(0.0);

  RoutingDestinationProto des2;
  des2.mutable_global_point()->set_longitude(-1.8552155302746883e-08);
  des2.mutable_global_point()->set_latitude(1.6905497148730788e-09);
  des2.mutable_global_point()->set_altitude(0.0);

  auto* destinations_ptr = request.mutable_destinations();
  destinations_ptr->Add(std::move(des1));
  destinations_ptr->Add(std::move(des2));

  PoseProto origin;
  origin.mutable_pos_smooth()->set_x(0.0);
  origin.mutable_pos_smooth()->set_y(0.0);
  origin.mutable_pos_smooth()->set_z(0.0);

  const auto clp_or =
      SearchRoutingRequest(smm, cc, *map_index, request, origin);
  EXPECT_OK(clp_or);
  EXPECT_EQ(clp_or->lane_paths().size(), 2);
}

TEST(GenerateRouteTest, RouteSectionsTest) {
  const auto& smm = LoadDojoMap();

  const mapping::LanePoint dest1(mapping::ElementId(2448), 0.9);
  const mapping::LanePoint dest2(mapping::ElementId(2470), 0.9);
  const mapping::LanePoint dest3(mapping::ElementId(64), 0.9);
  const mapping::LanePoint dest4(mapping::ElementId(20), 0.9);

  const auto vec_sections_or = GenerateRouteSectionsFromStops(
      smm, {dest1, dest2, dest3, dest4}, {1, 3}, /*infinite_loop=*/true,
      /*is_bus=*/false);
  EXPECT_OK(vec_sections_or);
  const auto route_sections_or = GenerateTotalRouteSections(*vec_sections_or);
  EXPECT_OK(route_sections_or);
  EXPECT_EQ(route_sections_or->size(), 25);
  EXPECT_EQ(route_sections_or->section_ids().front(),
            route_sections_or->section_ids().back());
}

TEST(GenerateRouteTest, CompositeLanePathToRouteProto) {
  const auto clp_str = R"(
    lane_paths {
    lane_ids: 3
    lane_ids: 950
    lane_ids: 35
    lane_ids: 2472
    lane_ids: 54
    lane_ids: 168
    lane_ids: 129
    start_fraction: 0.5
    end_fraction: 0.5
    lane_path_in_forward_direction: true
    })";
  CompositeLanePathProto clp_proto;
  file_util::StringToProto(clp_str, &clp_proto);
  CompositeLanePath clp;
  const auto& smm = LoadDojoMap();
  clp.FromProto(clp_proto, &smm);
  RouteProto route = CompositeLanePathToRouteProto(clp);
  ASSERT_TRUE(ProtoEquals(route.lane_path(), clp_proto));
}

TEST(GenerateRouteTest, StatPathInfo) {
  const auto clp_str = R"(
    lane_paths {
    lane_ids: 3
    lane_ids: 950
    lane_ids: 35
    lane_ids: 2472
    lane_ids: 54
    lane_ids: 168
    lane_ids: 129
    start_fraction: 0.5
    end_fraction: 0.5
    lane_path_in_forward_direction: true
    })";
  CompositeLanePathProto clp_proto;
  file_util::StringToProto(clp_str, &clp_proto);
  CompositeLanePath clp;
  const auto& smm = LoadDojoMap();
  clp.FromProto(clp_proto, &smm);
  auto path_info_or = StatPathInfo(smm, clp);
  ASSERT_TRUE(path_info_or.ok());
  ASSERT_GT(path_info_or->length(), 0.0);
}

}  // namespace
}  // namespace qcraft::planner::route
