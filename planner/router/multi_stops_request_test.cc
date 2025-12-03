#include "onboard/planner/router/multi_stops_request.h"

#include <algorithm>

#include "absl/hash/hash.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/test_util.h"

// #include "onboard/vis/canvas/canvas.h"
namespace qcraft {
namespace planner {
namespace {

TEST(MultiStopsRequest, Proto) {
  SetMap("dojo");
  const auto& semantic_map_manager = LoadDojoMap();
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

  MultipleStopsRequestProto multi_stops_proto;
  multi_stops_proto.set_infinite_loop(false);

  auto* stop = multi_stops_proto.add_stops();
  *stop->add_via_points() = des1;
  *stop->mutable_stop_point() = des2;
  stop->set_stop_name("111");

  *request.mutable_multi_stops() = multi_stops_proto;

  *request.mutable_id() = "test";

  MultipleStopsRequest multi_stops =
      BuildMultipleStopsRequest(semantic_map_manager, map_index, request)
          .value();

  RoutingRequestProto result_request;
  multi_stops.ToRoutingRequestProto(&result_request);

  EXPECT_EQ("test", multi_stops.request_id());
  EXPECT_EQ(1, multi_stops.stop_size());
  EXPECT_EQ(2, multi_stops.destination_size());
  EXPECT_EQ(true, multi_stops.skip_past_stops());
  EXPECT_EQ(false, multi_stops.infinite_loop());

  EXPECT_THAT(request, ProtoEq(result_request));
  EXPECT_FALSE(multi_stops.empty());
  const auto& multi_stops_request_proto =
      multi_stops.multi_stops_request_proto();
  EXPECT_TRUE(ProtoEquals(multi_stops_proto, multi_stops_request_proto));

  const auto& first_index_dest = multi_stops.destination(0);
  const auto& first_dest = multi_stops.first_destination();
  EXPECT_TRUE(ProtoEquals(first_index_dest, first_dest));
  EXPECT_TRUE(ProtoEquals(first_index_dest, des1));

  const auto& dest_protos = multi_stops.destinations_proto();
  EXPECT_EQ(dest_protos.size(), 1);
  EXPECT_EQ(dest_protos[0].size(), 2);
  const auto& stop_lists = multi_stops.stop_name_list();
  EXPECT_EQ(stop_lists.size(), 1);
}

TEST(MultiStopsRequest, AvoidRegionsTest) {
  const auto& semantic_map_manager = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();

  const auto cc = CoordinateConverter::FromMap("dojo");
  RoutingRequestProto request;

  auto* avoid_regions_proto = request.add_avoid_regions();
  auto* p1 = avoid_regions_proto->add_points();
  p1->set_longitude(-2.3556822693e-6);
  p1->set_latitude(-8.6365235327e-6);
  p1->set_altitude(0.0);

  auto* p2 = avoid_regions_proto->add_points();
  p2->set_longitude(-2.9908951475e-6);
  p2->set_latitude(-8.4843551316e-6);
  p2->set_altitude(0.0);

  auto* p3 = avoid_regions_proto->add_points();
  p3->set_longitude(-2.9703907077e-6);
  p3->set_latitude(-2.3552518245e-6);
  p3->set_altitude(0.0);

  auto* p4 = avoid_regions_proto->add_points();
  p4->set_longitude(-2.2982599043e-6);
  p4->set_latitude(-2.1095091197e-6);
  p4->set_altitude(0.0);

  MultipleStopsRequest multi_stops =
      BuildMultipleStopsRequest(semantic_map_manager, map_index, request)
          .value();

  absl::flat_hash_set<mapping::ElementId> avoid_lanes = {
      mapping::ElementId(214)};

  EXPECT_THAT(multi_stops.avoid_lanes(), avoid_lanes);

  auto points = mapping::ConverterGeoPoints(avoid_regions_proto->points());
  std::vector<Vec3d> points3d;
  for (const auto& pt : points) {
    const auto t = cc.GlobalToSmooth(Vec3d(pt));
    points3d.push_back(t);
  }
  EXPECT_EQ(points3d.size(), 4);
  // vis::Canvas& canvas = vantage_client_man::GetCanvas("debug");
  // canvas.DrawPolygon(points3d, vis::Color::kRed, 7.0);
  // vantage_client_man::FlushAll();
}
TEST(MultiStopsRequest, RoutingRequest) {
  SetMap("dojo");
  const auto& semantic_map_manager = LoadDojoMap();
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
  des1.mutable_global_point()->set_longitude(-1.8552155302746883e-08);
  des1.mutable_global_point()->set_latitude(1.6905497148730788e-09);
  des1.mutable_global_point()->set_altitude(0.0);

  MultipleStopsRequestProto multi_stops_proto;
  multi_stops_proto.set_infinite_loop(false);

  auto* stop = multi_stops_proto.add_stops();
  *stop->add_via_points() = des1;
  *stop->mutable_stop_point() = des2;
  stop->set_stop_name("111");
  auto* stop2 = multi_stops_proto.add_stops();
  *stop2->mutable_stop_point() = des1;
  stop2->set_stop_name("111");
  *request.mutable_multi_stops() = multi_stops_proto;

  *request.mutable_id() = "test";
  request.add_avoid_lanes(214);

  MultipleStopsRequest multiple_stops_request =
      BuildMultipleStopsRequest(semantic_map_manager, map_index, request)
          .value();
  RoutingRequestProto other;
  multiple_stops_request.ToRoutingRequestProto(&other);
  EXPECT_EQ(other, request);
  EXPECT_EQ("test", request.id());
  EXPECT_EQ(1, other.avoid_lanes_size());
  EXPECT_EQ(214, other.avoid_lanes(0));

  RoutingRequestProto next =
      multiple_stops_request
          .GenerateRoutingRequestProtoToNextStop(semantic_map_manager, 1)
          .value();

  EXPECT_EQ("test", next.id());
  EXPECT_EQ(1, next.avoid_lanes_size());
  EXPECT_EQ(214, next.avoid_lanes(0));
}

TEST(MultiStopsRequest, LanePointsTest) {
  SetMap("dojo");
  const auto& semantic_map_manager = LoadDojoMap();
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

  MultipleStopsRequestProto multi_stops_proto;
  multi_stops_proto.set_infinite_loop(false);

  auto* stop = multi_stops_proto.add_stops();
  *stop->add_via_points() = des1;
  *stop->mutable_stop_point() = des2;
  stop->set_stop_name("111");

  RoutingDestinationProto des3;
  des3.set_named_spot("b6_e3_end");

  auto* stop2 = multi_stops_proto.add_stops();
  *stop2->mutable_stop_point() = des3;
  stop2->set_stop_name("222");

  *request.mutable_multi_stops() = multi_stops_proto;

  MultipleStopsRequest multiple_stops_request =
      BuildMultipleStopsRequest(semantic_map_manager, map_index, request)
          .value();

  std::vector<std::vector<mapping::LanePoint>> lane_points = {
      {mapping::LanePoint(mapping::ElementId(56), 1.000000),
       mapping::LanePoint(mapping::ElementId(1), 0.000000)},
      {mapping::LanePoint(mapping::ElementId(3), 0.989193)}};

  bool expected =
      (multiple_stops_request.lane_point_destinations()[1][0].fraction() -
       lane_points[1][0].fraction()) < 1e-6;

  EXPECT_TRUE(expected);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
