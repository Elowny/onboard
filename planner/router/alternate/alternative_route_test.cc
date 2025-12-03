#include "onboard/planner/router/alternate/alternative_route.h"

#include <cstdint>
#include <vector>

#include "common/proto/lane_point.pb.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
namespace {
TEST(AlternativeRouteTest, FindAlternateRouteTest) {
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();
  const auto route_params = CreateDefaultRouteParam();

  RouteCore route_core(&smm, &route_params);

  const RoutingRequestContext route_context{.semantic_map_manager = &smm,
                                            .map_index = map_index,
                                            .route_param_proto = &route_params,
                                            .request_id = 1};

  RoutingRequestProto routing_request;

  RoutingDestinationProto des1;
  des1.mutable_lane_point()->set_lane_id(2448);
  des1.mutable_lane_point()->set_fraction(0.8);

  RoutingDestinationProto des2;
  des2.mutable_lane_point()->set_lane_id(2470);
  des2.mutable_lane_point()->set_fraction(0.8);

  MultipleStopsRequestProto multi_stops_proto;

  auto* stop1 = multi_stops_proto.add_stops();
  *stop1->add_via_points() = des1;
  *stop1->mutable_stop_point() = des2;

  multi_stops_proto.set_infinite_loop(false);

  *routing_request.mutable_multi_stops() = multi_stops_proto;
  *routing_request.add_destinations() = des1;
  *routing_request.add_destinations() = des2;

  const MultipleStopsRequest multi_stops_request =
      BuildMultipleStopsRequest(smm, map_index, routing_request).value();

  RouteSectionSequenceProto section_seq_proto;

  section_seq_proto.set_start_fraction(0.2);
  section_seq_proto.set_end_fraction(0.8);

  mapping::LanePointProto dest;
  dest.set_lane_id(2470);
  dest.set_fraction(0.8);

  *section_seq_proto.mutable_destination() = dest;

  section_seq_proto.add_section_id(12401);
  section_seq_proto.add_section_id(12400);
  section_seq_proto.add_section_id(12408);
  section_seq_proto.add_section_id(12412);

  AlternateRouteInput alter_input{
      .origin = mapping::LanePoint(mapping::ElementId(2448), 0.2),
      .is_bus = true,
      .look_ahead_dist = 500.0,
      .cost_per_meter = 3.0,
      .primary_sections = &section_seq_proto,
      .route_context = route_context,
      .routing_request = &routing_request,
      .multi_stops = &multi_stops_request};

  const auto alter_route_or = FindAlternateRoute(alter_input, &route_core);
  EXPECT_TRUE(alter_route_or.ok());
  EXPECT_THAT(
      alter_route_or->route_from_current.route_section_sequence().section_id(),
      testing::ElementsAreArray(std::vector<int64_t>{
          12401, 12082, 12317, 12399, 12068, 12402, 12081, 12318, 12412}));
}
}  // namespace
}  // namespace qcraft::planner::route
