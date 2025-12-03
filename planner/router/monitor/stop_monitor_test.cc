#include "onboard/planner/router/monitor/stop_monitor.h"

#include <memory>

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/lane_point.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/planner/router/multi_stops_request.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/planner/router/util/map_index.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner::route {

namespace {
TEST(StopMonitor, IsNearBusStopTest) {
  const auto& smm = LoadDojoMap();
  ASSERT_TRUE(internal::IsNearBusStop(smm, mapping::ElementId(2448),
                                      /*fraction=*/0.0, {0.0, 0.0}, 1.0));
}
TEST(StopMonitor, MonitorStopStatusTest) {
  const auto& smm = LoadDojoMap();
  const route::MapIndex* map_index = LoadDojoIndex();
  const auto cc = CoordinateConverter::FromMap("dojo");

  RouteParamProto route_param = CreateDefaultRouteParam();
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

  MultipleStopsRequest multi_stops =
      BuildMultipleStopsRequest(smm, map_index, routing_request).value();
  const auto pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(117.471, 0.0), 0.0, Vec2d(0.0, 0.0));

  RoutingRequestContext context = {.semantic_map_manager = &smm,
                                   .map_index = map_index,
                                   .route_param_proto = &route_param,
                                   .request_id = 1};
  StopMonitorInput input = {
      .context = context,
      .multi_stops = &multi_stops,
      .rms_debug = nullptr,
      .pose = &pose,
      .next_stop_index = 1,
      .res_button_pressed = true,
      .is_bus = false,
  };
  RouteCore route_core(&smm, &route_param);
  absl::StatusOr<StopMonitorOutput> output_or =
      MonitorStopStatus(input, cc, &route_core);
  EXPECT_OK(output_or);
  EXPECT_TRUE(output_or->goto_next_stop);
  EXPECT_TRUE(output_or->routing_result_proto.success());

  FLAGS_route_auto_next_stop = true;
  {
    const bool goto_next_stop = ShouldGoNextStop(
        smm, map_index, multi_stops, /* next_stop_index=*/1, {0.0, 0.0});
    ASSERT_FALSE(goto_next_stop);
  }

  {
    const auto lane_pt = mapping::LanePoint(des5.lane_point());
    SMM_LANE_PROTO_OR_RETURN(lane_proto, smm, lane_pt.lane_id(), void());
    const auto pt3d =
        ComputePointOnLane(lane_proto->polyline().points(), lane_pt.fraction());

    const bool goto_next_stop =
        ShouldGoNextStop(smm, map_index, multi_stops, /* next_stop_index=*/4,
                         {pt3d.x(), pt3d.y()});
    ASSERT_TRUE(goto_next_stop);
  }
}
}  // namespace
}  // namespace qcraft::planner::route
