#include <string>

#include "absl/status/statusor.h"
#include "common/proto/drive_mission.pb.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"

DECLARE_string(multi_stops_route);

namespace qcraft {
namespace planner {
namespace {

TEST(ConvertRouteFlagsToRoutingRequestProto, MultipleStopsRouteConverterTest) {
  FLAGS_multi_stops_route =
      "stops {via_points {global_point {longitude:2.105410059565989 latitude: "
      "0.54849034313059564 altitude: 12.979999542236328}}via_points "
      "{global_point {longitude:2.105980736113481  "
      "latitude:0.54830379221477754 altitude: 13.9399995803833 }}stop_name: "
      "\'6\'}infinite_loop: false";
  absl::StatusOr<RoutingRequestProto> result =
      ConvertAllRouteFlagsToRoutingRequestProto();
  if (result.ok()) {
    RoutingRequestProto routing_request = result.value();
    VLOG(3) << routing_request.DebugString();
  }
  FLAGS_multi_stops_route.clear();
  EXPECT_TRUE(result.ok());
}

TEST(ConvertRouteFlagsToRoutingRequestProto, RouteStrConverterTest) {
  FLAGS_route_str = "shennan-uturn,shennan-west,weizhi,baishi";
  absl::StatusOr<RoutingRequestProto> result =
      ConvertAllRouteFlagsToRoutingRequestProto();
  if (result.ok()) {
    RoutingRequestProto routing_request = result.value();
    VLOG(3) << routing_request.DebugString();
  }
  FLAGS_route_str.clear();
  EXPECT_TRUE(result.ok());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
