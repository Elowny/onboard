#include "onboard/planner/router/route_portal_util.h"

#include "gtest/gtest.h"
#include "onboard/base/macros.h"

namespace qcraft::planner {
namespace {

TEST(RoutePortalUtilTest, ConvertToDriverRouteTest) {
  std::string json =
      R"({"stops":[{"stop_name":"5","stop_point":{"geo_point":{"latitude":0.5484917434943782,"longitude":2.105400345804816},"global_point":{"latitude":0.5484917434943782,"longitude":2.105400345804816}}},{"stop_name":"6","stop_point":{"geo_point":{"latitude":0.5484662267807141,"longitude":2.105448831051436},"global_point":{"latitude":0.5484662267807141,"longitude":2.105448831051436}}},{"stop_name":"52","stop_point":{"geo_point":{"latitude":0.5484127498924329,"longitude":2.1054590412275602},"global_point":{"latitude":0.5484127498924329,"longitude":2.1054590412275602}}},{"stop_name":"211","stop_point":{"geo_point":{"latitude":0.548379885342618,"longitude":2.105591023025596,"altitude":12.84},"global_point":{"latitude":0.548379885342618,"longitude":2.105591023025596,"altitude":12.84}}},{"stop_name":"12","stop_point":{"geo_point":{"latitude":0.5483008742873802,"longitude":2.105759866177434},"global_point":{"latitude":0.5483008742873802,"longitude":2.105759866177434}}},{"stop_name":"13","stop_point":{"geo_point":{"latitude":0.5482228406165235,"longitude":2.1057282931712655},"global_point":{"latitude":0.5482228406165235,"longitude":2.1057282931712655}}},{"stop_name":"14","stop_point":{"geo_point":{"latitude":0.5483330581587869,"longitude":2.105338857855268},"global_point":{"latitude":0.5483330581587869,"longitude":2.105338857855268}}},{"stop_name":"15","stop_point":{"geo_point":{"latitude":0.5483722233472017,"longitude":2.1053174950252234},"global_point":{"latitude":0.5483722233472017,"longitude":2.1053174950252234}}}],"infinite_loop":true,"skip_past_stops":true,"limited_zone":{"lanes":[],"regions":[]},"override_zone":{"lanes":[],"regions":[]} })";
  const auto drive_route_or = ConvertToDriverRoute(json);
  EXPECT_OK(drive_route_or);
  EXPECT_TRUE(drive_route_or->skip_past_stops());
  const auto routing_request_proto_or =
      ConverterToRoutingRequestProto(*drive_route_or);
  EXPECT_OK(routing_request_proto_or);
  EXPECT_TRUE(routing_request_proto_or->has_multi_stops());
}

}  // namespace
}  // namespace qcraft::planner
