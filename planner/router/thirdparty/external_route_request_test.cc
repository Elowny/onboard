#include "onboard/planner/router/thirdparty/external_route_request.h"

#include <string>

#include "google/protobuf/repeated_ptr_field.h"
#include "nlohmann/json.hpp"

#include "gtest/gtest.h"
namespace qcraft::planner::route {

namespace {

// This test is used to test tencent route works normally, not for UT.
// TEST(ExternalRouteRequest, DISABLED_TencentRouteTest) {
//   // QR00215 points
//   Vec2d origin = {2.1047361, 0.5483020};
//   CoordinateTransformer cf;
//   auto tencent_coord_or = Wgs84ToGcj02ByQops({origin});
//   ASSERT_TRUE(tencent_coord_or.ok());
//   std::vector<Vec2d> waypoints = cf.Wgs84ToGcj02(
//       {{2.1050558, 0.5476658}, {2.1059578, 0.5481989}, {2.1056457,
//       0.5483349}});
//   Vec2d destination = cf.Wgs84ToGcj02({2.1054642, 0.5484130});
//   origin = cf.Wgs84ToGcj02(origin);
//   auto s = TencentRoute(cf, origin, destination, waypoints);
//   ASSERT_TRUE(s.ok());
// }

TEST(ExternalRouteRequest, ConvertTencentJsonTest) {
  Vec2d origin = {2.1047361, 0.5483020};
  CoordinateTransformer cf;
  std::string json = R"(
    {"code":0,"data":{"coordinates":["31.4155340000,120.5919050000","31.4191630000,120.5930970000"],
    "duration":14,"distance":7221,"steps":[{"instruction":"沿广济北路向北行驶418米,左转调头",
    "polylineIdx":[0,1],"roadName":"广济北路","dirDesc":"北","distance":418,
    "actDesc":"左转调头","accessorialDesc":""}]}}

 )";
  auto resp = nlohmann::json::parse(json, /*cb=*/nullptr,
                                    /*allow_exceptions =*/false,
                                    /*ignore_comments */ true);
  auto result = internal::ConvertTencentJson(cf, resp["data"]);
  ASSERT_EQ(result.points.size(), 2);
  ASSERT_GE(result.navi_preview.navi_segments().size(), 1);
}

}  // namespace

}  // namespace qcraft::planner::route
