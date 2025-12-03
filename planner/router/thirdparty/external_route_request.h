#ifndef ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_REQUEST_H_
#define ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_REQUEST_H_
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "nlohmann/json.hpp"

#include "onboard/math/vec.h"
#include "onboard/planner/router/thirdparty/coordinate_transformer.h"
#include "onboard/proto/route.pb.h"
namespace qcraft::planner::route {

struct ExternalRouteResult {
  std::vector<Vec2d> points;
  NaviPreviewProto navi_preview;
};
namespace internal {
ExternalRouteResult ConvertTencentJson(const CoordinateTransformer& cf,
                                       const nlohmann::json& content);
}

absl::StatusOr<ExternalRouteResult> TencentRoute(
    const CoordinateTransformer& cf, const Vec2d& origin,
    const Vec2d& destination, absl::Span<const Vec2d> waypoints,
    std::string_view url =
        "https://qsuite.qcraftai.com/api/lbs/v2/get-driving-direction",
    int road_type = 0);

// Radians
absl::StatusOr<std::vector<Vec2d>> Wgs84ToGcj02ByQops(
    const std::vector<Vec2d>& points,
    std::string_view url =
        "https://qsuite.qcraftai.com/api/lbs/v1/"
        "transform-coordinate-from-gps-to-third-party");

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_THIRDPARTY_EXTERNAL_ROUTE_REQUEST_H_
