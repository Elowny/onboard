#include "onboard/planner/router/thirdparty/external_route_util.h"

#include <algorithm>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/planner/router/thirdparty/coordinate_transformer.h"
#include "onboard/utils/status_macros.h"
namespace qcraft::planner::route {

namespace {
// Raw GPS in radians.
absl::StatusOr<ExternalRouteResult> GetTencentRoute(
    const std::vector<Vec2d>& points) {
  if (points.size() < 2) {
    return absl::InvalidArgumentError("Shoule be at least two points.");
  }

  CoordinateTransformer cf;
  ASSIGN_OR_RETURN(const auto tencent_coords,
                   planner::route::Wgs84ToGcj02ByQops(points));
  auto route_result_or = planner::route::TencentRoute(
      cf, tencent_coords[0], tencent_coords[tencent_coords.size() - 1],
      absl::MakeSpan(&tencent_coords[1], tencent_coords.size() - 2));

  return route_result_or;
}
}  // namespace

// All coordinates are raw GPS. NOT tencent.
absl::StatusOr<ExternalRouteResult> GetTencentRouteFromDestinations(
    const std::optional<Vec2d>& pos,
    const google::protobuf::RepeatedPtrField<RoutingDestinationProto>&
        destinations) {
  if (destinations.empty()) {
    return absl::InvalidArgumentError("destinations cannot be empty.");
  }
  std::vector<Vec2d> points;
  points.reserve(destinations.size() + 1);
  if (pos.has_value()) {
    points.push_back(*pos);
  }
  for (const auto& dest : destinations) {
    points.push_back(
        {dest.global_point().longitude(), dest.global_point().latitude()});
  }
  return GetTencentRoute(points);
}

absl::StatusOr<ExternalRouteResult> GetTencentRouteFromMultiStops(
    const std::optional<Vec2d>& pos,
    const MultipleStopsRequestProto& multi_stops) {
  const auto& stops = multi_stops.stops();
  if (stops.empty()) {
    return absl::InvalidArgumentError("destinations cannot be empty.");
  }
  std::vector<Vec2d> points;
  if (pos.has_value()) {
    points.push_back(*pos);
  }
  for (const auto& stop : stops) {
    for (const auto& via : stop.via_points()) {
      points.push_back(
          {via.global_point().longitude(), via.global_point().latitude()});
      points.push_back({stop.stop_point().global_point().longitude(),
                        stop.stop_point().global_point().latitude()});
    }
  }
  return GetTencentRoute(points);
}

}  // namespace qcraft::planner::route
