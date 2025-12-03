#include "onboard/planner/router/route_error.h"

#include "absl/strings/str_cat.h"

namespace qcraft::planner::route {

std::string RouteErrorCode::ToString() const {
  return absl::StrCat("code:", code_, ", action:", action_,
                      ", message:", message_);
}

RouteErrorCode OkRouteStatus() {
  return RouteErrorCode(RouteErrorCode::StatusCode::kOk, "", "");
}

RouteErrorCode RoutePathNotFoundError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kRoutePathNotFound,
                        "RoutePathNotFound", message);
}

RouteErrorCode RouteCreateLanePathFailedError(std::string_view message) {
  return RouteErrorCode(
      RouteErrorCode::StatusCode::kRouteCreateLanePathFailedError,
      "CreateLanePathFailed", message);
}

RouteErrorCode RerouteError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kReroute, "reroute",
                        message);
}

RouteErrorCode MapMatchError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kMapMatch, "map_match",
                        message);
}

RouteErrorCode InvalidLocalizationStatusError(std::string_view message) {
  return RouteErrorCode(
      RouteErrorCode::StatusCode::kInvalidLocalizationStatusError,
      "InvalidLocalizationStatusError", message);
}
RouteErrorCode InvalidEgoSectionIdError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kInvalidEgoSectionIdError,
                        "InvalidEgoSectionIdError", message);
}
RouteErrorCode EmptyMppSectionsError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kEmptyMppSectionsError,
                        "EmptyMppSectionsError", message);
}
RouteErrorCode NotMatchSdRouteWithHdMapError(std::string_view message) {
  return RouteErrorCode(
      RouteErrorCode::StatusCode::kNotMatchSdRouteWithHdMapError,
      "NotMatchSdRouteWithHdMapError", message);
}

RouteErrorCode NotMatchAvPosWithHdMapError(std::string_view message) {
  return RouteErrorCode(
      RouteErrorCode::StatusCode::kNotMatchAvPosWithHdMapError,
      "NotMatchSdRouteWithHdMapError", message);
}

RouteErrorCode InvalidSdRouteError(std::string_view message) {
  return RouteErrorCode(RouteErrorCode::StatusCode::kInvalidSdRouteError,
                        "InvalidSdRouteError", message);
}

}  // namespace qcraft::planner::route
