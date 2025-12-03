#include "onboard/planner/router/route_error.h"

#include <utility>

#include "gtest/gtest.h"

namespace qcraft::planner::route {
namespace {
TEST(RouteErrorTest, RouteError) {
  auto path_not_found =
      RoutePathNotFoundError("search route lane path failed.");
  EXPECT_TRUE(!path_not_found.action().empty());
  EXPECT_TRUE(!path_not_found.message().empty());
  EXPECT_EQ(path_not_found.code(),
            RouteErrorCode::StatusCode::kRoutePathNotFound);
  EXPECT_TRUE(!path_not_found.ToString().empty());
  EXPECT_EQ(OkRouteStatus().code(), RouteErrorCode::StatusCode::kOk);
  EXPECT_EQ(RouteCreateLanePathFailedError(" ").code(),
            RouteErrorCode::StatusCode::kRouteCreateLanePathFailedError);
  EXPECT_EQ(RerouteError(" ").code(), RouteErrorCode::StatusCode::kReroute);
  EXPECT_EQ(MapMatchError(" ").code(), RouteErrorCode::StatusCode::kMapMatch);
  RouteErrorCode error_code;
  error_code = std::move(path_not_found);
  EXPECT_EQ(error_code.code(), RouteErrorCode::StatusCode::kRoutePathNotFound);

  // NOA route errors
  EXPECT_EQ(InvalidLocalizationStatusError("localization is invalid.").code(),
            RouteErrorCode::StatusCode::kInvalidLocalizationStatusError);
  EXPECT_EQ(InvalidEgoSectionIdError("").code(),
            RouteErrorCode::StatusCode::kInvalidEgoSectionIdError);
  EXPECT_EQ(EmptyMppSectionsError("").code(),
            RouteErrorCode::StatusCode::kEmptyMppSectionsError);
  EXPECT_EQ(NotMatchSdRouteWithHdMapError("").code(),
            RouteErrorCode::StatusCode::kNotMatchSdRouteWithHdMapError);
  EXPECT_EQ(NotMatchAvPosWithHdMapError("").code(),
            RouteErrorCode::StatusCode::kNotMatchAvPosWithHdMapError);
  EXPECT_EQ(InvalidSdRouteError("").code(),
            RouteErrorCode::StatusCode::kInvalidSdRouteError);
}

}  // namespace
}  // namespace qcraft::planner::route
