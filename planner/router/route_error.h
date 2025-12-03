#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_ERROR_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_ERROR_H_

#include <string>
#include <string_view>

namespace qcraft::planner::route {

class RouteErrorCode {
 public:
  enum class StatusCode : int {
    kOk = 0,
    kMapMatch = 10,
    kRoutePathNotFound = 20,
    kRouteCreateLanePathFailedError = 21,
    kReroute = 30,

    // NOA errors
    kInvalidLocalizationStatusError = 40,
    kInvalidEgoSectionIdError = 41,
    kEmptyMppSectionsError = 42,
    kNotMatchSdRouteWithHdMapError = 43,
    kNotMatchAvPosWithHdMapError = 44,
    // Sd route erros
    kInvalidSdRouteError = 50,
  };

 public:
  RouteErrorCode() = default;
  RouteErrorCode(StatusCode code, std::string_view action,
                 std::string_view message)
      : code_(code), action_(action), message_(message) {}

  RouteErrorCode(const RouteErrorCode&) = default;
  RouteErrorCode& operator=(const RouteErrorCode& x) = default;
  RouteErrorCode(RouteErrorCode&&) noexcept = default;
  RouteErrorCode& operator=(RouteErrorCode&&) = default;

  ~RouteErrorCode() = default;

  StatusCode code() const { return code_; }

  std::string_view action() const { return action_; }

  std::string_view message() const { return message_; }

  std::string ToString() const;

 private:
  StatusCode code_;
  std::string action_;
  std::string message_;
};

RouteErrorCode OkRouteStatus();
RouteErrorCode RoutePathNotFoundError(std::string_view message);
RouteErrorCode RouteCreateLanePathFailedError(std::string_view message);
RouteErrorCode RerouteError(std::string_view message);
RouteErrorCode MapMatchError(std::string_view message);

// NOA route errors.
RouteErrorCode InvalidLocalizationStatusError(std::string_view message);
RouteErrorCode InvalidEgoSectionIdError(std::string_view message);
RouteErrorCode EmptyMppSectionsError(std::string_view message);
RouteErrorCode NotMatchSdRouteWithHdMapError(std::string_view message);
RouteErrorCode NotMatchAvPosWithHdMapError(std::string_view message);
RouteErrorCode InvalidSdRouteError(std::string_view message);

}  // namespace qcraft::planner::route

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_ERROR_H_
