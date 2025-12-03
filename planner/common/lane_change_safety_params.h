#ifndef ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_PARAMS_H_
#define ONBOARD_PLANNER_COMMON_LANE_CHANGE_SAFETY_PARAMS_H_

namespace qcraft::planner {

inline constexpr double kEnterTargetLateralThreshold = 1.0;        // m.
inline constexpr double kMaxAllowedDecelForObject = 2.0;           // m/s^2
inline constexpr double kMaxAllowedDecelForSlowLargeObject = 1.0;  // m/s^2
inline constexpr double kMaxAllowedDecelForFastLargeObject = 1.5;  // m/s^2
inline constexpr double kMaxAllowedDecelForEgo = 1.5;              // m/s^2
inline constexpr double kMinLonBufferToFront = 2.0;                // m.

// The following params could be tuned.
inline constexpr double kFollowerStandardResponseTime = 0.8;  // s
inline constexpr double kEgoResponseTime = 0.5;               // s
inline constexpr double kEgoFollowTimeBuffer = 1.0;           // s.
inline constexpr double kEgoLeadTimeBuffer = 0.8;             // s.

}  // namespace qcraft::planner
#endif
