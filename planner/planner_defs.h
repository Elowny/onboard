#ifndef ONBOARD_PLANNER_PLANNER_DEFS_H_
#define ONBOARD_PLANNER_PLANNER_DEFS_H_

#include "absl/time/time.h"

#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

// Platform-wise constants.
// TODO(all): Move these constants to param files.
#if defined(__S32G__)
inline constexpr int kTrajectorySteps = 50;           // 5s.
inline constexpr double kPlanningTimeHorizon = 10.0;  // s.
#elif defined(__X9HP__)
inline constexpr int kTrajectorySteps = 80;           // 8s.
inline constexpr double kPlanningTimeHorizon = 13.0;  // s.
#else
inline constexpr int kTrajectorySteps = 100;          // 10s.
inline constexpr double kPlanningTimeHorizon = 16.0;  // s.
#endif

inline constexpr double kTrajectoryTimeStep = 0.1;  // 100ms.
inline constexpr double kTrajectoryTimeHorizon =
    (kTrajectorySteps - 1) * kTrajectoryTimeStep;  // from 0.0s to 9.9s.

inline constexpr int kAccTrajectorySteps = 80;  // 8s.
inline constexpr double kAccTrajectoryTimeHorizon =
    (kAccTrajectorySteps - 1) * kTrajectoryTimeStep;  // from 0.0s to 7.9s.
inline constexpr double kSpacetimePlannerTrajectoryHorizon =
    kTrajectoryTimeStep * kTrajectorySteps;

inline constexpr double kPathSampleInterval = 0.2;  // m.

inline constexpr int kMaxPastPointNum = 50;
inline constexpr int kMaxAccPastPointNum = 5;

inline constexpr double kSpaceTimeVisualizationDefaultTimeScale = 10.0;  // m/s.

inline constexpr double kMinLCSpeed = 5.0 / 3.6;  // m/s.
inline constexpr double kMinLcLaneLength = 20.0;  // m.  Planner 3.0

inline constexpr double kDefaultLaneWidth = 3.5;  // m.
inline constexpr double kDefaultHalfLaneWidth = 0.5 * kDefaultLaneWidth;
inline constexpr double kMaxHalfLaneWidth = 2.7;                  // m.
inline constexpr double kMinLaneWidth = 2.6;                      // m.
inline constexpr double kMinHalfLaneWidth = kMinLaneWidth * 0.5;  // m.

// Desired lateral distance for nudge.
inline constexpr double kMaxLateralOffset = 10.0;         // m.
inline constexpr double kMaxLaneKeepLateralOffset = 0.4;  // m.

inline constexpr double kRouteStationUnitStep = 2.0;  // m.

inline constexpr double kDrivePassageKeepBehindLength = 10.0;  // m.

inline constexpr double kLaneChangeCheckForwardLength = 180.0;   // m.
inline constexpr double kLaneChangeCheckBackwardLength = 100.0;  // m.

inline constexpr double kMaxTravelDistanceBetweenFrames = 20.0;  // m.
inline constexpr double kMaxDistanceForClampingSections = 60.0;  // m.

inline constexpr double kInitializerMinFollowDistance = 3.0;

// Should be shorter than the width of a map patch.
inline constexpr double kPlannerLaneGraphLength = 200.0;  // m.

// Trajectory time length with curvature constraint
inline constexpr double kCurvatureLimitRange = 6.0;

inline constexpr double kAlccReferenceLineRequiredLength = 400.0;     // m.
inline constexpr double kMaplessReferenceLineRequiredLength = 400.0;  // m.
inline constexpr double kAlccPlcPreviewDistance = 20.0;               // m.

inline constexpr double kUTurnCurbGain = 0.3;

inline constexpr double kCurbGain = 1.0;

// NOLINTNEXTLINE(readability-identifier-naming)
inline const char SoftNameString[] = "Soft";
// NOLINTNEXTLINE(readability-identifier-naming)
inline const char HardNameString[] = "Hard";

inline constexpr double kAlternateRouteAllowRoundaboutDist = 2000.0;  // m.

inline constexpr int64_t kInvalidOnlineMapId = -1;

inline constexpr int kAsyncCounterInitVal = -1;

inline constexpr double kOnlineHdMapLoadMinDist = 300.0;  // m.
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_DEFS_H_
