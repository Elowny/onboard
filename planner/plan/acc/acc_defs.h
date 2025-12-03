#ifndef ONBOARD_PLANNER_PLAN_ACC_DEFS_H_
#define ONBOARD_PLANNER_PLAN_ACC_DEFS_H_
namespace qcraft {
namespace planner {
// Corridor.
inline constexpr double kAccMaxLaneWidth = 5.4;  // m.
// With respect to kPathSampleInterval.
inline constexpr double kAccCorridorSampleInterval = 2.0;        // m.
inline constexpr double kCorridorExtendLengthTimeHorizon = 8.0;  // s.
// Speed.
inline constexpr double kDefaultAccSpeedLimitMps = 150.0 / 3.6;  // m/s
inline constexpr double kPedestrianCutinTimeThreshold =
    4.0;  // Pedestrian ignore threshold for future cut-in.
inline constexpr double kCutinTimeThreshold =
    2.5;  // Ignore threshold for future cut-in.
inline constexpr double kMaxOncomingRatio =
    1.0 / 3.0;  // Allowed oncoming movement ratio.
inline constexpr double kMinOncomingS = 4.0;                        // m.
inline constexpr double kCrowdedSceneFollowDistanceDecrease = 0.5;  // m/
// Target.
enum class OnPathType {
  OPT_OFF_BOUND = 0,     // NOLINT
  OPT_NEAR_BOUND = 1,    // NOLINT
  OPT_BOUND = 2,         // NOLINT
  OPT_TARGET_BOUND = 3,  // NOLINT
};
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ACC_DEFS_H_
