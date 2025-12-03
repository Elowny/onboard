#ifndef ONBOARD_PLANNER_ML_PLANNER_ML_DEFS_H_
#define ONBOARD_PLANNER_ML_PLANNER_ML_DEFS_H_

namespace qcraft {
namespace planner {
namespace ml {

// onboard av history container settings.
// Note(Jinyun): Set kAvHistoryBufferSize > kAvHistoryBufferLen / pose time
// interval (0.01).
inline constexpr double kAvHistoryBufferSize = 200;
inline constexpr double kAvHistoryBufferLen = 1.5;  // s.

// av and objects's config.
inline constexpr double kHistoryStepNum = 11;
inline constexpr double kHistoryStepLen = 0.1;  // Seconds.

// map config.
inline constexpr double kMaxMapSampleLen = 10.0;  // m.
inline constexpr int kMapSegmentNum = 5;

// trajectory ground truth config.
inline constexpr double kTrajectoryGroundTruthDuration = 11.0;  // s.
inline constexpr double kTrajectoryGroundTruthStep = 0.1;       // s.

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ML_PLANNER_ML_DEFS_H_
