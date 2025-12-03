#ifndef ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"

namespace qcraft::planner {
// The function searches space-time trajectory with dynamic programming. In
// order to accelerate computing, time and speed are discretized to several
// intervals, similar to the spirit of Hybrid A-Star. This function is only
// tested for curvy geometry graph. Do not use it for now for straight
// geometry graph.
absl::StatusOr<MotionSearchOutput> DpSearchForRawTrajectory(
    const MotionSearchInput& input, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_DP_MOTION_SEARCHER_H_
