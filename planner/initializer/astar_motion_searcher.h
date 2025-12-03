#ifndef ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_H_
#define ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_H_

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"

namespace qcraft::planner {
// The function searches space-time trajectory with Astar algorithm.
absl::StatusOr<MotionSearchOutput> AstarSearchForRawTrajectory(
    const MotionSearchInput& input, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_ASTAR_MOTION_SEARCHER_H_
