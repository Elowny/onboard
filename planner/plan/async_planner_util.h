#ifndef ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_UTIL_H_

#include <memory>    // for operator==, shared_ptr
#include <optional>  // for optional
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/planner/common/planner_status.h"  // for PlannerStatus
#include "onboard/planner/plan/async_planner_output.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"  // for EstPlannerDebugProto
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

void UpdateAsyncCounter(int low_freq_cycle, int* counter);

inline bool ShouldRunLowFreqModule(int counter) { return counter == 0; }

inline bool IsPlannerAsync(int async_low_freq_cycle_iterations) {
  return async_low_freq_cycle_iterations != 0;
}

inline bool ShouldRetrieveLowFreqResult(int counter,
                                        int async_low_freq_cycle_iterations) {
  return counter >= async_low_freq_cycle_iterations;
}

inline bool MustRetrieveLowFreqResult(int counter,
                                      int async_low_freq_cycle_iterations) {
  return counter == async_low_freq_cycle_iterations;
}

absl::StatusOr<std::vector<PathPoint>> AlignPathWithPlanStartPoint(
    const std::vector<PathPoint>& path,
    const ApolloTrajectoryPointProto& plan_start_point);

inline bool ShouldRunAccPlannerInAsyncPlanner(const AsyncPlannerState& state) {
  return state.secondary_counter.has_value() &&
         state.latest_multi_task_est_result == nullptr;
}

void StitchStPathTrajectoryWithPastTrajectory(
    const TrajectoryProto& previous_trajectory,
    const std::optional<int>& start_index_on_prev_traj,
    const std::vector<PathPoint>& st_path_points,
    std::vector<PathPoint>* st_path_points_including_past);

// TODO(jiayu): Move to planner main loop internal.
bool IsNonEmptyPlannerResult(const PlannerStatus& status);

void FillHighFreqPlannerDebug(AsyncPlannerOutput* async_output,
                              EstPlannerDebugProto* est_planner_debug);

void UpdateAsyncCounterAfterLowFreqRetrieved(
    AsyncPlannerState* async_planner_state);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_UTIL_H_
