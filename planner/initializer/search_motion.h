#ifndef ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_
#define ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/charts.pb.h"

namespace qcraft::planner {

absl::StatusOr<MotionSearchOutput> SearchMotion(
    const MotionSearchInput& motion_input, ThreadPool* thread_pool);

// TODO(lidong): Refactor this function to just create initializer input.
absl::StatusOr<InitializerOutput> RunInitializer(
    const InitializerInput& initializer_input,
    absl::flat_hash_set<std::string>* unsafe_object_ids,
    SchedulerOutput* scheduler_output, DeciderOutput* decider_output,
    InitializerDebugProto* debug_proto,
    LaneChangeSafetyDebugProto* lane_change_safety_debug_proto,
    vis::vantage::ChartDataBundleProto* charts, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_SEARCH_MOTION_H_
