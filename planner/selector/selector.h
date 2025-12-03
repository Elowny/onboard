#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/planner/selector/selector_input.h"
#include "onboard/planner/selector/selector_output.h"
#include "onboard/planner/selector/selector_state.h"

namespace qcraft {
namespace planner {

absl::StatusOr<SelectorOutput> SelectTrajectory(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& results,
    SelectorDebugProto* selector_debug, SelectorState* selector_state);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_H_
