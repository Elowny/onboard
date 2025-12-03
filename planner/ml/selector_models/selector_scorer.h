#ifndef ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORER_H_
#define ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORER_H_

#include <map>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/selector/common_feature.h"
#include "onboard/planner/selector/selector_input.h"

namespace qcraft::planner {

absl::StatusOr<bool> CheckSelectorScoringPrecondition(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& planner_outputs);

absl::StatusOr<std::map<int, float>> ScoringSelectorTrajectory(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& planner_outputs);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_SCORER_H_
