#ifndef ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_FEATURE_EXTRACTOR_H_  // NOLINT
#define ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_FEATURE_EXTRACTOR_H_  // NOLINT

#include <vector>

#include "absl/status/status.h"

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/selector/common_feature.h"
#include "onboard/planner/selector/proto/selector_debug.pb.h"
#include "onboard/planner/selector/selector_defs.h"
#include "onboard/planner/selector/selector_input.h"

namespace qcraft::planner {

absl::Status DumpSelectorEvaluations(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const CostFeatures& cost_features,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& planner_outputs,
    SelectorDebugProto* selector_debug);

}  // namespace qcraft::planner

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_SELECTOR_MODELS_SELECTOR_FEATURE_EXTRACTOR_H_
