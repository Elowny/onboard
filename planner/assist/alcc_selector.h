#ifndef ONBOARD_PLANNER_ASSIST_ALCC_SELECTOR_H_
#define ONBOARD_PLANNER_ASSIST_ALCC_SELECTOR_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/planner/assist/plc_internal_result.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

absl::StatusOr<int> RunAlccSelector(
    const PlannerSemanticMapManager& psmm,
    const VehicleGeometryParamsProto& vehicle_geom,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& results, int preferred_idx,
    PlcInternalResult* plc_result);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ASSIST_ALCC_SELECTOR_H_
