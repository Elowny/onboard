#ifndef ONBOARD_PLANNER_ML_EXPERT_EXPERT_PLANNER_UTIL_H_
#define ONBOARD_PLANNER_ML_EXPERT_EXPERT_PLANNER_UTIL_H_

#include <vector>

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/ml/expert/expert_planner.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {
bool SelectorIntentionSameAsExpert(
    const std::vector<ApolloTrajectoryPointProto>& expert_traj_points,
    const std::vector<PlannerStatus>& status_list,
    const std::vector<EstPlannerOutput>& results,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm);

bool OptimizerHasNoIntentionSameAsExpert(
    const TrajectoryProto& expert_trajectory_proto,
    const std::vector<PlannerStatus>& status_list,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<EstPlannerDebug>& est_debugs,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm);

void OptimizerCheckEachIntentionSameAsExpert(
    const TrajectoryProto& expert_trajectory_proto,
    const std::vector<PlannerStatus>& status_list,
    const std::vector<EstPlannerDebug>& est_debugs,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    std::vector<EstPlannerOutput>* results);

void AppendExpertToResultList(
    const PlannerStatus& expert_status, ExpertPlannerOutput expert_result,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    std::vector<PlannerStatus>* status_list,
    std::vector<EstPlannerOutput>* results);
}  // namespace qcraft::planner
#endif
