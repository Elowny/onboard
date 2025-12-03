#include "onboard/planner/ml/expert/expert_planner_util.h"

#include "onboard/planner/ml/common/data_filtering_utils.h"
#include "onboard/planner/trajectory_util.h"
namespace qcraft::planner {

bool SelectorIntentionSameAsExpert(
    const std::vector<ApolloTrajectoryPointProto>& expert_traj_points,
    const std::vector<PlannerStatus>& status_list,
    const std::vector<EstPlannerOutput>& results,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm) {
  QCHECK_NE(expert_traj_points.size(), 0);
  for (int plan_idx = 0; plan_idx < status_list.size(); ++plan_idx) {
    // TODO(jingqiao): Consider whether need to filter fallback trajs.
    if (status_list[plan_idx].ok()) {
      if (TrajIntentionSameAsExpert(
              ToTrajectoryPoint(expert_traj_points),
              ToTrajectoryPoint(results[plan_idx].traj_points),
              results[plan_idx].scheduler_output.drive_passage,
              results[plan_idx].scheduler_output.sl_boundary,
              vehicle_geometry_params, psmm)) {
        return true;
      }
    }
  }
  return false;
}

bool OptimizerHasNoIntentionSameAsExpert(
    const TrajectoryProto& expert_trajectory_proto,
    const std::vector<PlannerStatus>& status_list,
    const std::vector<EstPlannerOutput>& results,
    const std::vector<EstPlannerDebug>& est_debugs,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm) {
  std::vector<TrajectoryPoint> traj_points;
  for (const auto& apollo_point : expert_trajectory_proto.trajectory_point()) {
    traj_points.emplace_back(TrajectoryPoint(apollo_point));
  }
  for (int plan_idx = 0; plan_idx < status_list.size(); ++plan_idx) {
    if (status_list[plan_idx].ok() &&
        !results[plan_idx].scheduler_output.is_fallback) {
      std::vector<TrajectoryPoint> candidate_traj_points;
      for (const auto& traj_point :
           est_debugs[plan_idx].optimizer_debug_proto.ddp().final_traj()) {
        candidate_traj_points.push_back(TrajectoryPoint(traj_point));
      }
      if (TrajIntentionSameAsExpert(
              traj_points, candidate_traj_points,
              results[plan_idx].scheduler_output.drive_passage,
              results[plan_idx].scheduler_output.sl_boundary,
              vehicle_geometry_params, psmm)) {
        return false;
      }
    }
  }
  return true;
}

void OptimizerCheckEachIntentionSameAsExpert(
    const TrajectoryProto& expert_trajectory_proto,
    const std::vector<PlannerStatus>& status_list,
    const std::vector<EstPlannerDebug>& est_debugs,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm,
    std::vector<EstPlannerOutput>* results) {
  std::vector<TrajectoryPoint> traj_points;
  for (const auto& apollo_point : expert_trajectory_proto.trajectory_point()) {
    traj_points.emplace_back(apollo_point);
  }
  for (int plan_idx = 0; plan_idx < status_list.size(); ++plan_idx) {
    if ((*results)[plan_idx]
            .candidate_auto_tuning_traj_proto.valid_for_train() &&
        !(*results)[plan_idx].scheduler_output.is_fallback) {
      std::vector<TrajectoryPoint> candidate_traj_points;
      for (const auto& traj_point :
           est_debugs[plan_idx].optimizer_debug_proto.ddp().final_traj()) {
        candidate_traj_points.push_back(TrajectoryPoint(traj_point));
      }
      (*results)[plan_idx].candidate_auto_tuning_traj_proto.set_valid_for_train(
          TrajIntentionSameAsExpert(
              traj_points, candidate_traj_points,
              (*results)[plan_idx].scheduler_output.drive_passage,
              (*results)[plan_idx].scheduler_output.sl_boundary,
              vehicle_geometry_params, psmm));
    }
  }
}

void AppendExpertToResultList(
    const PlannerStatus& expert_status, ExpertPlannerOutput expert_result,
    std::vector<SpacetimeTrajectoryManager>* st_traj_mgr_list,
    std::vector<PlannerStatus>* status_list,
    std::vector<EstPlannerOutput>* results) {
  st_traj_mgr_list->push_back(std::move(expert_result.filtered_traj_mgr));
  status_list->push_back(expert_status);
  results->emplace_back(EstPlannerOutput{
      .scheduler_output = std::move(expert_result.scheduler_output),
      .path = std::move(expert_result.path),
      .traj_points = std::move(expert_result.trajectory_points),
      .st_path_points = std::move(expert_result.st_path_points),
      .decider_state = std::move(expert_result.decider_state),
      .considered_st_objects = std::move(expert_result.considered_st_objects),
      .trajectory_end_info = std::move(expert_result.trajectory_end_info)});
}

}  // namespace qcraft::planner
