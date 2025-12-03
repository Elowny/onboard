#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_CAPTAIN_NET_GENERATOR_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_CAPTAIN_NET_GENERATOR_H_  // NOLINT

#include <vector>

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNetOutput
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner::ml {

PlannerStatus GenerateCaptainNetTrajectory(
    const PlannerSemanticMapManager& psmm,
    const std::vector<SchedulerOutput>& multi_tasks,
    const ml::ContextFeature& context_feature,
    const PlanStartPointInfo& start_point_info,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgrs,
    const ModelPool* planner_model_pool,
    std::vector<captain_net::CaptainNetOutput>* captain_net_results_ptr,
    std::vector<PlannerStatus>* status_list_ptr,
    std::vector<EstPlannerDebug>* est_debugs, ThreadPool* thread_pool);

}  // namespace qcraft::planner::ml

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_CAPTAIN_NET_GENERATOR_H_
