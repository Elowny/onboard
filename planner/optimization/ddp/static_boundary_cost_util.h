#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_STATIC_BOUNDARY_COST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_STATIC_BOUNDARY_COST_UTIL_H_

#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/optimization/ddp/path_time_corridor.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace optimizer {

enum class CurbCostType {
  kStaticBoundary = 1,
  kMsdV1 = 2,
  kMsdV2 = 3,
};

void AddStaticBoundaryCosts(
    int trajectory_steps, std::string_view base_name,
    bool enable_three_point_turn, const TrajectoryPoint& plan_start_point,
    const DrivePassage& drive_passage, const PlannerSemanticMapManager& psmm,
    const PathSlBoundary& path_sl_boundary,
    const PathTimeCorridor& path_time_corridor,
    const std::vector<double>& left_l_boundary_for_nudge,
    const std::vector<double>& right_l_boundary_for_nudge,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    CurbCostType curb_cost_type, std::optional<double>* extra_curb_buffer_opt,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs);

void AddSolidWhiteLineCost(
    int trajectory_steps, std::string_view base_name,
    const std::vector<TrajectoryPoint>& solver_init_traj,
    const ConstraintManager& constraint_manager,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleCircleModelParamsProto&
        trajectory_optimizer_vehicle_model_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_STATIC_BOUNDARY_COST_UTIL_H_
