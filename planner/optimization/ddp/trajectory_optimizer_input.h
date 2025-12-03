#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_

#include <map>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_state.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct TrajectoryOptimizerInput {
  absl::Span<const ApolloTrajectoryPointProto> trajectory;
  absl::Span<const ApolloTrajectoryPointProto> previous_trajectory;

  // std::nullopt when first time running trajectory optimizer.
  std::optional<TrajectoryOptimizerState> trajectory_optimizer_state;

  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const SpacetimePlannerObjectTrajectories* st_planner_object_traj = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const ConstraintManager* constraint_mgr = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_mgr = nullptr;
  ApolloTrajectoryPointProto plan_start_point;
  absl::Time plan_start_time;
  int plan_id = 0;
  const std::vector<TrajectoryPoint>* captain_trajectory = nullptr;
  LaneChangeStage lc_stage;
  // Params.
  const TrajectoryOptimizerParamsProto* trajectory_optimizer_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const PlannerFunctionsParamsProto* planner_functions_params = nullptr;
  const PlannerVehicleModelParamsProto* vehicle_models_params = nullptr;
  const VehicleGeometryParamsProto* veh_geo_params = nullptr;
  const VehicleDriveParamsProto* veh_drive_params = nullptr;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_INPUT_H_
