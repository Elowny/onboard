#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_VALIDATION_TRAJECTORY_VALIDATION_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_VALIDATION_TRAJECTORY_VALIDATION_H_  // NOLINT

#include <vector>  // for vector

#include "onboard/async/thread_pool.h"                   // for ThreadPool
#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNetOutput
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"  // for PartialSpacetimeObjectTrajectory
#include "onboard/planner/object/spacetime_trajectory_manager.h"  // for SpacetimeTrajectoryManager
#include "onboard/planner/planner_semantic_map_manager.h"  // for PlannerSemanticMapManager
#include "onboard/planner/scheduler/scheduler_output.h"  // for SchedulerOutput
#include "onboard/proto/vehicle.pb.h"  // for VehicleGeometryParamsProto

namespace qcraft::planner::ml {

std::vector<PartialSpacetimeObjectTrajectory> FakeConsideredStObjects(
    const SpacetimeTrajectoryManager& obj_mgr);

void CheckTrajectoryValidation(
    const PlannerSemanticMapManager& psmm,
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    bool full_stop, const SchedulerOutput& scheduler_output,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    captain_net::CaptainNetOutput* output, ThreadPool* thread_pool);

}  // namespace qcraft::planner::ml

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_VALIDATION_TRAJECTORY_VALIDATION_H_
