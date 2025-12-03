#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_UTIL_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_UTIL_H_
#include <vector>

#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace planner {

std::vector<PartialSpacetimeObjectTrajectory>
CollectSpeedConsiderObjectsPartialStTrajectory(
    const std::vector<EstPlannerOutput>& est_outputs,
    const std::vector<PartialSpacetimeObjectTrajectory>&
        fallback_considered_st_objects);

ObjectsPredictionProto CollectSpeedConsideredObjectsPrediction(
    const PlannerObjectManager& obj_mgr,
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    bool planner_export_all_prediction_to_speed_considered);
}  // namespace planner
}  // namespace qcraft
#endif
