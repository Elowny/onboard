#ifndef ONBOARD_ML_PLANNER_ML_CRUISE_TASK_INTPUT_H_
#define ONBOARD_ML_PLANNER_ML_CRUISE_TASK_INTPUT_H_

#include <memory>

#include "onboard/ml_planner/ml_planner_input.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
namespace qcraft::mlplanner {

struct MLCruiseTaskInput {
  const MLPlannerInput& ml_planner_input;

  std::shared_ptr<const planner::SpacetimeTrajectoryManager> st_traj_mgr =
      nullptr;
  std::shared_ptr<const planner::PlannerObjectManager> object_manager = nullptr;
};
}  // namespace qcraft::mlplanner

#endif  // ONBOARD_ML_PLANNER_ML_CRUISE_TASK_INTPUT_H_
