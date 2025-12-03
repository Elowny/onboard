#ifndef ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_

#include <vector>

#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/st_close_trajectory.h"

namespace qcraft::planner {
std::vector<StCloseTrajectory> GenerateMovingStCloseTrajectories(
    const SpacetimeObjectTrajectory& st_traj,
    std::vector<std::vector<StCloseTrajectory::StNearestPoint>>
        close_traj_points);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_GENERATOR_H_
