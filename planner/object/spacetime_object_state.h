#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_STATE_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_STATE_H_

#include <vector>

#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft::planner {

// Spacetime object states. We create one state for each point on predicted
// trajectory, except for stationary trajectories.
struct SpacetimeObjectState {
  // If the trajectory is a stationary trajectory, the trajectory will only have
  // one point. All state's `traj_point` points to the same trajectory point.
  // Therefore, the `t` of a stationary trajectory's point should not be used.
  // TODO(lidong): For stationary object state, set `traj_point` to nullptr.
  const prediction::PredictedTrajectoryPoint* traj_point;
  // The box and contour of the object.
  Box2d box;
  Polygon2d contour;
};

// A utility function that can sample spacetime object trajectory states from a
// predicted trajectory. The `init_contour` and `init_box` defines the object's
// geometry when object is at `init_pos`. The initial heading is the same as
// `init_box`'s heading.
std::vector<SpacetimeObjectState> SampleTrajectoryStates(
    const prediction::PredictedTrajectory& pred_traj, const Vec2d& init_pos,
    const Polygon2d& init_contour, const Box2d& init_box);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_OBJECT_STATE_H_
