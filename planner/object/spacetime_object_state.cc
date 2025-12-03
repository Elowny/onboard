#include "onboard/planner/object/spacetime_object_state.h"

#include <algorithm>

#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace planner {

std::vector<SpacetimeObjectState> SampleTrajectoryStates(
    const prediction::PredictedTrajectory& pred_traj, const Vec2d& init_pos,
    const Polygon2d& init_contour, const Box2d& init_box) {
  const auto& traj_points = pred_traj.points();
  if (traj_points.empty()) return {};
  const int num_traj_points = traj_points.size();

  std::vector<SpacetimeObjectState> states;
  states.reserve(num_traj_points);

  if (prediction::IsStationaryTrajectory(pred_traj)) {
    // Spare the spatial transformations below.
    for (int i = 0; i < num_traj_points; ++i) {
      states.push_back(SpacetimeObjectState{.traj_point = &traj_points[i],
                                            .box = init_box,
                                            .contour = init_contour});
    }
    return states;
  }

  // Note that besides transforming the object contour polygon, we also need to
  // transform the object's bounding box center as box's center may not be at
  // the object's pos.
  const Vec2d box_pos_shift = init_box.center() - init_pos;
  for (int i = 0; i < num_traj_points; ++i) {
    SpacetimeObjectState& obj_state = states.emplace_back();
    const auto& pt = traj_points[i];
    obj_state.traj_point = &pt;
    const Vec2d rotation =
        Vec2d::FastUnitFromAngle(pt.theta() - init_box.heading());
    obj_state.contour = init_contour.Transform(
        init_pos, rotation.x(), rotation.y(), pt.pos() - init_pos);

    obj_state.box = Box2d(pt.pos() + box_pos_shift, pt.theta(),
                          init_box.length(), init_box.width());
  }

  return states;
}

}  // namespace planner
}  // namespace qcraft
