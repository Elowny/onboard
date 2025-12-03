#include "onboard/planner/plan/acc/aspice_test/aspice_test_util.h"

#include <utility>

#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {
namespace aspice_test {

PlannerObject BuildPlannerObject(const std::string& name, const Vec2d& pos,
                                 double object_yaw, double obj_v, double obj_a,
                                 double object_length) {
  // Build target traj.
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id(name)
                                  .set_type(OT_VEHICLE)
                                  .set_pos(pos)
                                  .set_timestamp(1.0)
                                  .set_velocity(obj_v)
                                  .set_yaw(object_yaw)
                                  .set_length_width(object_length, 2.0)
                                  .set_box_center(pos)
                                  .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(pos, object_yaw, /*duration=*/10.0, obj_v, obj_a);
  return builder.Build();
}

SpacetimeObjectTrajectory BuildSpacetimeObjectTrajectory(
    const PlannerObject& object) {
  const auto& traj = object.traj(0);
  auto states = SampleTrajectoryStates(traj, object.pose().pos(),
                                       object.contour(), object.bounding_box());
  return SpacetimeObjectTrajectory(object, std::move(states), /*traj_index=*/0,
                                   /*required_lateral_gap=*/0.5);
}

}  // namespace aspice_test
}  // namespace planner
}  // namespace qcraft
