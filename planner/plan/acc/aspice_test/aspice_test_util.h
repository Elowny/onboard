#include <string>

#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"

namespace qcraft {
namespace planner {
namespace aspice_test {

PlannerObject BuildPlannerObject(const std::string& name, const Vec2d& pos,
                                 double object_yaw, double obj_v, double obj_a,
                                 double object_length);

SpacetimeObjectTrajectory BuildSpacetimeObjectTrajectory(
    const PlannerObject& object);

}  // namespace aspice_test
}  // namespace planner
}  // namespace qcraft
