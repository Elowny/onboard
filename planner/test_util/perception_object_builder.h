#ifndef ONBOARD_PLANNER_TEST_UTIL_PERCEPTION_OBJECT_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_PERCEPTION_OBJECT_BUILDER_H_

#include "absl/strings/string_view.h"

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {

// This class helps to create a perception object.
class PerceptionObjectBuilder {
 public:
  // Make default object settings.
  PerceptionObjectBuilder();

  PerceptionObjectBuilder& set_id(absl::string_view id);

  PerceptionObjectBuilder& set_type(ObjectType type);

  PerceptionObjectBuilder& set_pos(const Vec2d& pos);

  PerceptionObjectBuilder& set_timestamp(double time);

  PerceptionObjectBuilder& set_box_center(const Vec2d& pos);

  PerceptionObjectBuilder& set_length_width(double length, double width);

  PerceptionObjectBuilder& set_yaw(double yaw);

  PerceptionObjectBuilder& set_yaw_rate(double yaw_rate);

  PerceptionObjectBuilder& set_contour(const Polygon2d& contour);

  PerceptionObjectBuilder& set_velocity(double velocity);

  PerceptionObjectBuilder& set_speed(const Vec2d& speed);

  PerceptionObjectBuilder& set_accel(const Vec2d& accel);

  PerceptionObjectBuilder& set_accel(double accel);

  PerceptionObjectBuilder& set_trajectory(int states_num);

  // Creates the perception object.
  ObjectProto Build();

 private:
  ObjectProto object_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_PERCEPTION_OBJECT_BUILDER_H_
