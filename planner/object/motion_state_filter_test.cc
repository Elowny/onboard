#include "onboard/planner/object/motion_state_filter.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
namespace {

PoseProto MakePose(const Vec2d& pos, double yaw, double speed) {
  PoseProto proto;
  proto.set_yaw(yaw);
  proto.mutable_pos_smooth()->set_x(pos.x());
  proto.mutable_pos_smooth()->set_y(pos.y());
  proto.set_speed(speed);
  const Vec2d vel = Vec2d::FastUnitFromAngle(yaw) * speed;
  proto.mutable_vel_smooth()->set_x(vel.x());
  proto.mutable_vel_smooth()->set_y(vel.y());
  return proto;
}

TEST(MotionStateFilter, FilterStationary) {
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_timestamp(1.0)
                                  .set_velocity(2.0)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d::Zero())
                                  .Build();
  PlannerObjectBuilder builder;

  builder.set_type(OT_VEHICLE)
      .set_object(perception_obj)
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  builder.get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.3)
      .set_stationary_traj(Vec2d::Zero(), 0.0);

  const PlannerObject object = builder.Build();
  ASSERT_EQ(object.num_trajs(), 2);

  const auto vehicle_geom = DefaultVehicleGeometry();

  {
    // Has Overlap.
    const auto pose = MakePose(Vec2d(0.0, 0.0), 0.0, 0.0);
    MotionStateFilter filter(pose, vehicle_geom);
    EXPECT_EQ(filter.Filter(object, object.traj(0)), FilterReason::NONE);
    EXPECT_EQ(filter.Filter(object, object.traj(1)), FilterReason::NONE);
  }

  {
    // AV is in object's moving direction.
    const auto pose = MakePose(Vec2d(10.0, 0.0), 0.0, 0.0);
    MotionStateFilter filter(pose, vehicle_geom);
    EXPECT_EQ(filter.Filter(object, object.traj(0)), FilterReason::NONE);
    EXPECT_EQ(filter.Filter(object, object.traj(1)),
              FilterReason::STATIONARY_OBJECT_BEHIND_AV);
  }

  {
    // AV is in object's opposite moving direction.
    const auto pose = MakePose(Vec2d(-10.0, 0.0), -M_PI, 0.0);
    MotionStateFilter filter(pose, vehicle_geom);
    EXPECT_EQ(filter.Filter(object, object.traj(0)),
              FilterReason::OBJECT_BEHIND_MOVING_AWAY_FROM_AV);
    EXPECT_EQ(filter.Filter(object, object.traj(1)),
              FilterReason::STATIONARY_OBJECT_BEHIND_AV);
  }

  {
    // AV is moving toward object.
    const auto pose = MakePose(Vec2d(-10.0, 0.0), 0.0, 0.0);
    MotionStateFilter filter(pose, vehicle_geom);
    EXPECT_EQ(filter.Filter(object, object.traj(0)), FilterReason::NONE);
    EXPECT_EQ(filter.Filter(object, object.traj(1)), FilterReason::NONE);
  }
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
