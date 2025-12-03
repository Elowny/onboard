#include "onboard/planner/object/predicted_motion_filter.h"

#include <algorithm>

#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PredictedMotionFilter, FilterStationary) {
  SetMap("dojo");
  SemanticMapManager map;
  map.LoadWholeMap().Build();

  PlannerObjectBuilder bike_builder;
  bike_builder.set_type(OT_VEHICLE)
      .set_object(PerceptionObjectBuilder()
                      .set_id("bike")
                      .set_type(OT_CYCLIST)
                      .set_timestamp(1.0)
                      .set_velocity(2.0)
                      .set_yaw(0.0)
                      .set_length_width(4.0, 2.0)
                      .set_box_center(Vec2d::Zero())
                      .Build())
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.5)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  PlannerObjectBuilder car_builder;
  car_builder.set_type(OT_VEHICLE)
      .set_object(PerceptionObjectBuilder()
                      .set_id("car")
                      .set_type(OT_VEHICLE)
                      .set_timestamp(1.0)
                      .set_velocity(2.0)
                      .set_yaw(0.0)
                      .set_length_width(4.0, 2.0)
                      .set_box_center(Vec2d::Zero())
                      .Build())
      .get_object_prediction_builder()
      ->add_predicted_trajectory()
      ->set_probability(0.7)
      .set_straight_line(Vec2d::Zero(), Vec2d(10.0, 0.0),
                         /*init_v=*/2.0, /*last_v=*/10.0);

  const auto bike = bike_builder.Build();
  const auto car = car_builder.Build();

  ObjectVector<PlannerObject> objects;
  objects.push_back(bike);
  objects.push_back(car);
  const PredictedMotionFilter filter(objects);

  EXPECT_EQ(filter.Filter(car, car.traj(0)), FilterReason::NONE);
  EXPECT_EQ(filter.Filter(bike, bike.traj(0)),
            FilterReason::TRAJECTORY_COLLIDES_WITH_CONFIDENT_OTHERS);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
