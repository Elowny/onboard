#include "onboard/prediction/util/perception_util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/test_util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace prediction {
namespace {

TEST(AlignPerceptionObjectTime, Works) {
  auto perception_object = planner::PerceptionObjectBuilder()
                               .set_id("obj")
                               .set_pos(Vec2d(1.0, 2.0))
                               .set_timestamp(10.0)
                               .set_length_width(4.0, 2.0)
                               .set_yaw(0.0)
                               .set_yaw_rate(0.1)
                               .set_speed(Vec2d(1.0, 2.0))
                               .set_accel(Vec2d(1.0, -1.0))
                               .Build();
  ObjectProto& aligned_object = perception_object;
  ASSERT_OK(AlignPerceptionObjectTime(10.1, &aligned_object));
  EXPECT_THAT(aligned_object, ProtoPartialApproximatelyMatchesText(R"(
                                      pos { x: 1.105 y: 2.195 }
                                      vel { x: 1.1 y: 1.9 }
                                      accel { x: 1 y: -1 }
                                      timestamp: 10.1
                                      id: "obj"
                                      yaw: 0.01
                                      yaw_rate: 0.1
                                      bounding_box {
                                        x: 1.105
                                        y: 2.195
                                        heading: 0.01
                                        width: 2
                                        length: 4
                                      }
                                      bounding_box_source: FIERY_EYE_NET
                                      parked: false
                                      offroad: false)",
                                                                   1e-3));
  const Box2d box(aligned_object.bounding_box());
  const Polygon2d shifted_polygon =
      Polygon2d::FromPoints(aligned_object.contour(), /*is_convex=*/true)
          .value();
  EXPECT_THAT(shifted_polygon.points(),
              testing::UnorderedElementsAre(
                  Vec2dNear(box.GetCorner(Box2d::FRONT_LEFT), 1e-3),
                  Vec2dNear(box.GetCorner(Box2d::REAR_LEFT), 1e-3),
                  Vec2dNear(box.GetCorner(Box2d::REAR_RIGHT), 1e-3),
                  Vec2dNear(box.GetCorner(Box2d::FRONT_RIGHT), 1e-3)));
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
