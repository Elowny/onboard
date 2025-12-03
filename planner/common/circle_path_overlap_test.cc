#include "onboard/planner/common/circle_path_overlap.h"

#include <memory>

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEpsilon = 1.0e-4;

TEST(SegmentFirstOverlapAngleTest, test) {
  const Segment2d segment(Vec2d(0.0, 0.0), Vec2d(1.0, 0.0));
  const Vec2d center(0.0, 2.0);

  EXPECT_NEAR(
      SegmentFirstOverlapAngle(segment, center, /*ccw=*/true,
                               Segment2d(Vec2d(1.0, 3.0), Vec2d(3.0, 3.0))),
      0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(SegmentFirstOverlapAngle(segment, center, /*ccw=*/true,
                                       Box2d(Vec2d(2.0, 4.0), /*heading=*/0.0,
                                             /*length=*/1.0, /*width=*/2.0)),
              0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(
      SegmentFirstOverlapAngle(segment, center, /*ccw=*/true,
                               Polygon2d({Vec2d(1.5, 2.5), Vec2d(2.5, 3.5),
                                          Vec2d(2.5, 4.0), Vec2d(1.5, 4.0)})),
      0.5 * M_PI, kEpsilon);
}

TEST(BoxFirstOverlapAngleTest, test) {
  const Box2d box(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/2.0,
                  /*width=*/2.0);
  const Vec2d center(0.0, 2.0);

  EXPECT_NEAR(
      BoxFirstOverlapAngle(box, center, /*ccw=*/false,
                           Segment2d(Vec2d(-3.0, 3.0), Vec2d(0.0, 3.0))),
      0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(BoxFirstOverlapAngle(box, center, /*ccw=*/false,
                                   Box2d(Vec2d(-2.0, 4.0), /*heading=*/0.0,
                                         /*length=*/1.0, /*width=*/2.0)),
              0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(
      BoxFirstOverlapAngle(box, center, /*ccw=*/false,
                           Polygon2d({Vec2d(-2.0, 3.0), Vec2d(-1.0, 4.0),
                                      Vec2d(-2.0, 5.0), Vec2d(-3.0, 4.0)})),
      0.5 * M_PI, kEpsilon);
}

TEST(PolygonFirstOverlapAngleTest, test) {
  const Polygon2d polygon({Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), Vec2d(0.0, 1.0)});
  const Vec2d center(0.0, 2.0);

  EXPECT_NEAR(
      PolygonFirstOverlapAngle(polygon, center, /*ccw=*/true,
                               Segment2d(Vec2d(-2.5, 1.0), Vec2d(-1.5, 1.0))),
      1.5 * M_PI, kEpsilon);
  EXPECT_NEAR(PolygonFirstOverlapAngle(polygon, center, /*ccw=*/true,
                                       Box2d(Vec2d(-2.0, 0.5), /*heading=*/0.0,
                                             /*length=*/1.0, /*width=*/1.0)),
              1.5 * M_PI, kEpsilon);
  EXPECT_NEAR(
      PolygonFirstOverlapAngle(
          polygon, center, /*ccw=*/true,
          Polygon2d({Vec2d(-1.5, 1.0), Vec2d(-1.0, 1.0), Vec2d(-1.5, 1.5)})),
      1.5 * M_PI, kEpsilon);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
