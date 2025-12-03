#include "onboard/planner/common/geometry_path_util.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kEps = 1e-3;

inline bool HasOverlap(const Box2d& box, const Polygon2d& object) {
  return object.HasOverlap(box);
}

template <typename T>
inline bool HasOverlap(const Box2d& box, const T& object) {
  return box.HasOverlap(object);
}

template <typename T>
bool CheckStraightPathFirstOverlapDistance(const Box2d& ego_box,
                                           const T& object,
                                           double first_overlap_distance,
                                           bool forward) {
  if (first_overlap_distance == 0.0) {
    return HasOverlap(ego_box, object);
  }
  const double dir_sgn = forward ? 1.0 : -1.0;
  Vec2d shift_unit_vec = Vec2d::FastUnitFromAngle(ego_box.heading()) * dir_sgn;
  const Box2d ego_box1 =
      ego_box.Transform(shift_unit_vec * (first_overlap_distance - kEps));
  const Box2d ego_box2 =
      ego_box.Transform(shift_unit_vec * (first_overlap_distance + kEps));
  return !HasOverlap(ego_box1, object) && HasOverlap(ego_box2, object);
}

template <typename T>
bool CheckCurvePathFirstOverlapDistance(const Box2d& ego_box, const T& object,
                                        const Vec2d& pos,
                                        double first_overlap_distance,
                                        double kappa, bool forward) {
  if (first_overlap_distance == 0.0) {
    return HasOverlap(ego_box, object);
  }
  EXPECT_GT(std::abs(kappa), kEps);
  const double radius = 1.0 / kappa;
  const double dir_sgn = forward ? 1.0 : -1.0;
  const double rear_axis_to_center = pos.DistanceTo(ego_box.center());

  const double distance1 = first_overlap_distance - kEps;
  const double distance2 = first_overlap_distance + kEps;
  const double dtheta_half1 = WrapAngle(distance1 * kappa) * 0.5 * dir_sgn;
  const double dtheta_half2 = WrapAngle(distance2 * kappa) * 0.5 * dir_sgn;
  const Vec2d shift_vec1 =
      radius * std::sin(dtheta_half1) * 2.0 *
          Vec2d::FastUnitFromAngle(ego_box.heading() + dtheta_half1) -
      Vec2d::FastUnitFromAngle(ego_box.heading()) * rear_axis_to_center +
      Vec2d::FastUnitFromAngle(ego_box.heading() + 2.0 * dtheta_half1) *
          rear_axis_to_center;
  const Vec2d shift_vec2 =
      radius * std::sin(dtheta_half2) * 2.0 *
          Vec2d::FastUnitFromAngle(ego_box.heading() + dtheta_half2) -
      Vec2d::FastUnitFromAngle(ego_box.heading()) * rear_axis_to_center +
      Vec2d::FastUnitFromAngle(ego_box.heading() + 2.0 * dtheta_half2) *
          rear_axis_to_center;

  const Box2d ego_box1 = ego_box.Transform(shift_vec1, dtheta_half1 * 2.0);
  const Box2d ego_box2 = ego_box.Transform(shift_vec2, dtheta_half2 * 2.0);
  return !HasOverlap(ego_box1, object) && HasOverlap(ego_box2, object);
}

template <typename T>
bool CheckStraightPathFirstOverlapDistance(const Polygon2d& ego_polygon,
                                           const T& object,
                                           const Vec2d& tangent,
                                           double first_overlap_distance,
                                           bool forward) {
  if (first_overlap_distance == 0.0) {
    return ego_polygon.HasOverlap(object);
  }
  const double dir_sgn = forward ? 1.0 : -1.0;
  Vec2d shift_unit_vec = tangent * dir_sgn;
  const Polygon2d ego_polygon1 =
      ego_polygon.Shift(shift_unit_vec * (first_overlap_distance - kEps));
  const Polygon2d ego_polygon2 =
      ego_polygon.Shift(shift_unit_vec * (first_overlap_distance + kEps));
  return !ego_polygon1.HasOverlap(object) && ego_polygon2.HasOverlap(object);
}

template <typename T>
bool CheckCurvePathFirstOverlapDistance(const Polygon2d& ego_polygon,
                                        const T& object, const Vec2d& pos,
                                        const Vec2d& tangent,
                                        double first_overlap_distance,
                                        double kappa, bool forward) {
  if (first_overlap_distance == 0.0) {
    return ego_polygon.HasOverlap(object);
  }
  EXPECT_GT(std::abs(kappa), kEps);
  const double radius = 1.0 / kappa;
  const Vec2d center = pos + radius * tangent.Perp();
  const Vec2d translation{0.0, 0.0};
  const double dir_sgn = forward ? 1.0 : -1.0;

  const double distance1 = first_overlap_distance - kEps;
  const double distance2 = first_overlap_distance + kEps;
  const double dtheta1 = WrapAngle(distance1 * kappa) * dir_sgn;
  const double dtheta2 = WrapAngle(distance2 * kappa) * dir_sgn;

  const Polygon2d ego_polygon1 = ego_polygon.Transform(
      center, std::cos(dtheta1), std::sin(dtheta1), translation);
  const Polygon2d ego_polygon2 = ego_polygon.Transform(
      center, std::cos(dtheta2), std::sin(dtheta2), translation);
  return !ego_polygon1.HasOverlap(object) && ego_polygon2.HasOverlap(object);
}

}  // namespace

TEST(GeometryPathUtilTest, ComputeFirstOverlapDistanceWithSegment) {
  const Segment2d segment1(Vec2d(-2.1, 8.7), Vec2d(3.2, 12.3));
  const Segment2d segment2(Vec2d(-1.9, -7.2), Vec2d(2.8, -9.1));
  const Segment2d segment3(Vec2d(-2.1, 9.7), Vec2d(-0.1, 11.3));
  const Segment2d segment4(Vec2d(2.3, 7.7), Vec2d(4.1, 8.1));
  const Vec2d pos(1.0, 1.0);
  const Vec2d center(1.0, 2.0);
  const Box2d ego_box(center, /*heading=*/M_PI_2, /*length=*/4.0,
                      /*width=*/2.0);

  // Going Straight.
  auto res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                                    /*forward=*/true,
                                                    /*kappa=*/0.0, segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_box, segment1, *res,
                                                    /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment1);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               segment2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_box, segment2, *res,
                                                    /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               segment3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment3);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               segment4);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment4);
  EXPECT_FALSE(res.has_value());

  // Turning left.
  const double kappa = 0.05;  // 1/m.
  res =
      ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                             /*forward=*/true, kappa, segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment1, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment1, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                             /*forward=*/true, kappa, segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment2, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment2, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                             /*forward=*/true, kappa, segment3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment3, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               segment3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment3, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                             /*forward=*/true, kappa, segment4);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               segment4);
  EXPECT_FALSE(res.has_value());

  // Turning right.
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment1, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment1, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment2, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment2, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               segment3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               segment3);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               segment4);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment4, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               segment4);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, segment4, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));
}

TEST(GeometryPathUtilTest, ComputeFirstOverlapDistanceWithBox) {
  const Box2d box1(Vec2d(0.9, 7.1), /*heading=*/1.0, /*length=*/4.0,
                   /*width=*/2.0);
  const Box2d box2(Vec2d(-1.9, 7.2), /*heading=*/M_PI_2, /*length=*/4.0,
                   /*width=*/2.0);
  const Box2d box3(Vec2d(3.1, 9.7), /*heading=*/M_PI_2, /*length=*/4.0,
                   /*width=*/2.0);
  const Vec2d pos(1.0, 1.0);
  const Vec2d center(1.0, 2.0);
  const Box2d ego_box(center, /*heading=*/M_PI_2, /*length=*/4.0,
                      /*width=*/2.0);

  // Going Straight.
  auto res =
      ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                         /*forward=*/true, /*kappa=*/0.0, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_box, box1, *res,
                                                    /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, /*kappa=*/0.0,
                                           box1);
  EXPECT_FALSE(res.has_value());

  res =
      ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                         /*forward=*/true, /*kappa=*/0.0, box2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, /*kappa=*/0.0,
                                           box2);
  EXPECT_FALSE(res.has_value());

  res =
      ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                         /*forward=*/true, /*kappa=*/0.0, box3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, /*kappa=*/0.0,
                                           box3);
  EXPECT_FALSE(res.has_value());

  // Turning left.
  const double kappa = 0.05;  // 1/m.
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/true, kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box1, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box1, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/true, kappa, box2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box2, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, kappa, box2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box2, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/true, kappa, box3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, kappa, box3);
  EXPECT_FALSE(res.has_value());

  // Turning right.
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/true, -kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box1, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, -kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box1, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/true, -kappa, box2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, -kappa, box2);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/true, -kappa, box3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box3, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, ego_box,
                                           /*forward=*/false, -kappa, box3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, box3, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));
}

TEST(GeometryPathUtilTest, ComputeFirstOverlapDistanceWithPolygon) {
  const Polygon2d polygon1(Box2d(Vec2d(0.9, 7.1), /*heading=*/1.0,
                                 /*length=*/4.0,
                                 /*width=*/2.0));
  const Polygon2d polygon2(Box2d(Vec2d(-1.9, 7.2), /*heading=*/M_PI_2,
                                 /*length=*/4.0,
                                 /*width=*/2.0));
  const Polygon2d polygon3(Box2d(Vec2d(3.1, 9.7), /*heading=*/M_PI_2,
                                 /*length=*/4.0,
                                 /*width=*/2.0));
  const Vec2d pos(1.0, 1.0);
  const Vec2d center(1.0, 2.0);
  const Box2d ego_box(center, /*heading=*/M_PI_2, /*length=*/4.0,
                      /*width=*/2.0);

  // Going Straight.
  auto res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                                    /*forward=*/true,
                                                    /*kappa=*/0.0, polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_box, polygon1, *res,
                                                    /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               polygon1);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               polygon2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               polygon2);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               polygon3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               polygon3);
  EXPECT_FALSE(res.has_value());

  // Turning left.
  const double kappa = 0.05;  // 1/m.
  res =
      ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                             /*forward=*/true, kappa, polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon1, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon1, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                             /*forward=*/true, kappa, polygon2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon2, pos, *res,
                                                 kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               polygon2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon2, pos, *res,
                                                 kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                             /*forward=*/true, kappa, polygon3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, kappa,
                                               polygon3);
  EXPECT_FALSE(res.has_value());

  // Turning right.
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon1, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon1, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               polygon2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               polygon2);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/true, -kappa,
                                               polygon3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon3, pos, *res,
                                                 -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, ego_box,
                                               /*forward=*/false, -kappa,
                                               polygon3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_box, polygon3, pos, *res,
                                                 -kappa,
                                                 /*forward=*/false));
}

TEST(GeometryPathUtilTest, PolygonComputeFirstOverlapDistanceWithSegment) {
  const Segment2d segment1(Vec2d(-2.1, 8.7), Vec2d(3.2, 12.3));
  const Segment2d segment2(Vec2d(-1.9, -7.2), Vec2d(2.8, -9.1));
  const Segment2d segment3(Vec2d(-2.1, 9.7), Vec2d(-0.1, 11.3));
  const Segment2d segment4(Vec2d(2.3, 7.7), Vec2d(4.1, 8.1));
  const Vec2d pos(1.0, 1.0);
  const Vec2d center(1.0, 2.0);
  const Vec2d tangent(0.0, 1.0);
  const Box2d ego_box(center, /*heading=*/M_PI_2, /*length=*/4.0,
                      /*width=*/2.0);
  const Polygon2d ego_polygon(ego_box);

  // Going Straight.
  auto res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                                    /*forward=*/true,
                                                    /*kappa=*/0.0, segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_polygon, segment1,
                                                    tangent, *res,
                                                    /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment1);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               segment2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_polygon, segment2,
                                                    tangent, *res,
                                                    /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               segment3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment3);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               segment4);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               segment4);
  EXPECT_FALSE(res.has_value());

  // Turning left.
  const double kappa = 0.05;  // 1/m.
  res =
      ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment1, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment1, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment2, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment2, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, segment3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment3, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               segment3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment3, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, segment4);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               segment4);
  EXPECT_FALSE(res.has_value());

  // Turning right.
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment1, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               segment1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment1, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment2, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               segment2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment2, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               segment3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               segment3);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               segment4);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment4, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithSegment(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               segment4);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, segment4, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));
}

TEST(GeometryPathUtilTest, PolygonComputeFirstOverlapDistanceWithBox) {
  const Box2d box1(Vec2d(0.9, 7.1), /*heading=*/1.0, /*length=*/4.0,
                   /*width=*/2.0);
  const Box2d box2(Vec2d(-1.9, 7.2), /*heading=*/M_PI_2, /*length=*/4.0,
                   /*width=*/2.0);
  const Box2d box3(Vec2d(3.1, 9.7), /*heading=*/M_PI_2, /*length=*/4.0,
                   /*width=*/2.0);
  const Vec2d pos(1.0, 1.0);
  const Vec2d center(1.0, 2.0);
  const Vec2d tangent(0.0, 1.0);
  const Box2d ego_box(center, /*heading=*/M_PI_2, /*length=*/4.0,
                      /*width=*/2.0);
  const Polygon2d ego_polygon(ego_box);

  // Going Straight.
  auto res =
      ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                         /*forward=*/true, /*kappa=*/0.0, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_polygon, box1, tangent,
                                                    *res,
                                                    /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, /*kappa=*/0.0,
                                           box1);
  EXPECT_FALSE(res.has_value());

  res =
      ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                         /*forward=*/true, /*kappa=*/0.0, box2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, /*kappa=*/0.0,
                                           box2);
  EXPECT_FALSE(res.has_value());

  res =
      ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                         /*forward=*/true, /*kappa=*/0.0, box3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, /*kappa=*/0.0,
                                           box3);
  EXPECT_FALSE(res.has_value());

  // Turning left.
  const double kappa = 0.05;  // 1/m.
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/true, kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box1, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box1, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/true, kappa, box2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box2, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, kappa, box2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box2, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/true, kappa, box3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, kappa, box3);
  EXPECT_FALSE(res.has_value());

  // Turning right.
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/true, -kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box1, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, -kappa, box1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box1, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/true, -kappa, box2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, -kappa, box2);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/true, -kappa, box3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box3, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithBox(pos, tangent, ego_polygon,
                                           /*forward=*/false, -kappa, box3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, box3, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));
}

TEST(GeometryPathUtilTest, PolygonComputeFirstOverlapDistanceWithPolygon) {
  const Polygon2d polygon1(Box2d(Vec2d(0.9, 7.1), /*heading=*/1.0,
                                 /*length=*/4.0,
                                 /*width=*/2.0));
  const Polygon2d polygon2(Box2d(Vec2d(-1.9, 7.2), /*heading=*/M_PI_2,
                                 /*length=*/4.0,
                                 /*width=*/2.0));
  const Polygon2d polygon3(Box2d(Vec2d(3.1, 9.7), /*heading=*/M_PI_2,
                                 /*length=*/4.0,
                                 /*width=*/2.0));
  const Vec2d pos(1.0, 1.0);
  const Vec2d center(1.0, 2.0);
  const Vec2d tangent(0.0, 1.0);
  const Box2d ego_box(center, /*heading=*/M_PI_2, /*length=*/4.0,
                      /*width=*/2.0);
  const Polygon2d ego_polygon(ego_box);

  // Going Straight.
  auto res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                                    /*forward=*/true,
                                                    /*kappa=*/0.0, polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckStraightPathFirstOverlapDistance(ego_polygon, polygon1,
                                                    tangent, *res,
                                                    /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               polygon1);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               polygon2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               polygon2);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/true, /*kappa=*/0.0,
                                               polygon3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, /*kappa=*/0.0,
                                               polygon3);
  EXPECT_FALSE(res.has_value());

  // Turning left.
  const double kappa = 0.05;  // 1/m.
  res =
      ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon1, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon1, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, polygon2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon2, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               polygon2);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon2, pos,
                                                 tangent, *res, kappa,
                                                 /*forward=*/false));

  res =
      ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                             /*forward=*/true, kappa, polygon3);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, kappa,
                                               polygon3);
  EXPECT_FALSE(res.has_value());

  // Turning right.
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon1, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               polygon1);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon1, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));

  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               polygon2);
  EXPECT_FALSE(res.has_value());
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               polygon2);
  EXPECT_FALSE(res.has_value());

  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/true, -kappa,
                                               polygon3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon3, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/true));
  res = ComputeFirstOverlapDistanceWithPolygon(pos, tangent, ego_polygon,
                                               /*forward=*/false, -kappa,
                                               polygon3);
  EXPECT_TRUE(res.has_value());
  EXPECT_TRUE(CheckCurvePathFirstOverlapDistance(ego_polygon, polygon3, pos,
                                                 tangent, *res, -kappa,
                                                 /*forward=*/false));
}

}  // namespace planner
}  // namespace qcraft
