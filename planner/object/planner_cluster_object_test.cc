#include "onboard/planner/object/planner_cluster_object.h"

#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/test_util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEps = 1e-9;

TEST(PlannerClusterObjectTest, TestBuildPlannerScatterClusterObject) {
  BoundaryClusterProto cluster_proto;
  cluster_proto.set_source_type(BoundaryClusterProto::ST_VISION);
  cluster_proto.set_geometry_type(BoundaryClusterProto::GT_SCATTER);
  cluster_proto.set_object_type(BoundaryClusterProto::OT_GENERAL_BOUND);
  Vec3dProto p1, p2, p3;
  p1.set_x(1.0);
  p1.set_y(1.0);
  p1.set_z(1.0);
  p2.set_x(2.0);
  p2.set_y(2.0);
  p2.set_z(2.0);
  p3.set_x(3.0);
  p3.set_y(3.0);
  p3.set_z(3.0);
  *cluster_proto.add_points() = p1;
  *cluster_proto.add_points() = p2;
  *cluster_proto.add_points() = p3;

  const PlannerScatterClusterObject scatter_cluster_object(
      std::move(cluster_proto));
  EXPECT_EQ(scatter_cluster_object.points().size(), 3);
  EXPECT_THAT(scatter_cluster_object.points()[0], Vec2dNearXY(1.0, 1.0, kEps));
  EXPECT_THAT(scatter_cluster_object.points()[1], Vec2dNearXY(2.0, 2.0, kEps));
  EXPECT_THAT(scatter_cluster_object.points()[2], Vec2dNearXY(3.0, 3.0, kEps));
}

TEST(PlannerClusterObjectTest, TestBuildPlannerPolylineClusterObject) {
  BoundaryClusterProto cluster_proto;
  cluster_proto.set_source_type(BoundaryClusterProto::ST_VISION);
  cluster_proto.set_geometry_type(BoundaryClusterProto::GT_POLYLINE);
  cluster_proto.set_object_type(BoundaryClusterProto::OT_VEHICLE);
  Vec3dProto p1, p2, p3;
  p1.set_x(1.0);
  p1.set_y(1.0);
  p1.set_z(1.0);
  p2.set_x(2.0);
  p2.set_y(2.0);
  p2.set_z(2.0);
  p3.set_x(3.0);
  p3.set_y(4.0);
  p3.set_z(3.0);
  *cluster_proto.add_points() = p1;
  *cluster_proto.add_points() = p2;
  *cluster_proto.add_points() = p3;

  const PlannerPolylineClusterObject polyline_cluster_object(
      std::move(cluster_proto));
  EXPECT_EQ(polyline_cluster_object.segments().size(), 2);
  EXPECT_THAT(polyline_cluster_object.segments()[0].start(),
              Vec2dNearXY(1.0, 1.0, kEps));
  EXPECT_THAT(polyline_cluster_object.segments()[0].end(),
              Vec2dNearXY(2.0, 2.0, kEps));
  EXPECT_THAT(polyline_cluster_object.segments()[1].start(),
              Vec2dNearXY(2.0, 2.0, kEps));
  EXPECT_THAT(polyline_cluster_object.segments()[1].end(),
              Vec2dNearXY(3.0, 4.0, kEps));
}

TEST(PlannerClusterObjectTest, TestBuildPlannerPolygonClusterObject) {
  BoundaryClusterProto cluster_proto;
  cluster_proto.set_source_type(BoundaryClusterProto::ST_VISION);
  cluster_proto.set_geometry_type(BoundaryClusterProto::GT_POLYGON);
  cluster_proto.set_object_type(BoundaryClusterProto::OT_PEDESTRIAN);
  Vec3dProto p1, p2, p3;
  p1.set_x(1.0);
  p1.set_y(1.0);
  p1.set_z(1.0);
  p2.set_x(2.0);
  p2.set_y(2.0);
  p2.set_z(2.0);
  p3.set_x(3.0);
  p3.set_y(4.0);
  p3.set_z(3.0);
  *cluster_proto.add_points() = p1;
  *cluster_proto.add_points() = p2;
  *cluster_proto.add_points() = p3;

  const PlannerPolygonClusterObject polygon_cluster_object(
      std::move(cluster_proto));
  EXPECT_EQ(polygon_cluster_object.contour().num_points(), 3);
  EXPECT_THAT(polygon_cluster_object.contour().points()[0],
              Vec2dNearXY(1.0, 1.0, kEps));
  EXPECT_THAT(polygon_cluster_object.contour().points()[1],
              Vec2dNearXY(2.0, 2.0, kEps));
  EXPECT_THAT(polygon_cluster_object.contour().points()[2],
              Vec2dNearXY(3.0, 4.0, kEps));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
