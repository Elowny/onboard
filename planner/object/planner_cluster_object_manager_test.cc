#include "onboard/planner/object/planner_cluster_object_manager.h"

#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/proto/perception/parking/parking_freespace.pb.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PlannerClusterObjectManagerTest, TestBuildPlannerClusterObjectManager) {
  BoundaryClusterProto scatter_cluster_proto_1;
  scatter_cluster_proto_1.set_id(1);
  scatter_cluster_proto_1.set_source_type(BoundaryClusterProto::ST_VISION);
  scatter_cluster_proto_1.set_geometry_type(BoundaryClusterProto::GT_SCATTER);
  scatter_cluster_proto_1.set_object_type(
      BoundaryClusterProto::OT_GENERAL_BOUND);
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
  *scatter_cluster_proto_1.add_points() = p1;
  *scatter_cluster_proto_1.add_points() = p2;
  *scatter_cluster_proto_1.add_points() = p3;

  BoundaryClusterProto scatter_cluster_proto_2;
  scatter_cluster_proto_2.set_id(2);
  scatter_cluster_proto_2.set_source_type(BoundaryClusterProto::ST_VISION);
  scatter_cluster_proto_2.set_geometry_type(BoundaryClusterProto::GT_SCATTER);
  scatter_cluster_proto_2.set_object_type(
      BoundaryClusterProto::OT_GENERAL_BOUND);
  Vec3dProto p4, p5, p6;
  p4.set_x(4.0);
  p4.set_y(4.0);
  p4.set_z(4.0);
  p5.set_x(5.0);
  p5.set_y(5.0);
  p5.set_z(5.0);
  p6.set_x(6.0);
  p6.set_y(6.0);
  p6.set_z(6.0);
  *scatter_cluster_proto_2.add_points() = p4;
  *scatter_cluster_proto_2.add_points() = p5;
  *scatter_cluster_proto_2.add_points() = p6;

  BoundaryClusterProto polyline_cluster_proto_1;
  polyline_cluster_proto_1.set_id(3);
  polyline_cluster_proto_1.set_source_type(BoundaryClusterProto::ST_VISION);
  polyline_cluster_proto_1.set_geometry_type(BoundaryClusterProto::GT_POLYLINE);
  polyline_cluster_proto_1.set_object_type(
      BoundaryClusterProto::OT_GENERAL_BOUND);
  Vec3dProto p7, p8, p9;
  p7.set_x(7.0);
  p7.set_y(7.0);
  p7.set_z(7.0);
  p8.set_x(8.0);
  p8.set_y(8.0);
  p8.set_z(8.0);
  p9.set_x(9.0);
  p9.set_y(9.0);
  p9.set_z(9.0);
  *polyline_cluster_proto_1.add_points() = p7;
  *polyline_cluster_proto_1.add_points() = p8;
  *polyline_cluster_proto_1.add_points() = p9;

  BoundaryClusterProto polygon_cluster_proto_1;
  polygon_cluster_proto_1.set_id(4);
  polygon_cluster_proto_1.set_source_type(BoundaryClusterProto::ST_VISION);
  polygon_cluster_proto_1.set_geometry_type(BoundaryClusterProto::GT_POLYGON);
  polygon_cluster_proto_1.set_object_type(
      BoundaryClusterProto::OT_GENERAL_BOUND);

  Vec3dProto p10, p11, p12;
  p10.set_x(10.0);
  p10.set_y(10.0);
  p10.set_z(10.0);
  p11.set_x(11.0);
  p11.set_y(11.0);
  p11.set_z(11.0);
  p12.set_x(12.0);
  p12.set_y(10.0);
  p12.set_z(12.0);
  *polygon_cluster_proto_1.add_points() = p10;
  *polygon_cluster_proto_1.add_points() = p11;
  *polygon_cluster_proto_1.add_points() = p12;

  const PlannerClusterObjectManager cluster_object_manager(
      {scatter_cluster_proto_1, scatter_cluster_proto_2,
       polyline_cluster_proto_1, polygon_cluster_proto_1});

  EXPECT_EQ(cluster_object_manager.scatter_objects().size(), 2);
  EXPECT_TRUE(cluster_object_manager.FindScatterObjectById(1) != nullptr);
  EXPECT_TRUE(cluster_object_manager.FindScatterObjectById(2) != nullptr);
  EXPECT_TRUE(cluster_object_manager.FindScatterObjectById(3) == nullptr);

  EXPECT_EQ(cluster_object_manager.polyline_objects().size(), 1);
  EXPECT_TRUE(cluster_object_manager.FindPolylineObjectById(1) == nullptr);
  EXPECT_TRUE(cluster_object_manager.FindPolylineObjectById(2) == nullptr);
  EXPECT_TRUE(cluster_object_manager.FindPolylineObjectById(3) != nullptr);

  EXPECT_EQ(cluster_object_manager.polygon_objects().size(), 1);
  EXPECT_TRUE(cluster_object_manager.FindPolygonObjectById(1) == nullptr);
  EXPECT_TRUE(cluster_object_manager.FindPolygonObjectById(2) == nullptr);
  EXPECT_TRUE(cluster_object_manager.FindPolygonObjectById(3) == nullptr);
  EXPECT_TRUE(cluster_object_manager.FindPolygonObjectById(4) != nullptr);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
