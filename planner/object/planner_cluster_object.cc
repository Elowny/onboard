#include "onboard/planner/object/planner_cluster_object.h"

#include <algorithm>
#include <ostream>
#include <utility>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"

namespace qcraft {
namespace planner {

PlannerClusterObject::PlannerClusterObject(BoundaryClusterProto cluster_proto)
    : cluster_proto_(std::move(cluster_proto)) {
  QCHECK_NE(cluster_proto_.source_type(), BoundaryClusterProto::ST_NONE)
      << "Cluster object " << cluster_proto_.id() << " source type is NONE";
  QCHECK_NE(cluster_proto_.object_type(), BoundaryClusterProto::OT_UNKNOWN)
      << "Cluster object " << cluster_proto_.id() << " object type is UNKNOWN";
}

PlannerScatterClusterObject::PlannerScatterClusterObject(
    BoundaryClusterProto cluster_proto)
    : PlannerClusterObject(std::move(cluster_proto)) {
  QCHECK_EQ(cluster_proto_.geometry_type(), BoundaryClusterProto::GT_SCATTER);
  QCHECK_GT(cluster_proto_.points_size(), 0)
      << "Scatter cluster object " << cluster_proto_.id() << " has no points.";
  // TODO(renjie): May need to do time-alignment for moving objects.
  points_.reserve(cluster_proto_.points_size());
  for (const auto& p : cluster_proto_.points()) {
    points_.push_back(Vec2d(p.x(), p.y()));
  }
}

PlannerPolylineClusterObject::PlannerPolylineClusterObject(
    BoundaryClusterProto cluster_proto)
    : PlannerClusterObject(std::move(cluster_proto)) {
  QCHECK_EQ(cluster_proto_.geometry_type(), BoundaryClusterProto::GT_POLYLINE);
  QCHECK_GT(cluster_proto_.points_size(), 1)
      << "Polyline cluster object " << cluster_proto_.id() << " only has "
      << cluster_proto_.points_size() << " points.";
  // TODO(renjie): May need to do time-alignment for moving objects.
  const int seg_num = cluster_proto_.points_size() - 1;
  segments_.reserve(seg_num);
  for (int i = 0; i < seg_num; ++i) {
    segments_.emplace_back(
        Vec2d(cluster_proto_.points(i).x(), cluster_proto_.points(i).y()),
        Vec2d(cluster_proto_.points(i + 1).x(),
              cluster_proto_.points(i + 1).y()));
  }
}

PlannerPolygonClusterObject::PlannerPolygonClusterObject(
    BoundaryClusterProto cluster_proto)
    : PlannerClusterObject(std::move(cluster_proto)) {
  QCHECK_EQ(cluster_proto_.geometry_type(), BoundaryClusterProto::GT_POLYGON);
  QCHECK_GT(cluster_proto_.points_size(), 2)
      << "Polygon cluster object " << cluster_proto_.id() << " only has "
      << cluster_proto_.points_size() << " points.";
  // TODO(renjie): May need to do time-alignment for moving objects.
  std::vector<Vec2d> points;
  points.reserve(cluster_proto_.points_size());
  for (const auto& p : cluster_proto_.points()) {
    points.push_back(Vec2d(p.x(), p.y()));
  }

  contour_ = Polygon2d(std::move(points));
  QCHECK(contour_.is_convex())
      << "Polygon cluster object " << cluster_proto_.id() << " is not convex.";
}

}  // namespace planner
}  // namespace qcraft
