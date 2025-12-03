#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_CLUSTER_OBJECT_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_CLUSTER_OBJECT_H_

#include <stdint.h>

#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/proto/perception/parking/parking_freespace.pb.h"

namespace qcraft {
namespace planner {

// Stationary cluster objects represented by a cluster of points.
class PlannerClusterObject {
 public:
  using Id = uint32_t;

  explicit PlannerClusterObject(BoundaryClusterProto cluster_proto);

  const BoundaryClusterProto& cluster_proto() const { return cluster_proto_; }

  PlannerClusterObject::Id id() const { return cluster_proto_.id(); }

  BoundaryClusterProto::SourceType source() const {
    return cluster_proto_.source_type();
  }

  BoundaryClusterProto::ObjectType type() const {
    return cluster_proto_.object_type();
  }

 protected:
  BoundaryClusterProto cluster_proto_;
};

class PlannerScatterClusterObject : public PlannerClusterObject {
 public:
  explicit PlannerScatterClusterObject(BoundaryClusterProto cluster_proto);

  const std::vector<Vec2d>& points() const { return points_; }

 private:
  std::vector<Vec2d> points_;
};

class PlannerPolylineClusterObject : public PlannerClusterObject {
 public:
  explicit PlannerPolylineClusterObject(BoundaryClusterProto cluster_proto);

  const std::vector<Segment2d>& segments() const { return segments_; }

 private:
  std::vector<Segment2d> segments_;
};

class PlannerPolygonClusterObject : public PlannerClusterObject {
 public:
  explicit PlannerPolygonClusterObject(BoundaryClusterProto cluster_proto);

  const Polygon2d& contour() const { return contour_; }

 private:
  Polygon2d contour_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PLANNER_CLUSTER_OBJECT_H_
