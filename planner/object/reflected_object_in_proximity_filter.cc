#include "onboard/planner/object/reflected_object_in_proximity_filter.h"

#include <algorithm>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

namespace {
bool ReflectedObjectType(ObjectType type) {
  switch (type) {
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_FOD:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
    case ObjectType::OT_VEGETATION:
      return true;
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_UNKNOWN_MOVABLE:
      return false;
  }
}
}  // namespace

ReflectedObjectInProximityFilter::ReflectedObjectInProximityFilter(
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom,
    double padding) {
  const Vec2d pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  // Don't pad the front bumper.
  const Vec2d tangent = Vec2d::FastUnitFromAngle(pose.yaw());
  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  const double rac_to_center =
      half_length - vehicle_geom.back_edge_to_center() - 0.5 * padding;
  const Vec2d center = pos + tangent * rac_to_center;
  padded_box_ = Box2d(half_length + 0.5 * padding, half_width + padding, center,
                      pose.yaw(), tangent);
}

FilterReason::Type ReflectedObjectInProximityFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& /*traj*/) const {
  // Don't filter objects that does not satisfy reflection object type.
  if (!ReflectedObjectType(object.type())) {
    return FilterReason::NONE;
  }

  // Don't filter objects that are too large.
  if (std::max(object.bounding_box().length(), object.bounding_box().width()) >
      padded_box_.length()) {
    return FilterReason::NONE;
  }

  const auto& contour = object.contour();

  // When the object has overlap with AV padded box, filter it.
  if (contour.HasOverlap(padded_box_)) {
    return FilterReason::REFLECTED_OBJECT_IN_PROXIMITY;
  }
  return FilterReason::NONE;
}

}  // namespace planner
}  // namespace qcraft
