#ifndef ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_

#include <string>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

inline bool IsConsiderMirrorObject(const ObjectProto& object_proto,
                                   double min_mirror_height_avg,
                                   double max_mirror_height_avg) {
  if (!object_proto.has_min_z() || !object_proto.has_max_z() ||
      !object_proto.has_ground_z()) {
    return true;
  }
  const double object_max_height =
      object_proto.max_z() - object_proto.ground_z();
  const double object_min_height =
      object_proto.min_z() - object_proto.ground_z();
  return object_max_height > min_mirror_height_avg &&
         object_min_height < max_mirror_height_avg;
}

inline bool IsVehicle(ObjectType type) {
  return (type == ObjectType::OT_VEHICLE ||
          type == ObjectType::OT_LARGE_VEHICLE);
}

// Refer to https://qcraft.feishu.cn/docx/doxcnYZ47SsPqaBSVNtV0641Nqb
// For recognizing large vehicle (e.g. bus, truck, etc.).
bool IsLargeVehicle(const ObjectProto& object);

//  Returns the contour of a perception object.
Polygon2d ComputeObjectContour(const ObjectProto& object);

// Returns object proto representation of AV.
ObjectProto AvPoseProtoToObjectProto(
    const std::string& object_id,
    const VehicleGeometryParamsProto& vehicle_geom, const PoseProto& pose,
    bool offroad);

bool IsLidarObject(const ObjectProto& object);

bool IsCameraObject(const ObjectProto& object);

bool IsOccludedLidarObject(const ObjectProto& object);

bool IsOccludedCameraObject(const ObjectProto& object);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
