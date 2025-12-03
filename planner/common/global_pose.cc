#include "onboard/planner/common/global_pose.h"

#include <string>

#include "absl/strings/str_format.h"

#include "common/proto/map_geometry.pb.h"

namespace qcraft {

GlobalPose ConvertToGlobalPose(const GlobalPoseProto& proto) {
  return {.pos = Vec3d(proto.geo_location().longitude(),
                       proto.geo_location().latitude(),
                       proto.geo_location().altitude()),
          .heading = proto.yaw()};
}

GlobalPoseProto ConvertToGlobalPoseProto(const GlobalPose& global_pose) {
  GlobalPoseProto proto;
  proto.mutable_geo_location()->set_longitude(global_pose.pos.x());
  proto.mutable_geo_location()->set_latitude(global_pose.pos.y());
  proto.mutable_geo_location()->set_altitude(global_pose.pos.z());
  proto.set_yaw(global_pose.heading);
  return proto;
}

std::string GlobalPos2d::DebugString() const {
  return absl::StrFormat(
      "pos:{%.8f %.8f}, heading: %s", pos.x(), pos.y(),
      heading.has_value() ? std::to_string(*heading) : "none");
}

}  // namespace qcraft
