#ifndef ONBOARD_PLANNER_COMMON_GLOBAL_POSE_H_
#define ONBOARD_PLANNER_COMMON_GLOBAL_POSE_H_

#include <optional>
#include <string>

#include "common/proto/global_pose.pb.h"

#include "onboard/math/vec.h"

namespace qcraft {

struct GlobalPose {
  Vec3d pos;
  double heading;
};

struct GlobalPos2d {
  Vec2d pos;
  // It is not a smooth yaw, but diff slightly with the smooth yaw.
  std::optional<double> heading;
  std::string DebugString() const;
};

GlobalPose ConvertToGlobalPose(const GlobalPoseProto& proto);
GlobalPoseProto ConvertToGlobalPoseProto(const GlobalPose& global_pose);

}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_GLOBAL_POSE_H_
