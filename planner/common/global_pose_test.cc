#include "onboard/planner/common/global_pose.h"

#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/utils/proto_util.h"

namespace qcraft::planner {
namespace {
TEST(GlobalPoseTest, GlobalPose2d) {
  GlobalPos2d pos = {
      .pos = {1.0, 2.0},
      .heading = 0.717,
  };
  ASSERT_FALSE(pos.DebugString().empty());
}

TEST(GlobalPoseTest, GlobalPose) {
  GlobalPoseProto proto;
  auto* geo_loc = proto.mutable_geo_location();
  geo_loc->set_longitude(1.0);
  geo_loc->set_latitude(0.5);
  geo_loc->set_altitude(0.3);
  proto.set_yaw(0.4);

  auto pose = ConvertToGlobalPose(proto);
  auto other_proto = ConvertToGlobalPoseProto(pose);
  ASSERT_TRUE(ProtoEquals(proto, other_proto));
}

}  // namespace
}  // namespace qcraft::planner
