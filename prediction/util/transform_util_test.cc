#include "onboard/prediction/util/transform_util.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace qcraft::prediction {
namespace {
constexpr double kEpsilon = 1e-8;
PoseProto CreatePoseProto(const Vec3d& pos, double heading,
                          const Vec2d& vel_smooth, const Vec2d& acc_smooth,
                          double ar_smooth_z) {
  PoseProto pose;

  pose.mutable_pos_smooth()->set_x(pos.x());
  pose.mutable_pos_smooth()->set_y(pos.y());
  pose.mutable_pos_smooth()->set_z(pos.z());
  pose.set_yaw(heading);
  pose.set_speed(vel_smooth.norm());

  pose.mutable_vel_smooth()->set_x(vel_smooth.x());
  pose.mutable_vel_smooth()->set_y(vel_smooth.y());
  pose.mutable_vel_smooth()->set_z(0.0);

  pose.mutable_ar_smooth()->set_x(0.0);
  pose.mutable_ar_smooth()->set_y(0.0);
  pose.mutable_ar_smooth()->set_z(ar_smooth_z);

  pose.mutable_accel_smooth()->set_x(acc_smooth.x());
  pose.mutable_accel_smooth()->set_y(acc_smooth.y());
  pose.mutable_accel_smooth()->set_z(0.0);

  return pose;
}

ObjectProto CreateObjectProto(const Vec2d& pos, double yaw, const Vec2d& vel,
                              const Vec2d& accel) {
  ObjectProto objectproto;

  objectproto.mutable_pos()->set_x(pos.x());
  objectproto.mutable_pos()->set_y(pos.y());

  objectproto.set_yaw(yaw);

  objectproto.mutable_vel()->set_x(vel.x());
  objectproto.mutable_vel()->set_y(vel.y());

  objectproto.mutable_accel()->set_x(accel.x());
  objectproto.mutable_accel()->set_y(accel.y());

  objectproto.mutable_bounding_box()->set_heading(yaw);
  objectproto.mutable_bounding_box()->set_x(pos.x());
  objectproto.mutable_bounding_box()->set_y(pos.y());

  return objectproto;
}

// global = (smooth - smooth_ref).Rotate(yaw_diff) / scale + gloabl_ref
// smooth = ((global - global_ref) * scale).Rotate(-yaw_diff) + smooth_ref
// Figures for Test Case:
// https://qcraft.feishu.cn/docs/doccn4kn1H5so67GkqzUoMMlQ7g
TEST(TransformUtilTest, ConvertToUnifiedCoordinatePoseTest1) {
  PoseProto origin_pose = CreatePoseProto(/*pos=*/Vec3d(10.0, -20.0, -5.0),
                                          /*heading=*/0.0,
                                          /*vel_smooth=*/Vec2d(5.0, 0.0),
                                          /*acc_smooth=*/Vec2d(4.0, 3.0),
                                          /*ar_smooth_z=*/-M_PI / 2.0);

  CoordinateConverter origin;
  origin.UpdateLocalizationTransform(/*smooth_ref=*/Vec2d(-20.0, 10.0),
                                     /*global_ref=*/Vec2d(0.0, 0.0),
                                     /*z_diff=*/5.0,
                                     /*yaw_diff=*/M_PI / 2.0);
  CoordinateConverter unified;
  unified.UpdateLocalizationTransform(/*smooth_ref=*/Vec2d(10.0, -20.0),
                                      /*global_ref=*/Vec2d(0.0, 0.0),
                                      /*z_diff=*/-5.0,
                                      /*yaw_diff=*/-M_PI / 2.0);
  PoseProto unified_pose =
      ConvertToUnifiedCoordinatePose(origin_pose, origin, unified);

  EXPECT_NEAR(unified_pose.pos_smooth().x(), -20.0, kEpsilon);
  EXPECT_NEAR(unified_pose.pos_smooth().y(), 10.0, kEpsilon);
  EXPECT_NEAR(unified_pose.pos_smooth().z(), 5.0, kEpsilon);

  EXPECT_NEAR(unified_pose.vel_smooth().x(), -5.0, kEpsilon);
  EXPECT_NEAR(unified_pose.vel_smooth().y(), 0.0, kEpsilon);
  EXPECT_NEAR(unified_pose.vel_smooth().z(), 0.0, kEpsilon);

  EXPECT_NEAR(unified_pose.ar_smooth().x(), 0.0, kEpsilon);
  EXPECT_NEAR(unified_pose.ar_smooth().y(), 0.0, kEpsilon);
  EXPECT_NEAR(unified_pose.ar_smooth().z(), -M_PI / 2.0, kEpsilon);

  EXPECT_NEAR(unified_pose.accel_smooth().x(), -4.0, kEpsilon);
  EXPECT_NEAR(unified_pose.accel_smooth().y(), -3.0, kEpsilon);
  EXPECT_NEAR(unified_pose.accel_smooth().z(), 0.0, kEpsilon);

  EXPECT_NEAR(NormalizeAngle(unified_pose.yaw()), -M_PI, kEpsilon);
}

TEST(TransformUtilTest, ConvertToUnifiedCoordinatePoseTest2) {
  PoseProto origin_pose = CreatePoseProto(/*pos=*/Vec3d(20.0, 20.0, 0.0),
                                          /*heading=*/M_PI / 2.0,
                                          /*vel_smooth=*/Vec2d(0.0, 5.0),
                                          /*acc_smooth=*/Vec2d(0.0, 4.0),
                                          /*ar_smooth_z=*/-M_PI / 2.0);

  CoordinateConverter origin;
  origin.UpdateLocalizationTransform(/*smooth_ref=*/Vec2d(0.0, 0.0),
                                     /*global_ref=*/Vec2d(0.0, 0.0),
                                     /*z_diff=*/0.0,
                                     /*yaw_diff=*/0.0);
  CoordinateConverter unified;
  const double yaw_diff = M_PI / 6.0;
  Vec2d vector_bo(-20.0, -10.0);
  Vec2d unit_b_y2 = Vec2d(0.0, 1.0).Rotate(yaw_diff);
  Vec2d unit_b_x2 = Vec2d(1.0, 0.0).Rotate(yaw_diff);

  Vec2d smooth_ref = Vec2d(vector_bo.Dot(unit_b_x2), vector_bo.Dot(unit_b_y2));

  unified.UpdateLocalizationTransform(/*smooth_ref=*/smooth_ref,
                                      /*global_ref=*/Vec2d(0.0, 0.0),
                                      /*z_diff=*/0.0, /*yaw_diff=*/yaw_diff);
  PoseProto unified_pose =
      ConvertToUnifiedCoordinatePose(origin_pose, origin, unified);

  EXPECT_NEAR(unified_pose.pos_smooth().x(), (20.0 - 10.0) * std::sin(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_pose.pos_smooth().y(), (20.0 - 10.0) * std::cos(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_pose.pos_smooth().z(), 0.0, kEpsilon);

  EXPECT_NEAR(unified_pose.vel_smooth().x(), 5.0 * std::sin(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_pose.vel_smooth().y(), 5.0 * std::cos(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_pose.vel_smooth().z(), 0.0, kEpsilon);

  EXPECT_NEAR(unified_pose.ar_smooth().x(), 0.0, kEpsilon);
  EXPECT_NEAR(unified_pose.ar_smooth().y(), 0.0, kEpsilon);
  EXPECT_NEAR(unified_pose.ar_smooth().z(), -M_PI / 2.0, kEpsilon);

  EXPECT_NEAR(unified_pose.accel_smooth().x(), 4.0 * std::sin(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_pose.accel_smooth().y(), 4.0 * std::cos(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_pose.accel_smooth().z(), 0.0, kEpsilon);

  EXPECT_NEAR(NormalizeAngle(unified_pose.yaw()), M_PI / 3.0, kEpsilon);
}

TEST(TransformUtilTest, ConvertToUnifiedCoordinatePerceptionObjectTest1) {
  ObjectProto object_proto =
      CreateObjectProto(/*pos=*/Vec2d(10.0, -20.0), /*yaw=*/0.0,
                        /*vel=*/Vec2d(5.0, 0.0), /*accel=*/Vec2d(4.0, 3.0));

  CoordinateConverter origin;
  origin.UpdateLocalizationTransform(/*smooth_ref=*/Vec2d(-20.0, 10.0),
                                     /*global_ref=*/Vec2d(0.0, 0.0),
                                     /*z_diff=*/5.0,
                                     /*yaw_diff=*/M_PI / 2.0);
  CoordinateConverter unified;
  unified.UpdateLocalizationTransform(/*smooth_ref=*/Vec2d(10.0, -20.0),
                                      /*global_ref=*/Vec2d(0.0, 0.0),
                                      /*z_diff=*/-5.0,
                                      /*yaw_diff=*/-M_PI / 2.0);

  ObjectProto unified_object_proto =
      ConvertToUnifiedCoordinatePerceptionObject(object_proto, origin, unified);

  EXPECT_NEAR(unified_object_proto.pos().x(), -20.0, kEpsilon);
  EXPECT_NEAR(unified_object_proto.pos().y(), 10.0, kEpsilon);

  EXPECT_NEAR(unified_object_proto.vel().x(), -5.0, kEpsilon);
  EXPECT_NEAR(unified_object_proto.vel().y(), 0.0, kEpsilon);

  EXPECT_NEAR(unified_object_proto.accel().x(), -4.0, kEpsilon);
  EXPECT_NEAR(unified_object_proto.accel().y(), -3.0, kEpsilon);

  EXPECT_NEAR(NormalizeAngle(unified_object_proto.yaw()), -M_PI, kEpsilon);

  EXPECT_NEAR(unified_object_proto.bounding_box().x(), -20.0, kEpsilon);
  EXPECT_NEAR(unified_object_proto.bounding_box().y(), 10.0, kEpsilon);
  EXPECT_NEAR(unified_object_proto.bounding_box().heading(), -M_PI, kEpsilon);
}

TEST(TransformUtilTest, ConvertToUnifiedCoordinatePerceptionObjectTest2) {
  ObjectProto object_proto =
      CreateObjectProto(/*pos=*/Vec2d(20.0, 20.0), /*yaw=*/M_PI / 2.0,
                        /*vel=*/Vec2d(0.0, 5.0), /*accel=*/Vec2d(0.0, 4.0));

  CoordinateConverter origin;
  origin.UpdateLocalizationTransform(/*smooth_ref=*/Vec2d(0.0, 0.0),
                                     /*global_ref=*/Vec2d(0.0, 0.0),
                                     /*z_diff=*/0.0,
                                     /*yaw_diff=*/0.0);
  CoordinateConverter unified;
  const double yaw_diff = M_PI / 6.0;
  Vec2d vector_bo(-20.0, -10.0);
  Vec2d unit_b_y2 = Vec2d(0.0, 1.0).Rotate(yaw_diff);
  Vec2d unit_b_x2 = Vec2d(1.0, 0.0).Rotate(yaw_diff);

  Vec2d smooth_ref = Vec2d(vector_bo.Dot(unit_b_x2), vector_bo.Dot(unit_b_y2));

  unified.UpdateLocalizationTransform(/*smooth_ref=*/smooth_ref,
                                      /*global_ref=*/Vec2d(0.0, 0.0),
                                      /*z_diff=*/0.0, /*yaw_diff=*/yaw_diff);

  ObjectProto unified_object_proto =
      ConvertToUnifiedCoordinatePerceptionObject(object_proto, origin, unified);

  EXPECT_NEAR(unified_object_proto.pos().x(),
              (20.0 - 10.0) * std::sin(yaw_diff), kEpsilon);
  EXPECT_NEAR(unified_object_proto.pos().y(),
              (20.0 - 10.0) * std::cos(yaw_diff), kEpsilon);

  EXPECT_NEAR(unified_object_proto.vel().x(), 5.0 * std::sin(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_object_proto.vel().y(), 5.0 * std::cos(yaw_diff),
              kEpsilon);

  EXPECT_NEAR(unified_object_proto.accel().x(), 4.0 * std::sin(yaw_diff),
              kEpsilon);
  EXPECT_NEAR(unified_object_proto.accel().y(), 4.0 * std::cos(yaw_diff),
              kEpsilon);

  EXPECT_NEAR(NormalizeAngle(unified_object_proto.yaw()), M_PI / 3.0, kEpsilon);

  EXPECT_NEAR(unified_object_proto.bounding_box().x(),
              (20.0 - 10.0) * std::sin(yaw_diff), kEpsilon);
  EXPECT_NEAR(unified_object_proto.bounding_box().y(),
              (20.0 - 10.0) * std::cos(yaw_diff), kEpsilon);
  EXPECT_NEAR(unified_object_proto.bounding_box().heading(), M_PI / 3.0,
              kEpsilon);
}

}  // namespace
}  // namespace qcraft::prediction
