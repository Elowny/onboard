
#include "onboard/prediction/util/transform_util.h"

#include "onboard/math/geometry/proto/box2d.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace prediction {
namespace {
// This function is for vectors that only need to be rotated when performing
// localization transform, such as speed/accel.
Vec3d RotateVec3d(const Vec3d& vec, const CoordinateConverter& origin,
                  const CoordinateConverter& unified) {
  const Vec2d origin_vec(vec.x(), vec.y());
  const Vec2d target_vec =
      unified.GlobalToSmoothRotate(origin.SmoothToGlobalRotate(origin_vec));
  return Vec3d(target_vec.x(), target_vec.y(), vec.z());
}
}  // namespace

PoseProto ConvertToUnifiedCoordinatePose(const PoseProto& origin_pose,
                                         const CoordinateConverter& origin,
                                         const CoordinateConverter& unified) {
  PoseProto pose = origin_pose;
  // pos_smooth.
  Vec3dToProto(unified.GlobalToSmooth(origin.SmoothToGlobal(
                   Vec3dFromProto(origin_pose.pos_smooth()))),
               pose.mutable_pos_smooth());

  // yaw.
  pose.set_yaw(
      unified.GlobalYawToSmooth(origin.SmoothYawToGlobal(origin_pose.yaw())));

  // vel_smooth
  Vec3dToProto(
      RotateVec3d(Vec3dFromProto(origin_pose.vel_smooth()), origin, unified),
      pose.mutable_vel_smooth());

  // ar_smooth
  Vec3dToProto(
      RotateVec3d(Vec3dFromProto(origin_pose.ar_smooth()), origin, unified),
      pose.mutable_ar_smooth());

  // accel_smooth
  Vec3dToProto(
      RotateVec3d(Vec3dFromProto(origin_pose.accel_smooth()), origin, unified),
      pose.mutable_accel_smooth());

  // NOTE(lidong): pos_smooth_cov, attitude_smooth_cov and vel_smooth_cov are
  // not converted.

  return pose;
}

ObjectProto ConvertToUnifiedCoordinatePerceptionObject(
    const ObjectProto& origin_object, const CoordinateConverter& origin,
    const CoordinateConverter& unified) {
  ObjectProto object = origin_object;

  // pos.
  Vec2dToProto(unified.GlobalToSmooth(
                   origin.SmoothToGlobal(Vec2dFromProto(origin_object.pos()))),
               object.mutable_pos());

  // yaw.
  object.set_yaw(
      unified.GlobalYawToSmooth(origin.SmoothYawToGlobal(origin_object.yaw())));

  // vel.
  Vec2dToProto(unified.GlobalToSmoothRotate(origin.SmoothToGlobalRotate(
                   Vec2dFromProto(origin_object.vel()))),
               object.mutable_vel());

  // accel.
  Vec2dToProto(unified.GlobalToSmoothRotate(origin.SmoothToGlobalRotate(
                   Vec2dFromProto(origin_object.accel()))),
               object.mutable_accel());

  // contour.
  for (int i = 0; i < origin_object.contour_size(); ++i) {
    Vec2dToProto(unified.GlobalToSmooth(origin.SmoothToGlobal(
                     Vec2dFromProto(origin_object.contour(i)))),
                 object.mutable_contour()->Mutable(i));
  }

  // Bounding box.
  object.mutable_bounding_box()->set_heading(unified.GlobalYawToSmooth(
      origin.SmoothYawToGlobal(origin_object.bounding_box().heading())));

  const Vec2d box_center = unified.GlobalToSmooth(origin.SmoothToGlobal(Vec2d(
      origin_object.bounding_box().x(), origin_object.bounding_box().y())));

  object.mutable_bounding_box()->set_x(box_center.x());
  object.mutable_bounding_box()->set_y(box_center.y());

  // NOTE(lidong): `pos_cov` is not converted.

  return object;
}

bool HasInvalidLocTransformInHistory(
    absl::Span<const LocalizationTransformProto* const> loc_transforms) {
  for (const auto* loc_trans : loc_transforms) {
    if (loc_trans->loc_status() != LocalizationTransformProto::kNormal) {
      return true;
    }
  }
  return false;
}

}  // namespace prediction
}  // namespace qcraft
