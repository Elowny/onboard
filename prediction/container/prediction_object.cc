#include "onboard/prediction/container/prediction_object.h"

#include "onboard/prediction/util/transform_util.h"
namespace qcraft {
namespace prediction {
void PredictionObject::TransformCoordinate(
    const CoordinateConverter& target_loc) {
  transformed_object_proto_ =
      prediction::ConvertToUnifiedCoordinatePerceptionObject(
          origin_object_proto_, origin_loc_, target_loc);
  vec_vel_ = Vec2d(transformed_object_proto_.vel());
  v_ = vec_vel_.norm();
  pos_ = Vec2d(transformed_object_proto_.pos());
  heading_ = transformed_object_proto_.yaw();
  bounding_box_ = Box2d(transformed_object_proto_.bounding_box());
}

}  // namespace prediction
}  // namespace qcraft
