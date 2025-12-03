#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_OBJECT_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_OBJECT_H_
#include <utility>

#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
constexpr double kStationarySpeedThreshold = 0.1;
template <typename V>
inline bool GetStationaryState(const V& obj_proto) {
  const bool is_low_speed =
      Vec2d(obj_proto.vel()).norm() < kStationarySpeedThreshold;
  const bool is_static_state =
      obj_proto.parked() ||
      (obj_proto.moving_state() == ObjectProto::MS_STATIC);
  const bool is_static_obj = (obj_proto.type() == OT_UNKNOWN_STATIC);
  return is_low_speed || is_static_state || is_static_obj;
}
template <typename V>
inline bool IsReversedMoving(const V& obj_proto) {
  const Vec2d vel(obj_proto.vel());
  const Vec2d heading = Vec2d::UnitFromAngle(obj_proto.yaw());
  if (vel.dot(heading) < 0) {
    return true;
  }
  return false;
}
class PredictionObject {
 public:
  template <typename VV>
  explicit PredictionObject(VV&& object_proto,
                            CoordinateConverter loc_converter)
      : origin_object_proto_(std::forward<VV>(object_proto)),
        transformed_object_proto_(origin_object_proto_),
        origin_loc_(std::move(loc_converter)),
        vec_vel_(Vec2d(origin_object_proto_.vel())),
        v_(Vec2d(origin_object_proto_.vel()).norm()),
        pos_(origin_object_proto_.pos()),
        heading_(origin_object_proto_.yaw()),
        bounding_box_(Box2d(origin_object_proto_.bounding_box())),
        is_stationary_(GetStationaryState(origin_object_proto_)),
        is_reversed_(IsReversedMoving(origin_object_proto_)) {}

  bool IsStationary() const { return is_stationary_; }
  bool IsReversed() const { return is_reversed_; }
  const ObjectIDType& id() const { return transformed_object_proto_.id(); }
  double v() const { return v_; }
  const Vec2d& pos() const { return pos_; }
  double heading() const { return heading_; }
  const Box2d& bounding_box() const { return bounding_box_; }
  const ObjectProto& object_proto() const { return transformed_object_proto_; }
  double timestamp() const { return transformed_object_proto_.timestamp(); }
  const Vec2d& vec_vel() const { return vec_vel_; }
  const CoordinateConverter& origin_coordinate_converter() {
    return origin_loc_;
  }
  void TransformCoordinate(const CoordinateConverter& target_loc);

 private:
  class ObjectProto origin_object_proto_;
  class ObjectProto transformed_object_proto_;
  CoordinateConverter origin_loc_;
  Vec2d vec_vel_;
  double v_;
  Vec2d pos_;
  double heading_;
  Box2d bounding_box_;
  bool is_stationary_;
  bool is_reversed_;
};
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_OBJECT_H_
