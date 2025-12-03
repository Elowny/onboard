#include "onboard/planner/object/motion_state_filter.h"

#include <cmath>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft {
namespace planner {

MotionStateFilter::MotionStateFilter(
    const PoseProto& pose, const VehicleGeometryParamsProto& vehicle_geom) {
  yaw_ = pose.yaw();
  pos_ = Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
  tangent_ = Vec2d::FastUnitFromAngle(pose.yaw());
  speed_ = pose.speed();
  velocity_ = Vec2d(pose.vel_smooth().x(), pose.vel_smooth().y());

  constexpr double kBackOffDistance = 2.0;  // Meters.
  motion_backoff_pos_ =
      pos_ - tangent_ * (kBackOffDistance + vehicle_geom.back_edge_to_center());
  stationary_backoff_pos_ = pos_;
}

FilterReason::Type MotionStateFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  // When AV is not driving forward, don't filter.
  if (tangent_.Dot(velocity_) < 0.0) {
    return FilterReason::NONE;
  }

  const auto& contour = object.contour();
  const Vec2d obj_center = contour.CircleCenter();

  if (prediction::IsStationaryTrajectory(traj)) {
    const Vec2d av_to_obj = obj_center - stationary_backoff_pos_;
    const double proj_lon = tangent_.Dot(av_to_obj);
    const double padding = contour.CircleRadius();
    if (proj_lon < -padding) {
      return FilterReason::STATIONARY_OBJECT_BEHIND_AV;
    }
  } else {
    const Vec2d av_to_obj = obj_center - motion_backoff_pos_;
    const double proj_lon = tangent_.Dot(av_to_obj);
    const double padding = contour.CircleRadius();
    // For moving object, if the heading is within 60 degrees of the opposite
    // AV moving direction, ignore.
    constexpr double kOppositeDirectionThreshold = M_PI_4;
    if (proj_lon < -padding &&
        std::abs(AngleDifference(object.pose().theta(), OppositeAngle(yaw_))) <
            kOppositeDirectionThreshold) {
      return FilterReason::OBJECT_BEHIND_MOVING_AWAY_FROM_AV;
    }
  }
  return FilterReason::NONE;

  // Object is moving.
}

}  // namespace planner
}  // namespace qcraft
