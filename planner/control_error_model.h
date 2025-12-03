#ifndef ONBOARD_PLANNER_CONTROL_ERROR_MODEL_H_
#define ONBOARD_PLANNER_CONTROL_ERROR_MODEL_H_

#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {

class ControlErrorModel {
 public:
  ControlErrorModel() = default;
  ControlErrorModel(const ApolloTrajectoryPointProto& plan_pose,
                    const PoseProto& pose) {
    const Vec2d traj_x(plan_pose.path_point().x(), plan_pose.path_point().y());
    const Vec2d pose_x(Vec3dFromProto(pose.pos_smooth()));
    x_error_ = pose_x - traj_x;
    const double traj_v = plan_pose.v();
    const double pose_v = pose.vel_body().x();
    v_error_ = pose_v - traj_v;
    const double traj_theta = plan_pose.path_point().theta();
    const double pose_theta = pose.yaw();
    theta_error_ = NormalizeAngle(pose_theta - traj_theta);
    traj_tangent_ = Vec2d::FastUnitFromAngle(traj_theta);
  }

  Vec2d x_error() const { return x_error_; }
  double v_error() const { return v_error_; }
  double theta_error() const { return theta_error_; }

  double lat_error() const { return x_error_.dot(traj_tangent_.Perp()); }
  double lon_error() const { return x_error_.dot(traj_tangent_); }

  // TODO(Fang) future error prediction using control's model.

 private:
  Vec2d traj_tangent_ = Vec2d::UnitX();
  Vec2d x_error_ = Vec2d::Zero();
  double v_error_ = 0.0;
  double theta_error_ = 0.0;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_CONTROL_ERROR_MODEL_H_
