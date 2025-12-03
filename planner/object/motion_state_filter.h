#ifndef ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_

#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

class MotionStateFilter : public TrajectoryFilter {
 public:
  MotionStateFilter(const PoseProto& pose,
                    const VehicleGeometryParamsProto& vehicle_geom);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  double yaw_;
  Vec2d tangent_;
  Vec2d pos_;
  Vec2d velocity_;
  double speed_;
  Vec2d motion_backoff_pos_;
  Vec2d stationary_backoff_pos_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_MOTION_STATE_FILTER_H_
