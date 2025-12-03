#ifndef ONBOARD_PLANNER_OBJECT_PREDICTED_MOTION_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_PREDICTED_MOTION_FILTER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/aabox_kdtree2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace planner {

// This filter checks the prediction trajectory of each object, and ignores
// a trajectory if it has collision with another object that has confident
// trajectory. A confident trajectory of an object is a trajectory that we
// believe is likely to be followed by the object.
class PredictedMotionFilter : public TrajectoryFilter {
 public:
  // Stores information of an object state that is a confident state. A
  // confident state means that if there is an collision between two states, we
  // trust the confident state.
  struct ObjectCollisionCheckingState {
    // The id of the object that owns the state.
    std::string id;
    // The box that represents the shape of the object at the state. It might be
    // smaller than planner object's box.
    Box2d box;
    AABox2d ComputeAABox() const { return box.GetAABox(); }

    // For AABoxKDTree2d.
    double DistanceSquareTo(const Vec2d& point) const {
      return box.DistanceSquareTo(point);
    }
  };

  explicit PredictedMotionFilter(
      absl::Span<const PlannerObject> planner_objects);

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  std::vector<ObjectCollisionCheckingState> confident_object_states_;
  std::unique_ptr<AABoxKDTree2d<ObjectCollisionCheckingState>> kdtree_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PREDICTED_MOTION_FILTER_H_
