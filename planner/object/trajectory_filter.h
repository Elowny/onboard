#ifndef ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_

#include <memory>
#include <vector>

#include "onboard/planner/object/planner_object.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

class TrajectoryFilter {
 public:
  virtual FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const = 0;

  virtual ~TrajectoryFilter() {}
};

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_OBJECT_TRAJECTORY_FILTER_H_
