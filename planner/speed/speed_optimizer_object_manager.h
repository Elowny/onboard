#ifndef ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_

#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_optimizer_object.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"

namespace qcraft::planner {

enum SpeedOptimizerObjectType {
  MOVING_FOLLOW = 0,  // NOLINT(readability-identifier-naming)
  MOVING_LEAD = 1,    // NOLINT(readability-identifier-naming)
  STATIONARY = 2,     // NOLINT(readability-identifier-naming)
};

// Manage the speed optimizer objects those used in speed optimizer. Multiple
// moving st_trajs for the same object_id and the same decision will be
// integrated into a speed optimizer object and collected into the FOLLOW/LEAD
// type. Stationary st_traj will be separately collected into the STATIONARY
// type.
class SpeedOptimizerObjectManager {
 public:
  SpeedOptimizerObjectManager(
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      const SpeedVector* preliminary_speed,
      const SpacetimeTrajectoryManager& traj_mgr, double av_speed,
      double plan_total_time, double plan_time_interval,
      const SpeedFinderParamsProto& speed_finder_params);

  // Return speed optimizer objects with follow decision.
  absl::Span<const SpeedOptimizerObject> MovingFollowObjects() const {
    return objects_.at(SpeedOptimizerObjectType::MOVING_FOLLOW);
  }

  // Return speed optimizer objects with follow decision.
  absl::Span<const SpeedOptimizerObject> MovingLeadObjects() const {
    return objects_.at(SpeedOptimizerObjectType::MOVING_LEAD);
  }

  // Return stationary speed optimizer objects.
  absl::Span<const SpeedOptimizerObject> StationaryObjects() const {
    return objects_.at(SpeedOptimizerObjectType::STATIONARY);
  }

 private:
  std::vector<std::vector<SpeedOptimizerObject>> objects_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_
