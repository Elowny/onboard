#ifndef ONBOARD_PLANNER_INITIALIZER_BRUTE_FORCE_COLLISION_CHECKER_H_
#define ONBOARD_PLANNER_INITIALIZER_BRUTE_FORCE_COLLISION_CHECKER_H_

#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// This class is mainly used to valid and benchmark complicated collision
// checker.
class BruteForceCollisionChecker : public CollisionChecker {
 public:
  BruteForceCollisionChecker(
      absl::Span<const SpacetimeObjectTrajectory> trajectories,
      const VehicleGeometryParamsProto* vehicle_geom, int sample_step,
      double stationary_buffer, double moving_buffer);

  void UpdateStationaryObjectBuffer(double stationary_buffer) override;
  void UpdateMovingObjectBuffer(double moving_buffer) override;

  void CheckCollisionWithTrajectories(double init_t,
                                      absl::Span<const MotionState> states,
                                      const IgnoreTrajMap& ignored_trajs,
                                      CollisionInfo* info) const override;
  void CheckCollisionWithStationaryObjects(
      absl::Span<const GeometryState> states,
      CollisionInfo* info) const override;

  int sample_step() const { return sample_step_; }

 private:
  const VehicleGeometryParamsProto* vehicle_geom_;
  int sample_step_;
  double stationary_buffer_;
  double moving_buffer_;
  std::vector<SampledTrajectory> sampled_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_INITIALIZER_BRUTE_FORCE_COLLISION_CHECKER_H_
