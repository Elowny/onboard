#ifndef ONBOARD_PLANNER_INITIALIZER_COLLISION_CHECKER_H_
#define ONBOARD_PLANNER_INITIALIZER_COLLISION_CHECKER_H_

#include <optional>
#include <vector>

#include "absl/types/span.h"

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// This struct represents the collision information of a motion edge.
struct CollisionInfo {
  // This struct contains information about an object collision. It contains the
  // object's index and the earliest collision time. The indices are based on
  // the order in `SpacetimeTrajectoryManager`.
  struct ObjectCollision {
    int time;
    const SpacetimeObjectTrajectory* traj;
    std::optional<CollisionConfiguration> collision_configuration =
        std::nullopt;
  };

  // All the objects that has collision with the edge.
  std::vector<ObjectCollision> collision_objects;
  void Clear() { collision_objects.clear(); }
};

CollisionConfiguration DetermineConfiguration(const Box2d& av_box,
                                              const Box2d& obj_box);

// The abstract API of a collision checker.
class CollisionChecker {
 public:
  virtual void UpdateStationaryObjectBuffer(double stationary_obj_buffer) = 0;

  virtual void UpdateMovingObjectBuffer(double moving_obj_buffer) = 0;

  // init_t is the first timestamp of the motion. `edge` is the motion edge, and
  // the collision objects are returned by `info`.
  virtual void CheckCollisionWithTrajectories(
      double init_t, absl::Span<const MotionState> states,
      const IgnoreTrajMap& ignored_trajs, CollisionInfo* info) const = 0;

  virtual void CheckCollisionWithStationaryObjects(
      absl::Span<const GeometryState> states, CollisionInfo* info) const = 0;

  virtual ~CollisionChecker() {}
};

// The collision checker algorithm uses time-aligned AABox2d to pre-filter
// non-collisions cases.
// For each prediction trajectory, it groups all of every `kGroupSize` object
// states into one time-range, and compute the aggregated AABox for each time
// range. For a given motion, it finds the aligned time range, and also create
// the aggregated AABox in each time range. When doing collision checking, it
// first use the AABoxes in the same time range to check for collisions. In this
// way, we can early exit on cases that has absolutely no collision.
//
// In the following figure, we first create three segment groups for an object
// trajectory, and compute three AABoxes.
// For the given motion, we find that it overlaps with three groups.
//
//  ----------------------------time--------------------------->
//                       <----kGroupSize--->
//  |___________________|___________________|___________________|  <- obj traj
//
//             |<------motion edge time-------------->| <--- a given motion.
//

//             |--------|-------------------|---------| <---motion box group.
//
//
struct SampledTrajectory;

class BoxGroupCollisionChecker : public CollisionChecker {
 public:
  struct ObjectBoxGroup {
    // The index of the object in SpacetimeTrajectoryManager.
    const SampledTrajectory* sampled_traj = nullptr;
    // For every `kGroupSize` object geometry, we compute one AABox.
    std::vector<AABox2d> aaboxes;
  };
  // Sample step is only for trajectories, not for paths.
  BoxGroupCollisionChecker(
      const SpacetimePlannerObjectTrajectories* st_planner_obj_traj,
      const VehicleGeometryParamsProto* vehicle_geometry, int sample_step,
      double stationary_obj_buffer, double moving_obj_buffer);

  void UpdateStationaryObjectBuffer(double stationary_obj_buffer) override;

  void UpdateMovingObjectBuffer(double moving_obj_buffer) override;

  // Returns indices of objects that has collision with the provided edge.
  void CheckCollisionWithTrajectories(double init_t,
                                      absl::Span<const MotionState> states,
                                      const IgnoreTrajMap& ignored_trajs,
                                      CollisionInfo* info) const override;

  // Check collision with static objects using geometry state. No downsampling
  // will be performed.
  void CheckCollisionWithStationaryObjects(
      absl::Span<const GeometryState> states,
      CollisionInfo* info) const override;

  // Returns the object trajectory sample step used inside collision checker.
  int sample_step() const { return sample_step_; }

 private:
  // Group every `kGroupSize` states into one big AABox2d.
  static constexpr int kGroupSize = 16;

  int sample_step_;

  double angle_sample_step_;
  // 1.0 / angle_sample_step_;
  double inv_angle_sample_step_;

  std::vector<Box2d> boxes_by_angles_;
  // The box  groups of moving trajectories.

  ObjectVector<ObjectBoxGroup> traj_box_groups_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;

  // A buffer is added to them.
  std::vector<SampledTrajectory> sampled_trajs_;
  std::vector<SpacetimeObjectState> stationary_obj_states_;
};

// Downsample the prediction trajectory states by every sample_step.
struct SampledTrajectory {
  // The sampled object states.
  std::vector<SpacetimeObjectState> states;
  // The pointer to original spacetime trajectory.
  const SpacetimeObjectTrajectory* traj;
};
std::vector<SampledTrajectory> SampleTrajectory(
    absl::Span<const SpacetimeObjectTrajectory* const> trajs, int sample_step);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_INITIALIZER_COLLISION_CHECKER_H_
