#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

namespace planner {

class TrajectoryFilter;

// The manager of all the spacetime object trajectories. Each prediction or
// stationary object is modeled as a `SpacetimeObjectTrajectory`.
class SpacetimeTrajectoryManager {
 public:
  SpacetimeTrajectoryManager() {}

  SpacetimeTrajectoryManager(absl::Span<const TrajectoryFilter* const> filters,
                             absl::Span<const PlannerObject> planner_objects,
                             ThreadPool* thread_pool);
  explicit SpacetimeTrajectoryManager(
      absl::Span<const PlannerObject> planner_objects,
      ThreadPool* thread_pool = nullptr)
      : SpacetimeTrajectoryManager(
            /*filters=*/{}, planner_objects, thread_pool) {}

  explicit SpacetimeTrajectoryManager(
      absl::Span<SpacetimeObjectTrajectory> spacetime_trajectories);

  // Deep copy.
  SpacetimeTrajectoryManager(const SpacetimeTrajectoryManager& other);
  SpacetimeTrajectoryManager& operator=(
      const SpacetimeTrajectoryManager& other);

  SpacetimeTrajectoryManager(SpacetimeTrajectoryManager&& other) = default;
  SpacetimeTrajectoryManager& operator=(SpacetimeTrajectoryManager&& other) =
      default;

  // All the considered stationary object 'trajectories'.
  absl::Span<const SpacetimeObjectTrajectory* const> stationary_object_trajs()
      const {
    return considered_stationary_trajs_;
  }

  // All the considered moving trajectories.
  absl::Span<const SpacetimeObjectTrajectory* const> moving_object_trajs()
      const {
    return considered_moving_trajs_;
  }

  // All the considered trajectories.
  absl::Span<const SpacetimeObjectTrajectory> trajectories() const {
    return considered_trajs_;
  }

  const absl::flat_hash_map<std::string_view,
                            std::vector<const SpacetimeObjectTrajectory*>>&
  object_trajectories_map() const {
    return objects_id_map_;
  }

  // Returns all the considered trajectories of an object id.
  absl::Span<const SpacetimeObjectTrajectory* const> FindTrajectoriesByObjectId(
      std::string_view id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return {};
    return iter->second;
  }

  // Returns the planner object of an object id.
  const PlannerObject* FindObjectByObjectId(std::string_view id) const {
    const auto iter = objects_id_map_.find(id);
    if (iter == objects_id_map_.end()) return nullptr;
    return &(iter->second.front()->planner_object());
  }

  // Returns the considered trajectory of traj id.
  const SpacetimeObjectTrajectory* FindTrajectoryById(
      std::string_view traj_id) const {
    return FindPtrOrNull(trajectories_id_map_, traj_id);
  }

  // An Ignored trajectory and the ignore reason.
  struct IgnoredTrajectory {
    const prediction::PredictedTrajectory* traj;
    std::string object_id;
    int index;
    FilterReason::Type reason;
  };
  // All the ignored trajectories.
  absl::Span<const IgnoredTrajectory> ignored_trajectories() const {
    return ignored_trajs_;
  }

  // Modifications.
  void FilterTrajectoriesWithReason(
      const std::map<std::string, FilterReason::Type>& id_reason_map);

 private:
  void UpdatePointers(int stationary_size);

  absl::flat_hash_map<std::string_view,
                      std::vector<const SpacetimeObjectTrajectory*>>
      objects_id_map_;
  absl::flat_hash_map<std::string_view, const SpacetimeObjectTrajectory*>
      trajectories_id_map_;

  std::vector<SpacetimeObjectTrajectory> considered_trajs_;
  std::vector<IgnoredTrajectory> ignored_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_stationary_trajs_;
  std::vector<const SpacetimeObjectTrajectory*> considered_moving_trajs_;
  // Can be accessed through ignored_trajectories. Keep this container so that
  // pointer in IgnoredTrajectory is valid. Need to update pointers.
  absl::flat_hash_map<std::string, SpacetimeObjectTrajectory> ignored_st_trajs_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_TRAJECTORY_MANAGER_H_
