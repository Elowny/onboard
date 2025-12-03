#include "onboard/planner/object/spacetime_trajectory_manager.h"

#include <algorithm>
#include <iterator>
#include <ostream>

#include "onboard/async/parallel_for.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {
namespace {
template <typename T>
using NestedVector = std::vector<std::vector<T>>;

// This is the object buffer that AV should never enter.
double ComputeRequiredLateralGap(const PlannerObject& object) {
  switch (object.type()) {
    case OT_FOD:
      return 0.0;
    case OT_UNKNOWN_STATIC:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_BARRIER_ANTI_COLLISION_BUCKET:
    case OT_BARRIER_ANTI_COLLISION_POST:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.15;
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return 0.2;
  }
}
}  // namespace

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    absl::Span<const TrajectoryFilter* const> filters,
    absl::Span<const PlannerObject> planner_objects, ThreadPool* thread_pool) {
  SCOPED_QTRACE("SpacetimeTrajectoryManager");

  const int num_objects = planner_objects.size();
  NestedVector<SpacetimeObjectTrajectory> considered_trajs_per_object(
      num_objects);
  NestedVector<IgnoredTrajectory> ignored_trajs_per_object(num_objects);

  ParallelFor(0, num_objects, thread_pool, [&](int i) {
    const auto& planner_object = planner_objects[i];
    const auto& trajectories = planner_object.prediction().trajectories();
    QCHECK(!trajectories.empty())
        << planner_object.id() << " has no trajectory.";
    for (int traj_index = 0, s = trajectories.size(); traj_index < s;
         ++traj_index) {
      const auto& pred_traj = trajectories[traj_index];
      bool filtered = false;
      for (const auto* filter : filters) {
        const auto filter_reason = filter->Filter(planner_object, pred_traj);
        if (filter_reason != FilterReason::NONE) {
          ignored_trajs_per_object[i].push_back(
              {.traj = &pred_traj,
               .object_id = planner_object.id(),
               .index = pred_traj.index(),
               .reason = filter_reason});
          filtered = true;
          break;
        }
      }
      if (filtered) continue;

      const double required_lateral_gap =
          ComputeRequiredLateralGap(planner_object);
      considered_trajs_per_object[i].emplace_back(planner_object, traj_index,
                                                  required_lateral_gap);
    }
  });

  int trajectories_size = 0;
  for (const auto& planner_object : planner_objects) {
    trajectories_size += planner_object.prediction().trajectories().size();
  }
  considered_trajs_.reserve(trajectories_size);
  ignored_trajs_.reserve(trajectories_size);

  // Collect results from parallel for.
  int stationary_count = 0;
  for (auto& trajs : considered_trajs_per_object) {
    for (const auto& traj : trajs) {
      // Count stationary trajs in all considered trajs.
      if (traj.is_stationary()) stationary_count++;
    }
    std::move(trajs.begin(), trajs.end(),
              std::back_inserter(considered_trajs_));
  }
  for (auto& trajs : ignored_trajs_per_object) {
    std::move(trajs.begin(), trajs.end(), std::back_inserter(ignored_trajs_));
  }

  // Classify trajectories.
  UpdatePointers(stationary_count);
}

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    absl::Span<SpacetimeObjectTrajectory> spacetime_trajectories) {
  const int traj_size = spacetime_trajectories.size();
  considered_trajs_.reserve(traj_size);
  int stationary_count = 0;
  for (auto& traj : spacetime_trajectories) {
    if (traj.is_stationary()) {
      stationary_count++;
    }
    considered_trajs_.push_back(std::move(traj));
  }
  UpdatePointers(stationary_count);
}

SpacetimeTrajectoryManager::SpacetimeTrajectoryManager(
    const SpacetimeTrajectoryManager& other) {
  *this = other;
}

SpacetimeTrajectoryManager& SpacetimeTrajectoryManager::operator=(
    const SpacetimeTrajectoryManager& other) {
  SCOPED_QTRACE("SpacetimeTrajectoryManager/CopyAssignment");
  if (this != &other) {
    ignored_trajs_ = other.ignored_trajs_;
    considered_trajs_ = other.considered_trajs_;
    UpdatePointers(other.considered_stationary_trajs_.size());
  }
  return *this;
}

// Modifications.
void SpacetimeTrajectoryManager::FilterTrajectoriesWithReason(
    const std::map<std::string, FilterReason::Type>& id_reason_map) {
  SCOPED_QTRACE_ARG1("SpacetimeTrajectoryManager/FilterTrajectoriesWithReason",
                     "num to filter", id_reason_map.size());
  std::vector<SpacetimeObjectTrajectory> remain_to_consider;
  remain_to_consider.reserve(considered_trajs_.size() - id_reason_map.size());
  int remain_to_consider_stationary_count = 0;
  for (auto& considered_traj : considered_trajs_) {
    const auto* reason =
        FindOrNull(id_reason_map, std::string(considered_traj.traj_id()));
    if (reason == nullptr) {
      if (considered_traj.is_stationary()) {
        remain_to_consider_stationary_count++;
      }
      remain_to_consider.push_back(std::move(considered_traj));
      continue;
    }
    const auto traj_id = std::string(considered_traj.traj_id());
    auto [it, success] =
        ignored_st_trajs_.try_emplace(traj_id, std::move(considered_traj));
    if (success) {
      ignored_trajs_.emplace_back(IgnoredTrajectory{
          .traj = &it->second.trajectory(),
          .object_id = std::string(it->second.object_id()),
          .index = it->second.traj_index(),
          .reason = *reason,
      });
    }
  }
  considered_trajs_ = std::move(remain_to_consider);
  UpdatePointers(remain_to_consider_stationary_count);
}

void SpacetimeTrajectoryManager::UpdatePointers(int stationary_size) {
  SCOPED_QTRACE("SpacetimeTrajectoryManager/UpdatePointers");
  considered_stationary_trajs_.clear();
  considered_moving_trajs_.clear();
  objects_id_map_.clear();
  trajectories_id_map_.clear();
  const int traj_size = considered_trajs_.size();
  considered_stationary_trajs_.reserve(stationary_size);
  QCHECK_GE(traj_size, stationary_size);
  considered_moving_trajs_.reserve(traj_size - stationary_size);
  objects_id_map_.reserve(traj_size);
  trajectories_id_map_.reserve(traj_size);
  for (const auto& traj : considered_trajs_) {
    if (traj.is_stationary()) {
      considered_stationary_trajs_.push_back(&traj);
    } else {
      considered_moving_trajs_.push_back(&traj);
    }
    objects_id_map_[traj.planner_object().id()].push_back(&traj);
    trajectories_id_map_[traj.traj_id()] = &traj;
  }

  for (auto& ignored_traj : ignored_trajs_) {
    // Update pointers to PredictedTrajectory in SpacetimeObjectTrajectories
    // stored inside SpacetimeTrajectoryManager,
    const auto* ignored_st_traj = FindOrNull(
        ignored_st_trajs_, SpacetimeObjectTrajectory::MakeTrajectoryId(
                               ignored_traj.object_id, ignored_traj.index));
    if (ignored_st_traj == nullptr) continue;
    ignored_traj.traj = &ignored_st_traj->trajectory();
  }
}

}  // namespace planner
}  // namespace qcraft
