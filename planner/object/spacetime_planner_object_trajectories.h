#ifndef ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_H_
#define ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"

namespace qcraft {
namespace planner {

struct SpacetimePlannerObjectTrajectories {
  std::vector<SpacetimeObjectTrajectory> trajectories;
  // Spacetime planner trajectory and the selected reason.
  struct TrajectoryInfo {
    int traj_index;
    std::string object_id;
    SpacetimePlannerObjectTrajectoryReason::Type reason;
  };
  std::vector<TrajectoryInfo> trajectory_infos;
  absl::flat_hash_set<std::string> trajectory_ids;
  double st_start_offset = 0.0;

  void AddSpacetimePlannerObjectTrajectory(
      SpacetimeObjectTrajectory input_trajectory,
      const SpacetimePlannerObjectTrajectoryReason::Type reason) {
    if (!trajectory_ids.contains(input_trajectory.traj_id())) {
      trajectory_ids.insert(std::string(input_trajectory.traj_id()));
      trajectory_infos.push_back(
          {.traj_index = input_trajectory.traj_index(),
           .object_id = input_trajectory.planner_object().id(),
           .reason = reason});
      trajectories.push_back(std::move(input_trajectory));
    }
  }

  void ToProto(SpacetimePlannerObjectTrajectoriesProto* proto) const {
    const int num_trajs = trajectories.size();
    proto->mutable_trajectory()->Reserve(num_trajs);
    for (int i = 0; i < num_trajs; ++i) {
      auto* traj_proto = proto->add_trajectory();
      traj_proto->set_reason(trajectory_infos[i].reason);
      traj_proto->set_id(std::string(trajectories[i].traj_id()));
      traj_proto->set_index(trajectories[i].traj_index());
    }
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_SPACETIME_PLANNER_OBJECT_TRAJECTORIES_H_
