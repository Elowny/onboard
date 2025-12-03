#include "onboard/planner/speed/st_close_trajectory_generator.h"

#include <algorithm>
#include <string>
#include <utility>

// IWYU pragma: no_include "Eigen/Core"

#include "absl/strings/str_format.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft::planner {
namespace {
using StNearestPoint = StCloseTrajectory::StNearestPoint;
using CloseTrajPoints = std::vector<StNearestPoint>;
}  // namespace
std::vector<StCloseTrajectory> GenerateMovingStCloseTrajectories(
    const SpacetimeObjectTrajectory& st_traj,
    std::vector<CloseTrajPoints> close_traj_points) {
  std::vector<StCloseTrajectory> st_close_trajs;
  const auto object_type = ToStBoundaryObjectType(st_traj.object_type());
  if (close_traj_points.empty()) return st_close_trajs;

  const int traj_size = close_traj_points.size();
  st_close_trajs.reserve(traj_size);
  for (int i = 0; i < traj_size; ++i) {
    QCHECK(!close_traj_points[i].empty());
    auto traj_id = static_cast<std::string>(st_traj.traj_id());
    auto object_id = static_cast<std::string>(st_traj.object_id());
    std::string id =
        traj_size == 1 ? traj_id : absl::StrFormat("%s|%d", traj_id, i);

    st_close_trajs.emplace_back(std::move(close_traj_points[i]), std::move(id),
                                std::move(traj_id), std::move(object_id),
                                object_type, st_traj.trajectory().probability(),
                                st_traj.is_stationary());
  }
  return st_close_trajs;
}

}  // namespace qcraft::planner
