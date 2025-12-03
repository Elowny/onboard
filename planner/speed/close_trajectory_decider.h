#ifndef ONBOARD_PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_

#include <optional>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/st_close_trajectory.h"
namespace qcraft::planner {

std::vector<std::optional<SpeedLimit>> GetMovingCloseTrajSpeedLimits(
    absl::Span<const StCloseTrajectory> st_close_trajs, double path_length,
    double av_speed, double time_step, double max_time);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_
