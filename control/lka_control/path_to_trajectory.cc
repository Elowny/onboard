#include "onboard/control/lka_control/path_to_trajectory.h"

#include "absl/status/status.h"
#include "absl/time/time.h"

#include "onboard/global/clock.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace control {

absl::StatusOr<TrajectoryProto> PathToTrajectory(
    const std::vector<PathPoint>& path) {
  if (path.empty()) return absl::InternalError("planner path is empty !");
  TrajectoryProto trajectory;

  trajectory.mutable_header()->set_timestamp(absl::ToUnixMicros(Clock::Now()));
  trajectory.set_trajectory_start_timestamp(ToUnixDoubleSeconds(Clock::Now()));

  for (int i = 0; i < path.size(); ++i) {
    auto* trajectory_point = trajectory.mutable_trajectory_point()->Add();
    trajectory_point->mutable_path_point()->CopyFrom(path[i]);
    trajectory_point->set_v(0.0);
    trajectory_point->set_a(0.0);
    trajectory_point->set_j(0.0);
    trajectory_point->set_relative_time(planner::kTrajectoryTimeStep * i);
  }

  trajectory.set_gear(Chassis::GEAR_DRIVE);
  trajectory.set_low_speed_freespace(false);
  trajectory.set_enable_stationary_steering(false);
  trajectory.set_aeb_triggered(false);
  trajectory.mutable_door_decision()->set_door_state(DoorDecision::DOOR_CLOSE);
  trajectory.mutable_door_decision()->set_reason(DoorDecision::DEFAULT_CLOSE);
  trajectory.set_turn_signal(TurnSignal::TURN_SIGNAL_NONE);
  return trajectory;
}

}  // namespace control
}  // namespace qcraft
