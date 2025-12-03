#ifndef onboard_planner_selector_selector_output_h
#define onboard_planner_selector_selector_output_h

#include "onboard/math/vec.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft::planner {

struct SelectorOutput {
  int selected_idx = -1;
  int best_traj_idx = -1;
  int last_selected_idx = -1;
  TurnSignal turn_signal = TurnSignal::TURN_SIGNAL_NONE;
  bool all_trajectories_blocked = false;
  std::optional<bool> is_going_force_route_change_left = std::nullopt;
  bool in_high_way = false;
  bool lane_change_for_obstacle_fail = false;
  std::optional<Vec2d> merge_point = std::nullopt;
};

};  // namespace qcraft::planner

#endif  // onboard_planner_selector_selector_output_h
