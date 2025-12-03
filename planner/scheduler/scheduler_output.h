#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_OUTPUT_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_OUTPUT_H_

#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/hash/hash.h"

#include "onboard/math/frenet_frame.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft::planner {

struct SchedulerOutput {
  bool is_fallback = false;
  bool is_expert = false;
  // TODO(weijun): Move drive_passage.h to scheduler folder.
  DrivePassage drive_passage;
  // l-s boundaries on drive passage center.
  PathSlBoundary sl_boundary;
  // Should only be updated in async high-freq module and remains 0.0 otherwise.
  double target_offset_from_start = 0.0;
  LaneChangeStateProto lane_change_state;
  // For constraints on lane change pause stage, empty if not changing lane.
  mapping::LanePath lane_path_before_lc;
  double length_along_route = std::numeric_limits<double>::max();
  double max_reach_length = std::numeric_limits<double>::max();
  double standard_congestion_factor = 0.0;
  double traffic_congestion_factor = 0.0;
  bool blocked_abreast = false;
  bool should_smooth = false;
  bool borrow_lane = false;
  bool is_solid_lane_change = false;
  FrenetBox av_frenet_box_on_drive_passage;
  // Whether there is enough distance to lane change.
  bool request_help_lane_change_by_route = false;
  bool switch_alternate_route = false;
  TurnSignal planner_turn_signal = TURN_SIGNAL_NONE;
  TurnSignalReason turn_signal_reason = TURN_SIGNAL_OFF;

  using HashType = std::tuple<mapping::ElementId, bool, bool>;
  HashType Hash() const {
    return std::make_tuple(drive_passage.lane_path().front().lane_id(),
                           borrow_lane, is_fallback);
  }

  // TODO(weijun): add constraints
};

struct RouteTargetInfo {
  int plan_id;
  KdTreeFrenetFrame frenet_frame;
  FrenetBox ego_frenet_box;

  DrivePassage drive_passage;
  PathSlBoundary sl_boundary;
  SpacetimeTrajectoryManager st_traj_mgr;
  std::optional<Vec2d> merge_point = std::nullopt;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_OUTPUT_H_
