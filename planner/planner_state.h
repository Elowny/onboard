#ifndef ONBOARD_PLANNER_PLANNER_STATE_H_
#define ONBOARD_PLANNER_PLANNER_STATE_H_

#include <stdint.h>

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/time/time.h"

#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/assist/tja_state.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {
// The cross iteration states of planner.

struct PlannerState {
  LiteHeader header;

  int planner_frame_seq_num;

  // TODO(jiayu): Rename this struct name, after delete tl_history manager.
  YellowLightObservationsNew yellow_light_observations;

  // Previous planned trajectory.
  TrajectoryProto previous_trajectory;

  // Don't clear in cruise_task.
  std::vector<PathPoint> previous_st_path_global_including_past;

  // Audio playing
  absl::Time last_audio_alert_time;

  // Parking brake release time.
  absl::Time parking_brake_release_time;

  LaneChangeStateProto lane_change_state;

  // Record how many loops were skipped between this and previous trajectory.
  int planner_skip_counter = 0;

  InputSeqNum input_seq_num;

  AutonomyStateProto previous_autonomy_state;

  int previous_trajectory_plan_counter = 0;

  int version = 4;  // When snapshot upgrade, change version

  double last_door_override_time = 0.0;

  absl::Time current_time;

  int64_t route_update_id = -1;

  void FromProto(const PlannerStateProto& proto);

  void ToProto(PlannerStateProto* proto) const;

  bool Upgrade();  // Used for snapshot version compatible

  bool operator==(const PlannerState& other) const;

  bool operator!=(const PlannerState& other) const { return !(*this == other); }

  std::string DebugString() const;

  // Freespace planner state.
  FreespacePlannerStateProto freespace_planner_state;

  // ------------- Planner 3.0 -----------------
  mapping::LanePath prev_lane_path_before_lc;
  // State of decider
  DeciderStateProto decider_state;
  // State of initializer
  InitializerStateProto initializer_state;
  // Spacetime planner object
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;

  // ---------------- Planner 3.5 --------------
  // ---------- Multiple trajectories ----------

  // Previous target lane path starting from the position that is slightly
  // behind plan start point: (1) Insure successful projection as plan start
  // point might jump backwards due to resetting. (2) Keep decision
  // consistency.
  mapping::LanePath prev_target_lane_path;
  // Only for fallback planner.
  double prev_length_along_route = std::numeric_limits<double>::max();
  double prev_max_reach_length = std::numeric_limits<double>::max();
  // For stabilization of drive passage stations across frames.
  mapping::LanePoint station_anchor;

  // For teleop lane change.
  mapping::LanePath preferred_lane_path;

  // Route sections starting from the point that is several meters behind plan
  // start point.
  RouteSections prev_route_sections;

  // Reference line smooth.
  bool prev_smooth_state = false;

  MissionStageProto mission_stage;

  std::deque<PlanTask> plan_task_queue;

  SelectorState selector_state;

  std::optional<TrajectoryEndInfoProto> prev_traj_end_info;
  std::optional<TrajectoryOptimizerStateProto>
      selected_trajectory_optimizer_state_proto;

  SmoothedReferenceLineResultMap smooth_result_map;

  TjaState tja_state;

  bool stopped_at_route_end = false;

  AsyncPlannerState async_planner_state;

  bool previously_triggered_aeb = false;

  struct ObjectMotionState {
    double heading = 0.0;
    Vec2d accel;
    Vec2d vel;
    Vec2d pos;
  };

  using ObjectsHistoryMap =
      absl::flat_hash_map<std::string,
                          HistoryBufferAbslTime<ObjectMotionState>>;
  ObjectsHistoryMap object_history_map;

  struct HdMapState {
    double load_distance;
    bool has_destination;
  };
  std::optional<HdMapState> hd_map_state = std::nullopt;

  bool prev_collision_warning_request;

  // ----------------- L2 related ----------------
  AssistPlanStateProto assist_plan_state;

  std::shared_ptr<const PlannerSemanticMapManager> prev_low_freq_psmm = nullptr;

  int64_t prev_online_map_id = kInvalidOnlineMapId;

  HistoryTransformProto history_transform;

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_STATE_H_
