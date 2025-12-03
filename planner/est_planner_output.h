#ifndef ONBOARD_PLANNER_EST_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_EST_PLANNER_OUTPUT_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/leading_groups_builder.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/ml/captain_net/proto/captain_net_debug.pb.h"
#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_planner_object_trajectories.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/hmi_content_util.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/turn_signal.pb.h"

namespace qcraft {
namespace planner {

struct EstPlannerDebug {
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  FilteredTrajectories filtered_prediction_trajectories;
  ConstraintProto decision_constraints;
  InitializerDebugProto initializer_debug_proto;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;
  ml::CapnetTrajectoryDebugProto capnet_traj_debug;
  SpeedFinderDebugProto speed_finder_debug;
  TrajectoryValidationResultProto traj_validation_result;
  LaneChangeSafetyDebugProto lane_change_safety_debug_proto;
};

struct EstPlannerOutput {
  SchedulerOutput scheduler_output;
  DiscretizedPath path;
  LeadingGroup leading_trajs;
  absl::flat_hash_set<std::string> follower_set;
  absl::flat_hash_set<std::string> unsafe_object_ids;
  double follower_max_decel = 0.0;  // Should be larger than or equal to 0.0;
  // Final trajectory.
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<PathPoint> st_path_points;

  // The nearest stop line's s on existence.
  std::optional<double> first_stop_s = std::nullopt;
  std::optional<mapping::ElementId> redlight_lane_id = std::nullopt;

  // Return planner state that needs to persist here.
  DeciderStateProto decider_state;

  // Return planner state that needs to persist here.
  InitializerStateProto initializer_state;

  // State of trajectory optimizer.
  // Return planner state that needs to persist here.
  TrajectoryOptimizerStateProto trajectory_optimizer_state_proto;

  // To be filled into planner_state.
  SpacetimePlannerObjectTrajectoriesProto st_planner_object_trajectories;
  // Trajectories considered by speed.
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;

  // Optimizer Auto Tuning
  AutoTuningTrajectoryProto candidate_auto_tuning_traj_proto;
  AutoTuningTrajectoryProto expert_auto_tuning_traj_proto;

  // For hmi display.
  std::optional<std::string> alerted_front_vehicle = std::nullopt;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  std::optional<NudgeOjbectInfo> nudge_object_info;
  bool collision_warning_request = false;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_EST_PLANNER_OUTPUT_H_
