#ifndef ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct FallbackPlannerInput {
  const PlannerSemanticMapManager* psmm;
  const PlanStartPointInfo* start_point_info;
  // The first point is assured to be plan start point.
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_trajectory;
  // Only has meaningful value if est parallel main loop is enabled.
  const mapping::LanePath* prev_target_lane_path_from_start;
  double prev_length_along_route;
  double prev_max_reach_length;
  const mapping::LanePoint* station_anchor;
  bool prev_smooth_state = false;
  const mapping::LanePath* prev_lane_path_before_lc;
  const RouteSectionsInfo* route_sections_info_from_start;
  const PlannerObjectManager* obj_mgr;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const SceneOutputProto* scene_reasoning;
  const LaneChangeStateProto* prev_lc_state;
  const TrafficLightStatesProto* traffic_light_states;
  const DeciderStateProto* pre_decider_state;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  const SmoothedReferenceLineResultMap* smooth_result_map;
  absl::Time parking_brake_release_time;
  bool teleop_enable_traffic_light_stop = false;
  bool enable_pull_over = false;
  std::optional<double> brake_to_stop = std::nullopt;
  const RouteSections* prev_route_sections = nullptr;
  std::optional<double> cruising_speed_limit = std::nullopt;
  bool enable_force_stop = false;
};

struct FallbackPlannerOutput {
  DeciderStateProto decider_state;
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  SpacetimeTrajectoryManager filtered_traj_mgr;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;

  SchedulerOutput scheduler_output;
  DiscretizedPath path;
  std::vector<PathPoint> st_path_points;
};

PlannerStatus RunFallbackPlanner(
    const FallbackPlannerInput& input, const VehicleParamApi& vehicle_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const DecisionConstraintConfigProto& decision_constraint_config,
    const FallbackPlannerParamsProto& fallback_planner_params,
    FallbackPlannerOutput* output, EstPlannerDebug* debug,
    vis::vantage::ChartDataBundleProto* chart_data, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_FALLBACK_PLANNER_H_
