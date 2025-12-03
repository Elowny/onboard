#ifndef ONBOARD_PLANNER_ML_EXPERT_EXPERT_PLANNER_H_
#define ONBOARD_PLANNER_ML_EXPERT_EXPERT_PLANNER_H_

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
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct ExpertPlannerInput {
  const PlannerSemanticMapManager* psmm;
  const PlanStartPointInfo* start_point_info;
  const mapping::LanePath* prev_target_lane_path_from_start;
  const mapping::LanePath* prev_lane_path_before_lc_from_start;
  const LaneChangeStateProto* prev_lc_state;
  const mapping::LanePoint* station_anchor;
  const RouteSectionsInfo* sections_info_from_start;
  const PlannerObjectManager* obj_mgr;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const SceneOutputProto* scene_reasoning;
  const TrafficLightStatesProto* traffic_light_states;
  const DeciderStateProto* pre_decider_state;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  absl::Time parking_brake_release_time;
  bool teleop_enable_traffic_light_stop = false;
  bool enable_pull_over = false;
  std::optional<double> brake_to_stop = std::nullopt;
  const std::vector<LanePathInfo>* lp_infos;
  const PlannerParamsProto* planner_params;
  const VehicleParamApi* vehicle_params;
  const SmoothedReferenceLineResultMap* smooth_result_map;
  const RouteNaviInfo* route_navi_info = nullptr;
  bool prev_smooth_state = false;
  const RouteSections* prev_route_sections = nullptr;
  const TrajectoryProto* log_av_trajectory = nullptr;
  std::optional<double> cruising_speed_limit = std::nullopt;
  bool enable_force_stop = false;
  AutonomyStateProto::State autonomy_state = AutonomyStateProto::NOT_READY;
};

struct ExpertPlannerOutput {
  DeciderStateProto decider_state;
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  SpacetimeTrajectoryManager filtered_traj_mgr;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;

  SchedulerOutput scheduler_output;
  DiscretizedPath path;
  std::vector<PathPoint> st_path_points;
};

// Expert Planner is for reconstruction of algorithm intermediates(boundaries,
// considered_st_objs, etc...)
//  given a manual driven trajectory. It could be used for onboard feature
//  dumping of manual driven trajectory or validation of intermediates
//  construction
PlannerStatus RunExpertPlanner(const ExpertPlannerInput& input,
                               ExpertPlannerOutput* output,
                               ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_EXPERT_PLANNER_H_
