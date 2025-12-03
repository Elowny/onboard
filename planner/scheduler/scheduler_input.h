#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_INPUT_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_INPUT_H_

#include <string>
#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct MultiTasksSchedulerInput {
  const PlannerSemanticMapManager* psmm;
  const mapping::OnlineSemanticMapProto* online_semantic_map;
  const VehicleGeometryParamsProto* vehicle_geom;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const std::vector<LanePathInfo>* lane_path_infos;
  const RouteSectionsInfo* sections_info_from_current;
  const TrafficLightInfoMap* tl_info_map;
  bool prev_smooth_state = false;
  const ApolloTrajectoryPointProto* plan_start_point;
  const mapping::LanePoint* station_anchor;
  double start_route_s;
  const SmoothedReferenceLineResultMap* smooth_result_map;
  const RouteSections* prev_route_sections = nullptr;
  const mapping::LanePath* prev_target_lane_path_from_start = nullptr;
  const mapping::LanePath* prev_lane_path_before_lc_from_start = nullptr;
  const LaneChangeStateProto* prev_lc_state = nullptr;
  const RouteNaviInfo* route_navi_info = nullptr;
  std::optional<double> cruising_speed_limit = std::nullopt;
  bool planner_is_l4_mode = true;
  AutonomyStateProto::State autonomy_state = AutonomyStateProto::NOT_READY;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_INPUT_H_
