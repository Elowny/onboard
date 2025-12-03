#ifndef ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
#define ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_input.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

absl::StatusOr<std::vector<SchedulerOutput>> ScheduleMultiplePlanTasks(
    const MultiTasksSchedulerInput& input,
    const std::vector<LanePathInfo>& target_lp_infos, ThreadPool* thread_pool);

absl::StatusOr<SchedulerOutput> MakeSchedulerOutput(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& route_sections_info,
    const std::vector<LanePathInfo>& lp_infos, DrivePassage drive_passage,
    const LanePathInfo& lp_info, const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state,
    const RouteNaviInfo& route_navi_info, bool borrow, bool should_smooth,
    bool planner_is_l4_mode, AutonomyStateProto::State autonomy_state);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_MULTI_TASKS_SCHEDULER_H_
