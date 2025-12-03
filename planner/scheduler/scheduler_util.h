#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/proto/scheduler.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/proto/autonomy_state.pb.h"

namespace qcraft::planner {

// Creates a lane change state proto from a lane change stage. The rest fields
// use default settings.
LaneChangeStateProto MakeNoneLaneChangeState();

bool ShouldSmoothRefLane(const TrafficLightInfoMap& tl_info_map,
                         const DrivePassage& dp, bool prev_smooth_state);

double CalcAvhRefCenterL(const PlannerSemanticMapManager& psmm,
                         const DrivePassage& drive_passage,
                         const FrenetBox& ego_frenet_box,
                         const SmoothedReferenceLineResultMap& smooth_res_map,
                         bool should_smooth);

absl::StatusOr<LaneChangeStateProto> MakeLaneChangeState(
    const DrivePassage& drive_passage, const Vec2d& ego_pos,
    const FrenetBox& ego_frenet_box,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state, double ref_center_l,
    AutonomyStateProto::State autonomy_state);

void ToSchedulerOutputProto(const SchedulerOutput& output,
                            SchedulerOutputProto* proto);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_UTIL_H_
