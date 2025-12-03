#ifndef ONBOARD_PLANNER_PLAN_PLAN_TASK_HELPER_H_
#define ONBOARD_PLANNER_PLAN_PLAN_TASK_HELPER_H_

#include <deque>
#include <optional>
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/proto/plan_task.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

std::deque<PlanTask> CreatePlanTasksQueueFromRoutingResult(
    const RouteManagerOutput& route_output,
    const PlannerSemanticMapManager& psmm);

bool PlanTaskCompeleted(const PlanTask& task,
                        const AutonomyStateProto& autonomy_state,
                        const CoordinateConverter& cc,
                        const Vec2d& front_bumper_pos, double ego_heading,
                        double ego_v, const PlannerSemanticMapManager& psmm);

absl::StatusOr<std::vector<PlanTask>> SplitCruiseByUturnTask(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& prev_target_lane_path,
    const mapping::LanePoint& route_destination);

absl::StatusOr<std::vector<PlanTask>> CreateUturnTask(
    const PlannerSemanticMapManager& psmm, const PoseProto& ego_pose,
    const mapping::LanePath& prev_target_lane_path,
    const mapping::LanePoint& route_destination,
    const std::optional<TrajectoryEndInfoProto>& prev_traj_end_info,
    const TrajectoryProto& prev_traj);

absl::StatusOr<PlanTask> CreateBlockedRoadTask(
    const PlannerSemanticMapManager& psmm, const PoseProto& pose,
    const VehicleGeometryParamsProto& veh_geo_params);

bool ReachedRouteEnd(const Vec2d& ego_pos, double ego_v,
                     const PlanTask& last_task, const CoordinateConverter& cc,
                     const PlannerSemanticMapManager& psmm);

bool IsHdMapBasedTask(PlanTaskType type);

PlannerStatus ClassifyTaskErrorToPlannerStatus(PlanTaskType type,
                                               std::string_view error_msg,
                                               bool is_driverless_mode);

struct PreprocessPlannerObjectsOutput {
  ObjectVector<PlannerObject> planner_objects;
  ConflictResolverDebugProto prediction_post_process;
};

PreprocessPlannerObjectsOutput PreprocessPlannerObjects(
    const PlannerSemanticMapManager& psmm,
    const TrafficLightStatesProto& tl_states,
    const prediction::ConflictResolverParams& resolver_params,
    const ObjectsPredictionProto* prediction, const ObjectsProto* objects_proto,
    absl::Time plan_time, bool planner_consider_objects,
    ThreadPool* thread_pool);

absl::StatusOr<GlobalPose> CalculateGoalPoseByDestinationInfo(
    const PlannerSemanticMapManager& psmm, const CoordinateConverter& cc,
    const PlanTaskDestinationInfo& destination_info);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_PLAN_TASK_HELPER_H_
