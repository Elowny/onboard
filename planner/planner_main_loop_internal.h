#ifndef ONBOARD_PLANNER_PLANNER_MAIN_LOOP_INTERNAL_H_
#define ONBOARD_PLANNER_PLANNER_MAIN_LOOP_INTERNAL_H_

#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "common/proto/qalc.pb.h"

#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

std::shared_ptr<const ObjectsProto> GetAllObjects(
    const std::shared_ptr<const ObjectsProto>& real_objects,
    const std::shared_ptr<const ObjectsProto>& virtual_objects);

void FillTrajectoryProto(
    absl::Time plan_time,
    const std::vector<ApolloTrajectoryPointProto>& planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto>& past_points,
    const mapping::LanePath& target_lane_path_from_current,
    const LaneChangeStateProto& lane_change_state, TurnSignal turn_signal,
    const DoorDecision& door_decision, bool is_aeb_triggered,
    const DrivingStateProto& driving_state,
    const TrajectoryValidationResultProto& validate_result,
    TrajectoryProto* trajectory);

void ConvertPreviousTrajectoryToCurrentSmoothLateral(
    absl::Time predicted_plan_time, std::vector<PathPoint> previous_path,
    TrajectoryProto* previous_trajectory);

void ConvertPreviousPathToCurrentSmooth(
    const CoordinateConverter& coordinate_converter,
    const std::vector<PathPoint>& prev_path_global,
    std::vector<PathPoint>* prev_path);

void ConvertSmoothPathToGlobalCoordinates(
    const CoordinateConverter& coordinate_converter,
    std::vector<PathPoint>* path);

void ReportCandidateTrafficLightInfo(
    const TrafficLightInfoMap& traffic_light_map, PlannerDebugProto* debug);

void ReportSelectedTrafficLightInfo(
    const TrafficLightInfoMap& traffic_light_map,
    const mapping::LanePath& lane_path, TrafficLightInfoProto* proto);

// Prev traj includes past points. Prev traj will never be empty once the
// first successful planner iteration completes.
std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto& previous_trajectory,
    const MotionConstraintParamsProto& motion_constraint_params, bool reset);

PlanStartPointInfo ComputeACCPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,
    bool previously_triggered_aeb, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

PlanStartPointInfo ComputeEstPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state,
    bool previously_triggered_aeb, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

PlanStartPointInfo ComputeFreespacePlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto& prev_trajectory,
    const PoseProto& pose, const AutonomyStateProto& now_autonomy_state,
    const AutonomyStateProto& prev_autonomy_state, const Chassis& chassis,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params);

absl::Duration GetStPathPlanLookAheadTime(
    const PlanStartPointInfo& plan_start_point_info, const PoseProto& pose,
    absl::Duration planned_look_ahead_time,
    const TrajectoryProto& previous_trajectory);

StPathPlanStartPointInfo GetStPathPlanStartPointInfo(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo& plan_start_point_info,
    const TrajectoryProto& previous_trajectory,
    std::optional<double> trajectory_optimizer_time_step,
    std::optional<absl::Time> last_st_path_plan_start_time);

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections_from_start,
    const RouteNaviInfo& route_navi_info, mapping::ElementId ego_proj_lane_id,
    DriverAction::LaneChangeCommand lc_cmd);

void UpdatePreferredLanePath(const PlannerSemanticMapManager& psmm,
                             const RouteSections& route_sections_from_start,
                             const RouteNaviInfo& route_navi_info,
                             mapping::LanePath* preferred_lane_path,
                             QALCState* alc_state,
                             DriverAction::LaneChangeCommand* lc_cmd_state);

void HandleNewTeleopCommand(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections_from_start,
                            const RouteNaviInfo& route_navi_info,
                            mapping::ElementId ego_proj_lane_id,
                            DriverAction::LaneChangeCommand new_lc_cmd,
                            const mapping::LanePath& prev_target_lane_path,
                            const mapping::LanePath& prev_lp_before_lc,
                            const LaneChangeStateProto& prev_lc_state,
                            TurnSignal selector_prep_turn_signal,
                            mapping::LanePath* preferred_lane_path,
                            QALCState* alc_state,
                            DriverAction::LaneChangeCommand* lc_cmd_state);

void HandleAlcUserResponse(const PlannerSemanticMapManager& psmm,
                           const RouteSections& route_sections_from_start,
                           const RouteNaviInfo& route_navi_info,
                           const mapping::LanePath& prev_target_lane_path,
                           const LaneChangeStateProto& prev_lc_state,
                           std::optional<bool> alc_confirmation,
                           std::optional<bool> alc_request_lc_left,
                           mapping::LanePath* preferred_lane_path,
                           QALCState* alc_state,
                           DriverAction::LaneChangeCommand* lc_cmd_state);

// first: route sections from pos.
// second: route sections including behind parts.
absl::StatusOr<std::tuple<RouteSections, RouteSections, PointOnRouteSections>>
ProjectPointToRouteSections(const PlannerSemanticMapManager& psmm,
                            const RouteSections& route_sections,
                            const Vec2d& pos, double projection_range,
                            double keep_behind_length);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_MAIN_LOOP_INTERNAL_H_
